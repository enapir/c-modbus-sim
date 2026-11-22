/*
 * Modbus Slave Simulator - C port of modbus_sim.py for OpenWrt / embedded Linux
 *
 * Loads a YAML config and starts a Modbus TCP or RTU/ASCII slave server.
 * No external dependencies (no Python, libyaml or libmodbus): a YAML subset
 * parser, the Modbus slave stack and serial handling are all built in.
 * Builds with any C99 compiler and POSIX libc (musl / glibc / uClibc).
 *
 * Usage examples:
 *   # TCP server (default port 502)
 *   modbus-sim sample_config.yaml
 *
 *   # TCP server on custom host/port
 *   modbus-sim sample_config.yaml --host 127.0.0.1 --port 5020
 *
 *   # RTU server on serial port
 *   modbus-sim sample_config.yaml --mode rtu --serial-port /dev/ttyUSB0 --baudrate 9600
 *
 *   # Verbose: show register dump on startup
 *   modbus-sim sample_config.yaml --verbose
 *
 * Supported YAML subset: block mappings (indentation), comments, plain /
 * quoted scalars, flow mappings of scalars ({0x10: 1, 0x11: 2}), and
 * YAML 1.1 integers (decimal, 0x hex, 0b binary, 0 octal, '_' separators).
 * Block scalars (| >) and lists under unrelated keys are skipped.
 * Anchors / aliases / merge keys are rejected with an error.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <netdb.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>

#define VERSION "1.0.0"

#define MAX_PDU      253
#define MAX_RTU_ADU  256
#define MAX_TCP_ADU  260
#define MAX_CLIENTS  64
#define NREGS        65536

/* ---------------------------------------------------------------------------
 * Options
 * ------------------------------------------------------------------------- */

static struct {
    const char *yaml;
    int rtu;
    const char *host;
    int port;
    const char *serial_port;
    int baudrate, bytesize, stopbits;
    char parity;
    double timeout;
    int ascii;
    int local_echo;
    double reconnect_delay;
    int multi;
    int verbose, debug_requests;
} g_opt = {
    .host = "0.0.0.0", .port = 502,
    .serial_port = "/dev/ttyS0", .baudrate = 9600, .bytesize = 8, .stopbits = 1,
    .parity = 'N', .timeout = 3.0, .reconnect_delay = 2.0,
};

static volatile sig_atomic_t g_stop;

/* ---------------------------------------------------------------------------
 * Logging  ("%H:%M:%S [LEVEL] modbus-sim: message" on stderr, like Python)
 * ------------------------------------------------------------------------- */

enum { L_DEBUG, L_INFO, L_WARNING, L_ERROR };
static const char *const LEVEL_NAMES[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
static int g_log_level = L_INFO;

static void log_prefix(int lvl)
{
    char ts[16];
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    strftime(ts, sizeof ts, "%H:%M:%S", &tm);
    fprintf(stderr, "%s [%s] modbus-sim: ", ts, LEVEL_NAMES[lvl]);
}

static void vlog(int lvl, const char *fmt, va_list ap)
{
    log_prefix(lvl);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
}

static void logm(int lvl, const char *fmt, ...)
{
    va_list ap;
    if (lvl < g_log_level)
        return;
    va_start(ap, fmt);
    vlog(lvl, fmt, ap);
    va_end(ap);
}

/* Frame dumps: shown with --debug-requests even when --log-level is INFO. */
static void log_frame(const char *tag, const uint8_t *b, int n)
{
    if (!g_opt.debug_requests && g_log_level > L_DEBUG)
        return;
    log_prefix(L_DEBUG);
    fprintf(stderr, "%s:", tag);
    for (int i = 0; i < n; i++)
        fprintf(stderr, " %02x", b[i]);
    fputc('\n', stderr);
}

static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vlog(L_ERROR, fmt, ap);
    va_end(ap);
    exit(1);
}

static void *xcalloc(size_t n, size_t sz)
{
    void *p = calloc(n, sz);
    if (!p)
        die("out of memory");
    return p;
}

static char *xstrdup(const char *s)
{
    char *p = strdup(s);
    if (!p)
        die("out of memory");
    return p;
}

/* ---------------------------------------------------------------------------
 * Minimal YAML subset parser
 * ------------------------------------------------------------------------- */

typedef struct ynode {
    char *key;              /* NULL for the root */
    char *val;              /* NULL when the node is a mapping or null */
    int indent, line;
    struct ynode *child, *last, *next;
} ynode;

static ynode *ynew(const char *key, int indent, int line)
{
    ynode *n = xcalloc(1, sizeof *n);
    n->key = key ? xstrdup(key) : NULL;
    n->indent = indent;
    n->line = line;
    return n;
}

static void yadd(ynode *parent, ynode *c)
{
    if (parent->last)
        parent->last->next = c;
    else
        parent->child = c;
    parent->last = c;
}

static void yfree(ynode *n)
{
    while (n) {
        ynode *next = n->next;
        yfree(n->child);
        free(n->key);
        free(n->val);
        free(n);
        n = next;
    }
}

static char *rtrim(char *s)
{
    size_t n = strlen(s);
    while (n && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r'))
        s[--n] = 0;
    return s;
}

static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t')
        s++;
    return rtrim(s);
}

static int quote_start(const char *s, const char *p)
{
    return p == s || p[-1] == ' ' || p[-1] == '\t' || p[-1] == ':' ||
           p[-1] == '{' || p[-1] == ',';
}

/* Cut a '#' comment (start of line or after whitespace, outside quotes). */
static void strip_comment(char *s)
{
    char q = 0;
    for (char *p = s; *p; p++) {
        if (q) {
            if (*p == q)
                q = 0;
        } else if ((*p == '\'' || *p == '"') && quote_start(s, p)) {
            q = *p;
        } else if (*p == '#' && (p == s || p[-1] == ' ' || p[-1] == '\t')) {
            *p = 0;
            return;
        }
    }
}

/* The ':' separating key and value (followed by blank or EOL), outside quotes. */
static char *find_colon(char *s)
{
    char q = 0;
    for (char *p = s; *p; p++) {
        if (q) {
            if (*p == q)
                q = 0;
        } else if ((*p == '\'' || *p == '"') && p == s) {
            q = *p;
        } else if (*p == ':' && (p[1] == 0 || p[1] == ' ' || p[1] == '\t')) {
            return p;
        }
    }
    return NULL;
}

static char *unquote(char *s)
{
    size_t n = strlen(s);
    if (n >= 2 && (s[0] == '\'' || s[0] == '"') && s[n - 1] == s[0]) {
        s[n - 1] = 0;
        return s + 1;
    }
    return s;
}

static int is_null(const char *v)
{
    return !*v || !strcmp(v, "~") || !strcmp(v, "null") ||
           !strcmp(v, "Null") || !strcmp(v, "NULL");
}

static void ysetval(ynode *n, char *v, const char *file, int line)
{
    if (is_null(v))
        return;
    if (*v == '&' || *v == '*')
        die("%s:%d: YAML anchors/aliases are not supported", file, line);
    n->val = xstrdup(unquote(v));
}

/* {k: v, k: v} -> child nodes (flat scalars only). */
static void parse_flow(ynode *n, char *v, const char *file, int line)
{
    size_t len = strlen(v);
    char *p, *item;
    if (v[len - 1] != '}')
        die("%s:%d: multi-line flow mappings are not supported", file, line);
    v[len - 1] = 0;
    p = v + 1;
    if (strpbrk(p, "{}[]"))
        die("%s:%d: nested flow collections are not supported", file, line);
    while ((item = strsep(&p, ",")) != NULL) {
        char *c, *k;
        ynode *ch;
        item = trim(item);
        if (!*item)
            continue;
        c = find_colon(item);
        if (c)
            *c = 0;
        k = unquote(rtrim(item));
        ch = ynew(k, n->indent + 1, line);
        if (c)
            ysetval(ch, trim(c + 1), file, line);
        yadd(n, ch);
    }
}

static ynode *yaml_parse(char *text, const char *file)
{
    ynode *root = ynew(NULL, -1, 0);
    ynode *stack[64];
    int sp = 0, lineno = 0, block_indent = -1;
    char *p = text;

    stack[0] = root;
    if (!strncmp(p, "\xEF\xBB\xBF", 3))
        p += 3;

    while (p && *p) {
        char *line = p, *s, *nl = strchr(p, '\n');
        char *key, *val;
        int ind = 0;
        ynode *n;

        if (nl) {
            *nl = 0;
            p = nl + 1;
        } else {
            p = NULL;
        }
        lineno++;
        rtrim(line);

        while (line[ind] == ' ')
            ind++;
        s = line + ind;
        if (block_indent >= 0) {          /* inside a | or > block scalar */
            if (!*s || ind > block_indent)
                continue;
            block_indent = -1;
        }
        if (*s == '\t')
            die("%s:%d: tab characters are not allowed in YAML indentation", file, lineno);
        strip_comment(s);
        rtrim(s);
        if (!*s)
            continue;
        if (ind == 0 && (!strncmp(s, "---", 3) || !strcmp(s, "...") || *s == '%'))
            continue;

        while (sp > 0 && stack[sp]->indent >= ind)
            sp--;

        if (s[0] == '-' && (s[1] == 0 || s[1] == ' ')) {
            key = "-";                     /* list item: kept, never interpreted */
            val = trim(s + 1);
        } else {
            char *c = find_colon(s);
            if (!c) {
                logm(L_WARNING, "%s:%d: unsupported YAML syntax ignored: %s", file, lineno, s);
                continue;
            }
            *c = 0;
            key = unquote(rtrim(s));
            val = trim(c + 1);
            if (!strcmp(key, "<<"))
                die("%s:%d: YAML merge keys are not supported", file, lineno);
        }

        n = ynew(key, ind, lineno);
        yadd(stack[sp], n);
        if (*val == '|' || *val == '>') {
            n->val = xstrdup("");
            block_indent = ind;
        } else if (*val == '{') {
            parse_flow(n, val, file, lineno);
        } else {
            ysetval(n, val, file, lineno);
        }
        if (sp + 1 >= (int)(sizeof stack / sizeof stack[0]))
            die("%s:%d: YAML nesting too deep", file, lineno);
        stack[++sp] = n;
    }
    return root;
}

/* Last child with the given key (PyYAML: duplicate keys, last one wins). */
static ynode *ychild(const ynode *n, const char *key)
{
    ynode *found = NULL;
    for (ynode *c = n ? n->child : NULL; c; c = c->next)
        if (c->key && !strcmp(c->key, key))
            found = c;
    return found;
}

/* Integer scalar as PyYAML (YAML 1.1) + Python int() would read it. */
static int yaml_int(const char *str, long long *out)
{
    char buf[80], *e;
    const char *s;
    size_t k = 0;
    int neg = 0, base = 10;
    unsigned long long u;

    if (!str)
        return -1;
    for (s = str; *s && k + 1 < sizeof buf; s++)
        if (*s != '_')
            buf[k++] = *s;
    buf[k] = 0;
    if (!k)
        return -1;

    if (!strcmp(buf, "true") || !strcmp(buf, "True") || !strcmp(buf, "TRUE") ||
        !strcmp(buf, "yes") || !strcmp(buf, "Yes") || !strcmp(buf, "YES") ||
        !strcmp(buf, "on") || !strcmp(buf, "On") || !strcmp(buf, "ON")) {
        *out = 1;
        return 0;
    }
    if (!strcmp(buf, "false") || !strcmp(buf, "False") || !strcmp(buf, "FALSE") ||
        !strcmp(buf, "no") || !strcmp(buf, "No") || !strcmp(buf, "NO") ||
        !strcmp(buf, "off") || !strcmp(buf, "Off") || !strcmp(buf, "OFF")) {
        *out = 0;
        return 0;
    }

    s = buf;
    if (*s == '-' || *s == '+')
        neg = *s++ == '-';
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
    } else if (s[0] == '0' && s[1] == 'b') {
        base = 2;
        s += 2;
    } else if (s[0] == '0' && s[1] && strspn(s, "01234567") == strlen(s)) {
        base = 8;
    }
    if (*s && strchr("0123456789abcdefABCDEF", *s)) {
        errno = 0;
        u = strtoull(s, &e, base);
        if (!*e && !errno) {
            *out = neg ? -(long long)u : (long long)u;
            return 0;
        }
    }

    /* float -> truncated like int(1.5) */
    {
        double d;
        errno = 0;
        d = strtod(buf, &e);
        if (!*e && !errno && isfinite(d) && strpbrk(buf, ".eE") &&
            fabs(d) < 9.2e18) {
            *out = (long long)d;
            return 0;
        }
    }
    return -1;
}

/* ---------------------------------------------------------------------------
 * Slaves / register data
 * ------------------------------------------------------------------------- */

typedef struct {
    int id;
    unsigned base;          /* first register address */
    unsigned count;         /* number of registers */
    uint16_t *regs;         /* shared by FC03 and FC04 */
    uint8_t coil, discrete; /* single bit at address 0 (FC01/02/05/15) */
} slave_t;

static slave_t *g_slaves[256];

/* Per-FC flattening scratch. tag 0 = unset, 1 = flat entry, 2+ = section. */
typedef struct {
    uint16_t tag[NREGS];
    long long val[NREGS];
    uint8_t cset[NREGS];
    uint16_t cval[NREGS];
} scratch_t;

static scratch_t *g_scr;

static void put_reg(const ynode *e, unsigned tag, int warn, int *n)
{
    long long addr, v;
    if (yaml_int(e->key, &addr) < 0) {
        if (warn)
            logm(L_WARNING, "line %d: non-integer address %s ignored", e->line, e->key);
        return;
    }
    if (e->child || !e->val || yaml_int(e->val, &v) < 0) {
        if (warn)
            logm(L_WARNING, "line %d: address %s has no integer value, ignored", e->line, e->key);
        return;
    }
    if (addr < 0 || addr >= NREGS) {
        if (warn)
            logm(L_WARNING, "line %d: address %lld out of range 0-65535, ignored", e->line, addr);
        return;
    }
    if (g_scr->tag[addr] && g_scr->tag[addr] != tag) {
        if (warn)
            logm(L_WARNING, "Duplicate address %lld overrides earlier value", addr);
    }
    if (!g_scr->tag[addr])
        (*n)++;
    g_scr->tag[addr] = (uint16_t)tag;
    g_scr->val[addr] = v;
}

/* Flatten optional one-level named sections into g_scr->tag/val.
 *
 * holding_registers / input_registers accept either a flat {address: value}
 * mapping or named sections {section: {address: value}} (e.g. one section
 * per source .mbs file). Both styles can be mixed. Returns register count.
 */
static int flatten_sections(const ynode *fc, int warn)
{
    unsigned tag = 1;
    int n = 0;

    memset(g_scr->tag, 0, sizeof g_scr->tag);
    if (!fc)
        return 0;
    if (fc->val && warn)
        logm(L_WARNING, "line %d: %s is not a mapping, ignored", fc->line, fc->key);

    for (const ynode *c = fc->child; c; c = c->next) {
        if (!c->child) {
            put_reg(c, 1, warn, &n);
            continue;
        }
        /* a later section with the same name replaces this one entirely */
        int shadowed = 0;
        for (const ynode *d = c->next; d; d = d->next)
            if (d->child && !strcmp(d->key, c->key))
                shadowed = 1;
        tag++;
        if (shadowed)
            continue;
        for (const ynode *e = c->child; e; e = e->next)
            put_reg(e, tag, warn, &n);
    }
    return n;
}

/* Slave entries keyed by integer id, skipping ones overridden by a later duplicate. */
static int slave_id_of(const ynode *s, int *id)
{
    long long v, w;
    if (yaml_int(s->key, &v) < 0)
        return -1;
    for (const ynode *d = s->next; d; d = d->next)
        if (yaml_int(d->key, &w) == 0 && w == v)
            return -2;
    *id = (int)v;
    return v < 0 || v > 255 ? -3 : 0;
}

static void show_registers(const ynode *slaves)
{
    static const char *const fcs[][2] = {
        { "holding_registers", "FC3 Holding Registers" },
        { "input_registers", "FC4 Input Registers" },
    };
    for (const ynode *s = slaves ? slaves->child : NULL; s; s = s->next) {
        int sid;
        if (slave_id_of(s, &sid) != 0 || !s->child)
            continue;
        printf("\n=== Slave %d ===\n", sid);
        for (int f = 0; f < 2; f++) {
            int n = flatten_sections(ychild(s, fcs[f][0]), 0), lo = -1, hi = -1;
            if (!n)
                continue;
            for (int a = 0; a < NREGS; a++)
                if (g_scr->tag[a]) {
                    if (lo < 0)
                        lo = a;
                    hi = a;
                }
            printf("  [%s]  (%d registers)  range 0x%04X-0x%04X\n", fcs[f][1], n, lo, hi);
            for (int a = lo; a <= hi; a++)
                if (g_scr->tag[a])
                    printf("    0x%04X (%5d) = %6lld  (0x%04X)\n", a, a, g_scr->val[a],
                           (unsigned)(g_scr->val[a] & 0xFFFF));
        }
    }
    fflush(stdout);
}

static void build_slave(int id, const ynode *cfg)
{
    int nh, ni, lo = -1, hi = -1, nc = 0;
    slave_t *s;

    memset(g_scr->cset, 0, sizeof g_scr->cset);
    /* 実機 PCS は holding/input を同一アドレス空間で扱い、FC03(ReadHolding) と
     * FC04(ReadInput) の双方で同じデータを返す。設定(holding)＋監視(input)の
     * 和集合を両 FC で配信する (重複時は input_registers が優先)。 */
    nh = flatten_sections(cfg ? ychild(cfg, "holding_registers") : NULL, 1);
    for (int a = 0; a < NREGS; a++)
        if (g_scr->tag[a]) {
            g_scr->cset[a] = 1;
            g_scr->cval[a] = (uint16_t)g_scr->val[a];
        }
    ni = flatten_sections(cfg ? ychild(cfg, "input_registers") : NULL, 1);
    for (int a = 0; a < NREGS; a++)
        if (g_scr->tag[a]) {
            g_scr->cset[a] = 1;
            g_scr->cval[a] = (uint16_t)g_scr->val[a];
        }
    for (int a = 0; a < NREGS; a++)
        if (g_scr->cset[a]) {
            if (lo < 0)
                lo = a;
            hi = a;
            nc++;
        }

    logm(L_INFO, "Slave %d: %d holding regs, %d input regs (combined %d, served on FC03+FC04)",
         id, nh, ni, nc);

    s = xcalloc(1, sizeof *s);
    s->id = id;
    if (lo < 0) {                       /* placeholder: one zero register at 0 */
        s->base = 0;
        s->count = 1;
        s->regs = xcalloc(1, sizeof(uint16_t));
    } else {
        s->base = (unsigned)lo;
        s->count = (unsigned)(hi - lo + 1);
        s->regs = xcalloc(s->count, sizeof(uint16_t));
        for (int a = lo; a <= hi; a++)
            s->regs[a - lo] = g_scr->cval[a];
    }
    if (g_slaves[id]) {
        free(g_slaves[id]->regs);
        free(g_slaves[id]);
    }
    g_slaves[id] = s;
}

static void build_slaves(const ynode *slaves)
{
    int count = 0;
    if (slaves && slaves->val)
        logm(L_WARNING, "line %d: 'slaves' is not a mapping, ignored", slaves->line);
    for (const ynode *s = slaves ? slaves->child : NULL; s; s = s->next) {
        int sid, r = slave_id_of(s, &sid);
        if (r == -1) {
            logm(L_WARNING, "Skipping slave with non-integer ID '%s'", s->key);
            continue;
        }
        if (r == -2)
            continue;
        if (r == -3) {
            logm(L_WARNING, "Skipping slave ID %s (must be 0-255)", s->key);
            continue;
        }
        build_slave(sid, s->child ? s : NULL);
        count++;
    }
    if (!count) {
        logm(L_WARNING, "No slaves in config; creating empty slave ID 1");
        build_slave(1, NULL);
    }
}

/* ---------------------------------------------------------------------------
 * Modbus PDU processing
 * ------------------------------------------------------------------------- */

enum {
    EX_ILLEGAL_FUNCTION = 0x01,
    EX_ILLEGAL_ADDRESS = 0x02,
    EX_ILLEGAL_VALUE = 0x03,
    EX_DEVICE_FAILURE = 0x04,
};

static unsigned get16(const uint8_t *p) { return (unsigned)p[0] << 8 | p[1]; }

static void put16(uint8_t *p, unsigned v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static int exc(uint8_t *rsp, uint8_t fc, uint8_t code)
{
    rsp[0] = fc | 0x80;
    rsp[1] = code;
    return 2;
}

static int in_range(const slave_t *s, unsigned addr, unsigned qty)
{
    return addr >= s->base && addr + qty <= s->base + s->count;
}

/* req/len: request PDU (function code + data). rsp: >= MAX_PDU bytes.
 * Returns the response PDU length. */
static int handle_pdu(slave_t *s, const uint8_t *req, int len, uint8_t *rsp)
{
    uint8_t fc = req[0];
    unsigned addr, qty, bc, v;

    rsp[0] = fc;
    switch (fc) {
    case 0x01: /* Read Coils */
    case 0x02: /* Read Discrete Inputs */
        if (len != 5)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        addr = get16(req + 1);
        qty = get16(req + 3);
        if (qty < 1 || qty > 2000)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        if (addr != 0 || qty != 1)
            return exc(rsp, fc, EX_ILLEGAL_ADDRESS);
        rsp[1] = 1;
        rsp[2] = fc == 0x01 ? s->coil : s->discrete;
        return 3;

    case 0x03: /* Read Holding Registers */
    case 0x04: /* Read Input Registers */
        if (len != 5)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        addr = get16(req + 1);
        qty = get16(req + 3);
        if (qty < 1 || qty > 125)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        if (!in_range(s, addr, qty))
            return exc(rsp, fc, EX_ILLEGAL_ADDRESS);
        rsp[1] = (uint8_t)(qty * 2);
        for (unsigned i = 0; i < qty; i++)
            put16(rsp + 2 + i * 2, s->regs[addr - s->base + i]);
        return 2 + (int)qty * 2;

    case 0x05: /* Write Single Coil */
        if (len != 5)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        addr = get16(req + 1);
        v = get16(req + 3);
        if (v != 0xFF00 && v != 0x0000)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        if (addr != 0)
            return exc(rsp, fc, EX_ILLEGAL_ADDRESS);
        s->coil = v ? 1 : 0;
        memcpy(rsp, req, 5);
        return 5;

    case 0x06: /* Write Single Register */
        if (len != 5)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        addr = get16(req + 1);
        if (!in_range(s, addr, 1))
            return exc(rsp, fc, EX_ILLEGAL_ADDRESS);
        s->regs[addr - s->base] = (uint16_t)get16(req + 3);
        memcpy(rsp, req, 5);
        return 5;

    case 0x0F: /* Write Multiple Coils */
        if (len < 6)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        addr = get16(req + 1);
        qty = get16(req + 3);
        bc = req[5];
        if (qty < 1 || qty > 1968 || bc != (qty + 7) / 8 || len != 6 + (int)bc)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        if (addr != 0 || qty != 1)
            return exc(rsp, fc, EX_ILLEGAL_ADDRESS);
        s->coil = req[6] & 1;
        memcpy(rsp, req, 5);
        return 5;

    case 0x10: /* Write Multiple Registers */
        if (len < 6)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        addr = get16(req + 1);
        qty = get16(req + 3);
        bc = req[5];
        if (qty < 1 || qty > 123 || bc != qty * 2 || len != 6 + (int)bc)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        if (!in_range(s, addr, qty))
            return exc(rsp, fc, EX_ILLEGAL_ADDRESS);
        for (unsigned i = 0; i < qty; i++)
            s->regs[addr - s->base + i] = (uint16_t)get16(req + 6 + i * 2);
        memcpy(rsp, req, 5);
        return 5;

    case 0x16: /* Mask Write Register */
        if (len != 7)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        addr = get16(req + 1);
        if (!in_range(s, addr, 1))
            return exc(rsp, fc, EX_ILLEGAL_ADDRESS);
        {
            unsigned and_m = get16(req + 3), or_m = get16(req + 5);
            uint16_t *r = &s->regs[addr - s->base];
            *r = (uint16_t)((*r & and_m) | (or_m & ~and_m));
        }
        memcpy(rsp, req, 7);
        return 7;

    case 0x17: /* Read/Write Multiple Registers */
        if (len < 10)
            return exc(rsp, fc, EX_ILLEGAL_VALUE);
        {
            unsigned raddr = get16(req + 1), rqty = get16(req + 3);
            unsigned waddr = get16(req + 5), wqty = get16(req + 7);
            bc = req[9];
            if (rqty < 1 || rqty > 125 || wqty < 1 || wqty > 121 ||
                bc != wqty * 2 || len != 10 + (int)bc)
                return exc(rsp, fc, EX_ILLEGAL_VALUE);
            if (!in_range(s, raddr, rqty) || !in_range(s, waddr, wqty))
                return exc(rsp, fc, EX_ILLEGAL_ADDRESS);
            for (unsigned i = 0; i < wqty; i++)
                s->regs[waddr - s->base + i] = (uint16_t)get16(req + 10 + i * 2);
            rsp[1] = (uint8_t)(rqty * 2);
            for (unsigned i = 0; i < rqty; i++)
                put16(rsp + 2 + i * 2, s->regs[raddr - s->base + i]);
            return 2 + (int)rqty * 2;
        }

    case 0x2B: /* Read Device Identification: empty object list, like pymodbus */
        if (len != 4 || req[1] != 0x0E)
            return exc(rsp, fc, EX_ILLEGAL_FUNCTION);
        rsp[1] = 0x0E;
        rsp[2] = req[2];    /* read device id code */
        rsp[3] = 0x83;      /* conformity level */
        rsp[4] = 0;         /* more follows */
        rsp[5] = 0;         /* next object id */
        rsp[6] = 0;         /* number of objects */
        return 7;

    default:
        return exc(rsp, fc, EX_ILLEGAL_FUNCTION);
    }
}

/* Route a request to unit `id`. Returns the response PDU length, 0 = no reply. */
static int dispatch(uint8_t id, const uint8_t *pdu, int len, uint8_t *rsp, int tcp)
{
    if (len < 1)
        return 0;
    if (id == 0 && !tcp) {              /* serial broadcast: execute, never reply */
        for (int i = 0; i < 256; i++)
            if (g_slaves[i])
                handle_pdu(g_slaves[i], pdu, len, rsp);
        return 0;
    }
    if (!g_slaves[id]) {
        if (!tcp) {
            logm(L_DEBUG, "Request for slave %d ignored (not configured)", id);
            return 0;
        }
        logm(L_WARNING, "Requested slave %d does not exist", id);
        return exc(rsp, pdu[0], EX_DEVICE_FAILURE);     /* same as pymodbus */
    }
    return handle_pdu(g_slaves[id], pdu, len, rsp);
}

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static long long now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static void sleep_sec(double s)
{
    struct timespec ts;
    ts.tv_sec = (time_t)s;
    ts.tv_nsec = (long)((s - (double)ts.tv_sec) * 1e9);
    while (!g_stop && nanosleep(&ts, &ts) < 0 && errno == EINTR)
        ;
}

static int write_all(int fd, const uint8_t *b, int n)
{
    while (n > 0) {
        ssize_t w = write(fd, b, (size_t)n);
        if (w < 0) {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN) {
                struct pollfd p = { fd, POLLOUT, 0 };
                poll(&p, 1, 1000);
                continue;
            }
            return -1;
        }
        b += w;
        n -= (int)w;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * Modbus TCP server
 * ------------------------------------------------------------------------- */

typedef struct {
    int fd;
    int len;
    uint8_t buf[MAX_TCP_ADU];
    char peer[64];
} client_t;

static int tcp_listen(const char *host, int port)
{
    struct addrinfo hints, *res, *ai;
    char portstr[16];
    int fd = -1, one = 1, rc;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    snprintf(portstr, sizeof portstr, "%d", port);
    rc = getaddrinfo(host, portstr, &hints, &res);
    if (rc)
        die("Cannot resolve %s: %s", host, gai_strerror(rc));
    for (ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0)
            continue;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        if (bind(fd, ai->ai_addr, ai->ai_addrlen) == 0 && listen(fd, 16) == 0)
            break;
        close(fd);
        fd = -1;
    }
    if (fd < 0)
        die("Cannot listen on %s:%d: %s", host, port, strerror(errno));
    freeaddrinfo(res);
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    return fd;
}

/* Process complete MBAP frames in the client buffer. Returns -1 to drop the client. */
static int tcp_process(client_t *c)
{
    uint8_t out[MAX_TCP_ADU];

    while (c->len >= 7) {
        unsigned pid = get16(c->buf + 2), mlen = get16(c->buf + 4);
        int total, rl;

        if (pid != 0 || mlen < 2 || mlen > 254) {
            logm(L_WARNING, "%s: invalid MBAP header (protocol=%u length=%u), closing",
                 c->peer, pid, mlen);
            return -1;
        }
        total = 6 + (int)mlen;
        if (c->len < total)
            break;
        log_frame("recv", c->buf, total);

        rl = dispatch(c->buf[6], c->buf + 7, (int)mlen - 1, out + 7, 1);
        if (rl > 0) {
            memcpy(out, c->buf, 4);         /* transaction + protocol id */
            put16(out + 4, (unsigned)rl + 1);
            out[6] = c->buf[6];
            log_frame("send", out, 7 + rl);
            if (write_all(c->fd, out, 7 + rl) < 0)
                return -1;
        }
        c->len -= total;
        memmove(c->buf, c->buf + total, (size_t)c->len);
    }
    return 0;
}

static void run_tcp(void)
{
    static client_t clients[MAX_CLIENTS];
    struct pollfd pfd[1 + MAX_CLIENTS];
    int idx[1 + MAX_CLIENTS];
    int lfd = tcp_listen(g_opt.host, g_opt.port);

    for (int i = 0; i < MAX_CLIENTS; i++)
        clients[i].fd = -1;
    logm(L_INFO, "Starting Modbus TCP server on %s:%d  (Ctrl+C to stop)", g_opt.host, g_opt.port);

    while (!g_stop) {
        int n = 0, r;

        pfd[n].fd = lfd;
        pfd[n].events = POLLIN;
        idx[n++] = -1;
        for (int i = 0; i < MAX_CLIENTS; i++)
            if (clients[i].fd >= 0) {
                pfd[n].fd = clients[i].fd;
                pfd[n].events = POLLIN;
                idx[n++] = i;
            }

        r = poll(pfd, (nfds_t)n, -1);
        if (r < 0) {
            if (errno == EINTR)
                continue;
            die("poll: %s", strerror(errno));
        }

        if (pfd[0].revents & POLLIN) {
            struct sockaddr_storage ss;
            socklen_t sl = sizeof ss;
            int fd = accept(lfd, (struct sockaddr *)&ss, &sl), slot = -1;
            if (fd >= 0) {
                char host[48] = "?", serv[8] = "?";
                getnameinfo((struct sockaddr *)&ss, sl, host, sizeof host, serv, sizeof serv,
                            NI_NUMERICHOST | NI_NUMERICSERV);
                for (int i = 0; i < MAX_CLIENTS; i++)
                    if (clients[i].fd < 0) {
                        slot = i;
                        break;
                    }
                if (slot < 0) {
                    logm(L_WARNING, "Too many clients, rejecting %s:%s", host, serv);
                    close(fd);
                } else {
                    fcntl(fd, F_SETFD, FD_CLOEXEC);
                    clients[slot].fd = fd;
                    clients[slot].len = 0;
                    snprintf(clients[slot].peer, sizeof clients[slot].peer, "%s:%s", host, serv);
                    logm(L_INFO, "Client connected: %s", clients[slot].peer);
                }
            }
        }

        for (int k = 1; k < n; k++) {
            client_t *c = &clients[idx[k]];
            ssize_t got;
            if (!pfd[k].revents)
                continue;
            got = read(c->fd, c->buf + c->len, sizeof c->buf - (size_t)c->len);
            if (got < 0 && (errno == EINTR || errno == EAGAIN))
                continue;
            if (got > 0) {
                c->len += (int)got;
                if (tcp_process(c) == 0)
                    continue;
            }
            logm(L_INFO, "Client disconnected: %s", c->peer);
            close(c->fd);
            c->fd = -1;
        }
    }

    for (int i = 0; i < MAX_CLIENTS; i++)
        if (clients[i].fd >= 0)
            close(clients[i].fd);
    close(lfd);
}

/* ---------------------------------------------------------------------------
 * Modbus RTU / ASCII serial server
 * ------------------------------------------------------------------------- */

static const struct { int baud; speed_t sp; } BAUDS[] = {
    { 300, B300 }, { 600, B600 }, { 1200, B1200 }, { 2400, B2400 }, { 4800, B4800 },
    { 9600, B9600 }, { 19200, B19200 }, { 38400, B38400 }, { 57600, B57600 },
    { 115200, B115200 }, { 230400, B230400 },
#ifdef B460800
    { 460800, B460800 },
#endif
#ifdef B921600
    { 921600, B921600 },
#endif
};

static speed_t baud_constant(int baud)
{
    for (size_t i = 0; i < sizeof BAUDS / sizeof BAUDS[0]; i++)
        if (BAUDS[i].baud == baud)
            return BAUDS[i].sp;
    return 0;
}

static int serial_open(void)
{
    struct termios tio;
    static const tcflag_t sizes[] = { CS5, CS6, CS7, CS8 };
    speed_t sp = baud_constant(g_opt.baudrate);
    int fd = open(g_opt.serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);

    if (fd < 0)
        return -1;
    if (tcgetattr(fd, &tio) < 0) {
        close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    tio.c_cflag &= ~(tcflag_t)(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
    tio.c_cflag |= CLOCAL | CREAD | sizes[g_opt.bytesize - 5];
    if (g_opt.parity != 'N')
        tio.c_cflag |= PARENB | (g_opt.parity == 'O' ? PARODD : 0);
    if (g_opt.stopbits == 2)
        tio.c_cflag |= CSTOPB;
    tio.c_iflag &= ~(tcflag_t)(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

typedef struct {
    int fd;
    int echo_skip;                  /* bytes of our own transmission still to drop */
    uint8_t last_tx[MAX_RTU_ADU];   /* last reply (binary ADU without checksum) */
    int last_tx_len;
    long long last_tx_until;        /* last_tx counts as echo until this time */
    double char_us;                 /* time of one character on the wire */
} serial_t;

static double char_time_us(void)
{
    int bits = 1 + g_opt.bytesize + (g_opt.parity != 'N') + g_opt.stopbits;
    return bits * 1e6 / g_opt.baudrate;
}

/* An adapter that echoes TX makes our reply look like a request (FC05/06
 * replies equal the request). Drop an exact copy of the last reply that
 * arrives right after it was sent. */
static int is_own_echo(serial_t *st, const uint8_t *adu, int n)
{
    int echo = st->last_tx_len == n && now_us() < st->last_tx_until &&
               !memcmp(st->last_tx, adu, (size_t)n);
    if (echo) {
        st->last_tx_len = 0;
        logm(L_DEBUG, "Ignoring echo of our own reply");
    }
    return echo;
}

static void serial_send(serial_t *st, const uint8_t *adu, int n, const uint8_t *wire, int wn)
{
    log_frame("send", adu, n);
    memcpy(st->last_tx, adu, (size_t)n);
    st->last_tx_len = n;
    st->last_tx_until = now_us() + (long long)(wn * st->char_us) + 100000;
    if (write_all(st->fd, wire, wn) < 0) {
        logm(L_WARNING, "Serial write failed: %s", strerror(errno));
        return;
    }
    if (g_opt.local_echo)
        st->echo_skip += wn;
}

static uint16_t crc16(const uint8_t *p, int n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static int crc_ok(const uint8_t *b, int n)
{
    uint16_t c = crc16(b, n - 2);
    return b[n - 2] == (c & 0xFF) && b[n - 1] == (c >> 8);
}

static int rtu_fc_known(uint8_t fc)
{
    return (fc & 0x80) || (fc >= 0x01 && fc <= 0x06) || fc == 0x0F || fc == 0x10 ||
           fc == 0x16 || fc == 0x17;
}

/* Look for a CRC-valid RTU frame at b[0]. Candidate lengths come from the
 * function code: the request length first, then the response length (other
 * slaves' replies are on the bus too). Returns the frame length, 0 if more
 * data is needed, -1 if b[0] cannot start a frame. */
static int rtu_scan(const uint8_t *b, int n, int *is_req)
{
    int lens[2], k = 0;
    uint8_t fc;

    if (n < 3)
        return 0;
    fc = b[1];
    if (fc & 0x80) {
        lens[k++] = 5;
    } else {
        switch (fc) {
        case 0x01: case 0x02: case 0x03: case 0x04:
            lens[k++] = 8;
            lens[k++] = 5 + b[2];
            break;
        case 0x05: case 0x06:
            lens[k++] = 8;
            break;
        case 0x0F: case 0x10:
            if (n < 7)
                return 0;
            lens[k++] = 9 + b[6];
            lens[k++] = 8;
            break;
        case 0x16:
            lens[k++] = 10;
            break;
        case 0x17:
            if (n < 11)
                return 0;
            lens[k++] = 13 + b[10];
            lens[k++] = 5 + b[2];
            break;
        default:
            return 0;       /* unknown length: decided when the line goes idle */
        }
    }
    for (int i = 0; i < k; i++) {
        if (lens[i] > MAX_RTU_ADU)
            continue;
        if (lens[i] > n)
            return 0;
        if (crc_ok(b, lens[i])) {
            *is_req = i == 0 && !(fc & 0x80);
            return lens[i];
        }
    }
    return -1;
}

static void rtu_request(serial_t *st, const uint8_t *b, int n)
{
    uint8_t out[MAX_RTU_ADU];
    int rl;
    uint16_t crc;

    if (is_own_echo(st, b, n - 2))
        return;
    log_frame("recv", b, n);
    rl = dispatch(b[0], b + 1, n - 3, out + 1, 0);
    if (rl <= 0)
        return;
    out[0] = b[0];
    crc = crc16(out, rl + 1);
    out[rl + 1] = (uint8_t)crc;
    out[rl + 2] = (uint8_t)(crc >> 8);
    serial_send(st, out, rl + 1, out, rl + 3);
}

static const char HEX[] = "0123456789ABCDEF";

static int hexval(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

static void ascii_request(serial_t *st, const char *hex, int n)
{
    uint8_t b[MAX_RTU_ADU], out[MAX_RTU_ADU];
    char wire[2 * MAX_RTU_ADU + 8];
    int nb = n / 2, rl, w = 0;
    uint8_t sum = 0;

    if (n % 2 || nb < 3 || nb > MAX_RTU_ADU)
        return;
    for (int i = 0; i < nb; i++) {
        int hi = hexval(hex[2 * i]), lo = hexval(hex[2 * i + 1]);
        if (hi < 0 || lo < 0)
            return;
        b[i] = (uint8_t)(hi << 4 | lo);
        sum = (uint8_t)(sum + b[i]);
    }
    if (sum != 0) {
        logm(L_DEBUG, "ASCII frame with bad LRC dropped");
        return;
    }
    if (is_own_echo(st, b, nb - 1))
        return;
    log_frame("recv", b, nb - 1);
    rl = dispatch(b[0], b + 1, nb - 2, out + 1, 0);
    if (rl <= 0)
        return;
    out[0] = b[0];
    sum = 0;
    for (int i = 0; i <= rl; i++)
        sum = (uint8_t)(sum + out[i]);
    out[rl + 1] = (uint8_t)-sum;
    wire[w++] = ':';
    for (int i = 0; i < rl + 2; i++) {
        wire[w++] = HEX[out[i] >> 4];
        wire[w++] = HEX[out[i] & 15];
    }
    wire[w++] = '\r';
    wire[w++] = '\n';
    serial_send(st, out, rl + 1, (const uint8_t *)wire, w);
}

/* Serve one open port until error/hangup (-1) or stop (0). */
static int serve_serial(serial_t *st)
{
    uint8_t buf[2 * MAX_RTU_ADU];
    char line[2 * MAX_RTU_ADU + 8];
    int n = 0, in_frame = 0, idle_ms;

    if (g_opt.ascii) {
        /* --timeout: a partial ASCII frame is dropped after this much silence */
        idle_ms = (int)(g_opt.timeout * 1000);
    } else {
        /* t3.5 (1.75 ms above 19200 baud), but at least 50 ms so USB-serial
         * latency inside a frame is not taken for a frame gap. Complete frames
         * are recognised by length+CRC without waiting for the gap. */
        double t35 = g_opt.baudrate > 19200 ? 1750 : 3.5 * st->char_us;
        idle_ms = (int)ceil(t35 / 1000);
        if (idle_ms < 50)
            idle_ms = 50;
    }
    if (idle_ms < 1)
        idle_ms = 1;

    while (!g_stop) {
        struct pollfd p = { st->fd, POLLIN, 0 };
        uint8_t tmp[256];
        ssize_t got;
        int busy = n > 0 || in_frame || st->echo_skip > 0, off = 0;
        int r = poll(&p, 1, busy ? idle_ms : -1);

        if (r < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (r == 0) {           /* line idle */
            st->echo_skip = 0;
            if (!g_opt.ascii && n >= 4 && crc_ok(buf, n)) {
                rtu_request(st, buf, n);
            } else if (n > 0 || in_frame) {
                logm(L_DEBUG, "Discarding incomplete frame (%d bytes)", g_opt.ascii ? 0 : n);
            }
            n = 0;
            in_frame = 0;
            continue;
        }
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
            return -1;
        got = read(st->fd, tmp, sizeof tmp);
        if (got < 0) {
            if (errno == EINTR || errno == EAGAIN)
                continue;
            return -1;
        }
        if (got == 0)
            return -1;
        if (st->echo_skip > 0) {
            off = st->echo_skip < got ? st->echo_skip : (int)got;
            st->echo_skip -= off;
        }

        if (g_opt.ascii) {
            for (; off < got; off++) {
                char c = (char)tmp[off];
                if (c == ':') {
                    in_frame = 1;
                    n = 0;
                } else if (!in_frame) {
                    continue;
                } else if (c == '\n') {
                    in_frame = 0;
                    if (n > 0 && line[n - 1] == '\r')
                        ascii_request(st, line, n - 1);
                } else if (n < (int)sizeof line) {
                    line[n++] = c;
                } else {
                    in_frame = 0;
                }
            }
            continue;
        }

        for (; off < got; off++) {
            if (n == (int)sizeof buf)
                memmove(buf, buf + 1, (size_t)--n);
            buf[n++] = tmp[off];
        }
        for (;;) {
            int is_req = 0, len = rtu_scan(buf, n, &is_req);
            if (len == 0 && n >= 2 && !rtu_fc_known(buf[1])) {
                /* unknown function code at the head (maybe noise): skip to a
                 * complete valid frame further on instead of waiting for idle */
                int off = 1;
                while (off < n && rtu_scan(buf + off, n - off, &is_req) <= 0)
                    off++;
                if (off < n) {
                    n -= off;
                    memmove(buf, buf + off, (size_t)n);
                    continue;
                }
            }
            if (len == 0)
                break;
            if (len < 0) {          /* resync: slide one byte */
                memmove(buf, buf + 1, (size_t)--n);
                continue;
            }
            if (is_req)
                rtu_request(st, buf, len);
            n -= len;
            memmove(buf, buf + len, (size_t)n);
        }
    }
    return 0;
}

static void run_serial(void)
{
    serial_t st;
    int failed = 0;

    memset(&st, 0, sizeof st);
    st.char_us = char_time_us();
    if (!baud_constant(g_opt.baudrate))
        die("Unsupported baud rate %d", g_opt.baudrate);
    logm(L_INFO, "Starting Modbus %s server on %s (baud=%d, %d%c%d, timeout=%.1fs)  (Ctrl+C to stop)",
         g_opt.ascii ? "ASCII" : "RTU", g_opt.serial_port, g_opt.baudrate, g_opt.bytesize,
         g_opt.parity, g_opt.stopbits, g_opt.timeout);

    while (!g_stop) {
        st.fd = serial_open();
        if (st.fd < 0) {
            logm(failed ? L_DEBUG : L_ERROR, "Cannot open %s: %s (retrying every %.1fs)",
                 g_opt.serial_port, strerror(errno), g_opt.reconnect_delay);
            failed = 1;
            sleep_sec(g_opt.reconnect_delay);
            continue;
        }
        failed = 0;
        logm(L_INFO, "Serial port %s opened", g_opt.serial_port);
        st.echo_skip = 0;
        st.last_tx_len = 0;
        if (serve_serial(&st) < 0 && !g_stop)
            logm(L_WARNING, "Serial port %s error: %s; reconnecting in %.1fs",
                 g_opt.serial_port, errno ? strerror(errno) : "hangup", g_opt.reconnect_delay);
        close(st.fd);
        if (!g_stop)
            sleep_sec(g_opt.reconnect_delay);
    }
}

/* ---------------------------------------------------------------------------
 * CLI
 * ------------------------------------------------------------------------- */

static const char *g_prog = "modbus-sim";

static void usage_line(FILE *f)
{
    fprintf(f,
        "usage: %s [-h] [--mode {tcp,rtu}] [--host HOST] [--port PORT]\n"
        "       [--serial-port SERIAL_PORT] [--baudrate BAUDRATE] [--bytesize {5,6,7,8}]\n"
        "       [--parity {N,E,O}] [--stopbits {1,2}] [--timeout TIMEOUT]\n"
        "       [--rtu-framer {rtu,ascii}] [--handle-local-echo]\n"
        "       [--reconnect-delay RECONNECT_DELAY] [--allow-multiple-devices] [-v]\n"
        "       [--debug-requests] [--log-level {DEBUG,INFO,WARNING,ERROR}]\n"
        "       FILE.yaml\n", g_prog);
}

static void usage(void)
{
    usage_line(stdout);
    printf(
        "\nModbus Slave simulator (C / OpenWrt) " VERSION " - loads YAML, serves TCP or RTU\n"
        "\npositional arguments:\n"
        "  FILE.yaml             Slave configuration file (YAML, supports 0x hex addresses)\n"
        "\noptions:\n"
        "  -h, --help            show this help message and exit\n"
        "  --mode {tcp,rtu}      Transport mode (default: tcp)\n"
        "  -v, --verbose         Show register dump on startup\n"
        "  --debug-requests      Log every received/sent Modbus frame (hex)\n"
        "  --log-level {DEBUG,INFO,WARNING,ERROR}\n"
        "                        Log level (default: INFO)\n"
        "  --version             Show version and exit\n"
        "\nTCP options:\n"
        "  --host HOST           Bind address (default: 0.0.0.0)\n"
        "  --port PORT           TCP port (default: 502)\n"
        "\nRTU options:\n"
        "  --serial-port SERIAL_PORT\n"
        "                        Serial port (default: /dev/ttyS0)\n"
        "  --baudrate BAUDRATE   Baud rate (default: 9600)\n"
        "  --bytesize {5,6,7,8}  Data bits (default: 8)\n"
        "  --parity {N,E,O}      Parity: N=None / E=Even / O=Odd (default: N)\n"
        "  --stopbits {1,2}      Stop bits (default: 1)\n"
        "  --timeout TIMEOUT     ASCII: drop a partial frame after this many seconds\n"
        "                        of silence (default: 3.0)\n"
        "  --rtu-framer {rtu,ascii}\n"
        "                        Framing mode: rtu or ascii (default: rtu)\n"
        "  --handle-local-echo   Discard local echo (RS-485 half-duplex adapters)\n"
        "  --reconnect-delay RECONNECT_DELAY\n"
        "                        Reconnect delay in seconds after disconnect (default: 2.0)\n"
        "  --allow-multiple-devices\n"
        "                        Accepted for compatibility (other devices' traffic on\n"
        "                        the RS-485 bus is always tolerated)\n");
}

static void arg_error(const char *fmt, ...)
{
    va_list ap;
    usage_line(stderr);
    fprintf(stderr, "%s: error: ", g_prog);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    exit(2);
}

static int arg_int(const char *name, const char *s)
{
    char *e;
    long v;
    errno = 0;
    v = strtol(s, &e, 10);
    if (!*s || *e || errno || v < 0 || v > 10000000)
        arg_error("argument %s: invalid int value: '%s'", name, s);
    return (int)v;
}

static double arg_float(const char *name, const char *s)
{
    char *e;
    double v;
    errno = 0;
    v = strtod(s, &e);
    if (!*s || *e || errno || !(v >= 0))
        arg_error("argument %s: invalid float value: '%s'", name, s);
    return v;
}

static void arg_choice(const char *name, const char *s, const char *const *choices)
{
    for (const char *const *c = choices; *c; c++)
        if (!strcmp(s, *c))
            return;
    arg_error("argument %s: invalid choice: '%s'", name, s);
}

enum {
    O_MODE = 256, O_HOST, O_PORT, O_SERIAL, O_BAUD, O_BYTESIZE, O_PARITY, O_STOPBITS,
    O_TIMEOUT, O_FRAMER, O_ECHO, O_RECONNECT, O_MULTI, O_DEBUGREQ, O_LOGLEVEL, O_VERSION,
};

static void parse_args(int argc, char **argv)
{
    static const struct option longopts[] = {
        { "help", no_argument, NULL, 'h' },
        { "verbose", no_argument, NULL, 'v' },
        { "mode", required_argument, NULL, O_MODE },
        { "host", required_argument, NULL, O_HOST },
        { "port", required_argument, NULL, O_PORT },
        { "serial-port", required_argument, NULL, O_SERIAL },
        { "baudrate", required_argument, NULL, O_BAUD },
        { "bytesize", required_argument, NULL, O_BYTESIZE },
        { "parity", required_argument, NULL, O_PARITY },
        { "stopbits", required_argument, NULL, O_STOPBITS },
        { "timeout", required_argument, NULL, O_TIMEOUT },
        { "rtu-framer", required_argument, NULL, O_FRAMER },
        { "handle-local-echo", no_argument, NULL, O_ECHO },
        { "reconnect-delay", required_argument, NULL, O_RECONNECT },
        { "allow-multiple-devices", no_argument, NULL, O_MULTI },
        { "debug-requests", no_argument, NULL, O_DEBUGREQ },
        { "log-level", required_argument, NULL, O_LOGLEVEL },
        { "version", no_argument, NULL, O_VERSION },
        { NULL, 0, NULL, 0 },
    };
    static const char *const modes[] = { "tcp", "rtu", NULL };
    static const char *const sizes[] = { "5", "6", "7", "8", NULL };
    static const char *const parities[] = { "N", "E", "O", NULL };
    static const char *const stops[] = { "1", "2", NULL };
    static const char *const framers[] = { "rtu", "ascii", NULL };
    int c;

    opterr = 0;
    while ((c = getopt_long(argc, argv, "hv", longopts, NULL)) != -1) {
        switch (c) {
        case 'h': usage(); exit(0);
        case 'v': g_opt.verbose = 1; break;
        case O_MODE: arg_choice("--mode", optarg, modes); g_opt.rtu = !strcmp(optarg, "rtu"); break;
        case O_HOST: g_opt.host = optarg; break;
        case O_PORT: g_opt.port = arg_int("--port", optarg); break;
        case O_SERIAL: g_opt.serial_port = optarg; break;
        case O_BAUD: g_opt.baudrate = arg_int("--baudrate", optarg); break;
        case O_BYTESIZE: arg_choice("--bytesize", optarg, sizes); g_opt.bytesize = atoi(optarg); break;
        case O_PARITY: arg_choice("--parity", optarg, parities); g_opt.parity = optarg[0]; break;
        case O_STOPBITS: arg_choice("--stopbits", optarg, stops); g_opt.stopbits = atoi(optarg); break;
        case O_TIMEOUT: g_opt.timeout = arg_float("--timeout", optarg); break;
        case O_FRAMER: arg_choice("--rtu-framer", optarg, framers); g_opt.ascii = !strcmp(optarg, "ascii"); break;
        case O_ECHO: g_opt.local_echo = 1; break;
        case O_RECONNECT: g_opt.reconnect_delay = arg_float("--reconnect-delay", optarg); break;
        case O_MULTI: g_opt.multi = 1; break;
        case O_DEBUGREQ: g_opt.debug_requests = 1; break;
        case O_LOGLEVEL: {
            int found = -1;
            for (int i = 0; i < 4; i++)
                if (!strcmp(optarg, LEVEL_NAMES[i]))
                    found = i;
            if (found < 0)
                arg_error("argument --log-level: invalid choice: '%s'", optarg);
            g_log_level = found;
            break;
        }
        case O_VERSION: printf("modbus-sim " VERSION "\n"); exit(0);
        case ':':
            arg_error("argument %s: expected one argument", argv[optind - 1]);
            break;
        default:
            arg_error("unrecognized arguments: %s", argv[optind - 1]);
        }
    }
    if (optind >= argc)
        arg_error("the following arguments are required: FILE.yaml");
    if (optind + 1 < argc)
        arg_error("unrecognized arguments: %s", argv[optind + 1]);
    g_opt.yaml = argv[optind];
    if (g_opt.port < 1 || g_opt.port > 65535)
        arg_error("argument --port: must be 1-65535");
}

static char *read_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    char *buf;
    long n;

    if (!f)
        arg_error("[Errno %d] %s: '%s'", errno, strerror(errno), path);
    if (fseek(f, 0, SEEK_END) < 0 || (n = ftell(f)) < 0 || fseek(f, 0, SEEK_SET) < 0)
        arg_error("cannot read %s", path);
    buf = xcalloc((size_t)n + 1, 1);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n)
        arg_error("cannot read %s", path);
    fclose(f);
    return buf;
}

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

int main(int argc, char **argv)
{
    struct sigaction sa;
    ynode *root;
    char *text;

    if (argc > 0 && argv[0][0]) {
        const char *slash = strrchr(argv[0], '/');
        g_prog = slash ? slash + 1 : argv[0];
    }
    parse_args(argc, argv);

    logm(L_INFO, "Loading YAML config from %s", g_opt.yaml);
    text = read_file(g_opt.yaml);
    root = yaml_parse(text, g_opt.yaml);
    free(text);
    if (!root->child)
        arg_error("%s: expected a YAML mapping at the top level", g_opt.yaml);

    g_scr = xcalloc(1, sizeof *g_scr);
    if (g_opt.verbose)
        show_registers(ychild(root, "slaves"));
    build_slaves(ychild(root, "slaves"));
    free(g_scr);
    g_scr = NULL;
    yfree(root);

    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;          /* no SA_RESTART: poll() returns EINTR */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    if (g_opt.rtu)
        run_serial();
    else
        run_tcp();
    logm(L_INFO, "Server stopped.");
    return 0;
}
