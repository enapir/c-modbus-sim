# modbus-sim

YAML ファイルで定義したレジスタ値を Modbus TCP / RTU / ASCII で応答する Modbus スレーブシミュレータです。
Python 版 `modbus_sim.py` を C に移植したもので、OpenWrt などの組込み Linux で動かすことを想定しています。

- 外部依存なし（Python / libyaml / libmodbus 不要）。YAML パーサ、Modbus スタック、シリアル処理をすべて内蔵
- C99 + POSIX libc（musl / glibc / uClibc）でビルド可能。1 ファイル（[modbus_sim.c](modbus-sim/src/modbus_sim.c)）
- Python 版と同じ YAML 形式・同じコマンドラインオプション
- OpenWrt パッケージ（procd init スクリプト + UCI 設定）付き

## ディレクトリ構成

```
modbus-sim/
├── Makefile                  OpenWrt パッケージ定義
├── src/
│   ├── Makefile              ネイティブ / クロスビルド用
│   └── modbus_sim.c          本体
└── files/
    ├── config.yaml           サンプル設定 (/etc/modbus-sim/config.yaml)
    ├── modbus-sim.config     UCI 設定 (/etc/config/modbus-sim)
    └── modbus-sim.init       procd init スクリプト (/etc/init.d/modbus-sim)
dist/                         ビルド済み成果物 (mipsel_24kc)
```

## ビルド

### ネイティブ / クロスコンパイル

```sh
cd modbus-sim/src
make                                                    # ホスト向け
make CC=mipsel-openwrt-linux-musl-gcc LDFLAGS=-static   # OpenWrt 向け静的バイナリ
```

静的リンクしたバイナリはルータにコピーするだけで動作します（パッケージのインストール不要）。

### OpenWrt SDK / buildroot

```sh
cp -r modbus-sim <sdk>/package/
cd <sdk>
make package/modbus-sim/compile V=s
# -> bin/packages/<arch>/base/modbus-sim_*.ipk
#    (OpenWrt 24.10 以降のスナップショットでは .apk)
```

### ビルド済みバイナリ

`dist/` に mipsel_24kc（MT7621 など）向けの成果物があります。

| ファイル | 内容 |
|---|---|
| `modbus-sim-mipsel_24kc-static` | 静的リンクの単体バイナリ |
| `modbus-sim_1.0.0-r1_mipsel_24kc.ipk` | OpenWrt パッケージ |

## 使い方

```sh
# Modbus TCP（デフォルト 0.0.0.0:502）
modbus-sim config.yaml

# ポート・バインドアドレスを指定
modbus-sim config.yaml --host 127.0.0.1 --port 5020

# Modbus RTU（シリアル）
modbus-sim config.yaml --mode rtu --serial-port /dev/ttyUSB0 --baudrate 9600

# Modbus ASCII、RS-485 アダプタのローカルエコーを除去
modbus-sim config.yaml --mode rtu --rtu-framer ascii --handle-local-echo

# 起動時にレジスタ一覧を表示し、送受信フレームを 16 進でログ出力
modbus-sim config.yaml -v --debug-requests
```

### オプション

| オプション | デフォルト | 説明 |
|---|---|---|
| `FILE.yaml` | （必須） | スレーブ設定ファイル |
| `--mode {tcp,rtu}` | `tcp` | 通信方式 |
| `-v`, `--verbose` | | 起動時にレジスタ一覧を表示 |
| `--debug-requests` | | 送受信した Modbus フレームを 16 進でログ出力 |
| `--log-level {DEBUG,INFO,WARNING,ERROR}` | `INFO` | ログレベル |
| `--version` | | バージョンを表示して終了 |
| **TCP** | | |
| `--host HOST` | `0.0.0.0` | バインドアドレス |
| `--port PORT` | `502` | TCP ポート |
| **RTU / ASCII** | | |
| `--serial-port PATH` | `/dev/ttyS0` | シリアルデバイス |
| `--baudrate N` | `9600` | ボーレート（300〜921600） |
| `--bytesize {5,6,7,8}` | `8` | データビット |
| `--parity {N,E,O}` | `N` | パリティ |
| `--stopbits {1,2}` | `1` | ストップビット |
| `--rtu-framer {rtu,ascii}` | `rtu` | フレーミング |
| `--timeout SEC` | `3.0` | ASCII: この秒数無通信が続いたら途中のフレームを破棄 |
| `--handle-local-echo` | | 自分の送信のエコーを破棄（RS-485 半二重アダプタ用） |
| `--reconnect-delay SEC` | `2.0` | シリアルポート切断時の再接続間隔 |
| `--allow-multiple-devices` | | Python 版との互換用（RS-485 上の他機器の通信は常に許容） |

ログは標準エラー出力に `HH:MM:SS [LEVEL] modbus-sim: メッセージ` の形式で出力されます。
`SIGINT` / `SIGTERM` で終了します。

## 設定ファイル（YAML）

```yaml
slaves:
  1:                          # スレーブ ID (0-255)
    holding_registers:
      0x0000: 100             # アドレス: 値
      0x0001: 200
    input_registers:
      0x0002: 1234
  2:
    holding_registers:
      # 名前付きセクションでまとめることもできる（フラットな記述と混在可）
      battery:
        0x1000: 500
        0x1001: 0xFFFF
      grid: {0x2000: 1, 0x2001: 2}
```

- アドレス・値は 10 進、`0x` 16 進、`0b` 2 進、`0` 始まりの 8 進、`_` 区切りを受け付けます（YAML 1.1 準拠）
- 値は下位 16 ビットに切り詰められます（負数は 2 の補数）
- アドレスの重複は後勝ち。同名セクションは後のものがセクションごと置き換えます
- スレーブが 1 つも定義されていない場合は、空のスレーブ ID 1 を作成します

対応する YAML はサブセットです。ブロックマッピング、コメント、クォート付き / なしのスカラ、1 行のフローマッピング（`{k: v, ...}`）に対応します。
アンカー / エイリアス / マージキー（`&` `*` `<<`）はエラーになり、それ以外の未対応構文（ブロックスカラーや無関係なキー配下のリスト）は無視されます。

## 動作仕様

### レジスタの扱い

実機 PCS の挙動に合わせ、`holding_registers` と `input_registers` は**同じアドレス空間**として扱います。
両者を合わせたデータを FC03 と FC04 の両方で同じように返し、同じアドレスが両方にある場合は `input_registers` の値を使います。

レジスタ領域は定義された最小アドレスから最大アドレスまでの連続領域で、途中の未定義アドレスは 0 として読めます。範囲外へのアクセスは Illegal Data Address（0x02）を返します。
書き込みはメモリ上の値を書き換えるだけで、YAML ファイルには保存されません。

### 対応ファンクションコード

| FC | 内容 | 備考 |
|---|---|---|
| 0x01 / 0x02 | Read Coils / Discrete Inputs | アドレス 0 の 1 ビットのみ |
| 0x03 / 0x04 | Read Holding / Input Registers | 同じデータを返す |
| 0x05 / 0x0F | Write Single / Multiple Coils | アドレス 0 の 1 ビットのみ |
| 0x06 / 0x10 | Write Single / Multiple Registers | |
| 0x16 | Mask Write Register | |
| 0x17 | Read/Write Multiple Registers | |
| 0x2B / 0x0E | Read Device Identification | 空のオブジェクトリストを返す |

上記以外は Illegal Function（0x01）を返します。

### 未定義スレーブ・ブロードキャスト

- TCP: 未定義のユニット ID には Slave Device Failure（0x04）を返します（pymodbus と同じ）
- RTU / ASCII: 未定義のスレーブ ID には応答しません。ID 0（ブロードキャスト）は全スレーブで実行し、応答は返しません

### TCP

- 最大 64 クライアントまで同時接続できます
- MBAP ヘッダが不正な場合（プロトコル ID ≠ 0 など）はその接続を切断します

### RTU / ASCII

- RTU はファンクションコードから決まるフレーム長と CRC でフレームを判定するため、t3.5 の無通信を待たずに応答します。USB シリアルの遅延を考慮し、フレーム間の無通信判定は最低 50 ms です
- RS-485 上の他のスレーブの応答フレームも認識して読み飛ばします
- 送信直後に自分の応答と同じフレームを受信した場合はエコーとみなして破棄します（`--handle-local-echo` を指定すると送信バイト数分を常に破棄）
- シリアルポートが開けない場合や切断された場合は `--reconnect-delay` 間隔で再接続を試みます

## OpenWrt での運用

### インストール

```sh
scp dist/modbus-sim_1.0.0-r1_mipsel_24kc.ipk root@<router>:/tmp/
ssh root@<router> opkg install /tmp/modbus-sim_1.0.0-r1_mipsel_24kc.ipk
```

インストールされるファイルは次のとおりです。

| パス | 内容 |
|---|---|
| `/usr/bin/modbus-sim` | 本体 |
| `/etc/init.d/modbus-sim` | procd init スクリプト |
| `/etc/config/modbus-sim` | UCI 設定（conffile） |
| `/etc/modbus-sim/config.yaml` | レジスタ定義（conffile） |

### 設定と起動

サービスは**デフォルトで無効**（`enabled '0'`）です。

```sh
# レジスタ定義を差し替え
scp my_config.yml root@<router>:/etc/modbus-sim/config.yaml

# 有効化して起動
uci set modbus-sim.main.enabled=1
uci commit modbus-sim
/etc/init.d/modbus-sim enable
/etc/init.d/modbus-sim start

# ログ確認
logread -e modbus-sim
```

RTU で使う場合の例:

```sh
uci set modbus-sim.main.mode=rtu
uci set modbus-sim.main.serial_port=/dev/ttyUSB0
uci set modbus-sim.main.baudrate=19200
uci set modbus-sim.main.parity=E
uci commit modbus-sim
/etc/init.d/modbus-sim restart
```

UCI オプションはコマンドラインオプションに対応しています。

| UCI オプション | デフォルト | 対応するコマンドラインオプション |
|---|---|---|
| `enabled` | `0` | （サービスの有効 / 無効） |
| `config` | `/etc/modbus-sim/config.yaml` | `FILE.yaml` |
| `mode` | `tcp` | `--mode` |
| `host` / `port` | `0.0.0.0` / `502` | `--host` / `--port` |
| `serial_port` | `/dev/ttyUSB0` | `--serial-port` |
| `baudrate` / `bytesize` / `parity` / `stopbits` | `9600` / `8` / `N` / `1` | 同名オプション |
| `framer` | `rtu` | `--rtu-framer` |
| `handle_local_echo` | `0` | `--handle-local-echo` |
| `debug_requests` | `0` | `--debug-requests` |
| `log_level` | `INFO` | `--log-level` |

UCI 設定を変更したあと `/etc/init.d/modbus-sim reload`（または `reload_config`）を実行すると反映されます。YAML ファイルの変更も reload で再読み込みされます。
プロセスが異常終了した場合は procd が自動で再起動します。
