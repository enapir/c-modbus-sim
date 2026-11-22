#
# OpenWrt package: Modbus slave simulator (C port of modbus_sim.py)
#
# SDK / buildroot:
#   cp -r openwrt/modbus-sim <sdk>/package/
#   make package/modbus-sim/compile V=s
#   -> bin/packages/<arch>/base/modbus-sim_*.ipk (or .apk on OpenWrt 24.10+ snapshots)
#

include $(TOPDIR)/rules.mk

PKG_NAME:=modbus-sim
PKG_VERSION:=1.0.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Package/modbus-sim
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Modbus TCP/RTU slave simulator (YAML configured)
endef

define Package/modbus-sim/description
  Modbus slave simulator that loads register values from a YAML file and
  serves them over Modbus TCP or RTU/ASCII (serial). No Python required.
endef

define Package/modbus-sim/conffiles
/etc/config/modbus-sim
/etc/modbus-sim/config.yaml
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS)" \
		CPPFLAGS="$(TARGET_CPPFLAGS)" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/modbus-sim/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/modbus_sim $(1)/usr/bin/modbus-sim
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/modbus-sim.init $(1)/etc/init.d/modbus-sim
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/modbus-sim.config $(1)/etc/config/modbus-sim
	$(INSTALL_DIR) $(1)/etc/modbus-sim
	$(INSTALL_CONF) ./files/config.yaml $(1)/etc/modbus-sim/config.yaml
endef

$(eval $(call BuildPackage,modbus-sim))
