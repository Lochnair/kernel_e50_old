#!/bin/sh

make -j$(nproc) ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- vmlinux modules
make ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- INSTALL_MOD_PATH=target/ modules_install
./build_image.sh
tar --owner=root --group=root -cjvf sfe-modules.tar.bz2 -C target lib/modules/4.14.54-UBNT/kernel/net/shortcut-fe \
								 lib/modules/4.14.54-UBNT/kernel/net/bridge/bridge.ko \
								 lib/modules/4.14.54-UBNT/kernel/net/netfilter/nf_conntrack.ko \
								 lib/modules/4.14.54-UBNT/kernel/net/netfilter/nf_conntrack_netlink.ko
rm -rf target
