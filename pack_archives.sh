#!/bin/sh
make -j$(nproc) ARCH=mips CROSS_COMPILE=mipsel-erx-linux-musl- vmlinux modules
make ARCH=mips CROSS_COMPILE=mipsel-erx-linux-musl- INSTALL_MOD_PATH=target/ modules_install
./build_image.sh
tar --owner=root --group=root -cjvf sfe-modules.tar.bz2 -C target lib/modules/3.10.14-UBNT/kernel/net/shortcut-fe \
								 lib/modules/3.10.14-UBNT/kernel/net/bridge/bridge.ko \
								 lib/modules/3.10.14-UBNT/kernel/net/netfilter/nf_conntrack.ko \
								 lib/modules/3.10.14-UBNT/kernel/net/netfilter/nf_conntrack_netlink.ko
rm -rf target
