#!/bin/bash -eux

set -eux

pushd /tmp/input
mv guestinit.sh /home/ubuntu/guestinit.sh
mv bzImage /boot/vmlinuz-5.4.46
mv config-5.4.46 /boot/
mv m5 /sbin/m5
update-grub
tar xf kheaders.tar.bz2 -C /
popd
rm -rf /tmp/input

mkdir -p /lib/firmware/intel/ice/ddp
wget https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/tree/intel/ice/ddp/ice-1.3.28.0.pkg -P /lib/firmware/intel/ice/ddp/
mv /lib/firmware/intel/ice/ddp/ice-1.3.28.0.pkg /lib/firmware/intel/ice/ddp/ice.pkg
apt-get update
apt-get -y install \
    iperf \
    iputils-ping \
    netperf \
    netcat \
    ethtool \
    tcpdump \
    pciutils
