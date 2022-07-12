#!/bin/bash -eux

set -eux

pushd /tmp/input
mv guestinit.sh /home/ubuntu/guestinit.sh
mv bzImage /boot/vmlinuz-5.4.46
mv config-5.4.46 /boot/
mv m5 /sbin/m5
mkdir -p /lib/firmware/intel/ice/ddp
mv ice.pkg /lib/firmware/intel/ice/ddp/ice.pkg
update-grub
tar xf kheaders.tar.bz2 -C /
popd
rm -rf /tmp/input

apt-get update
apt-get -y install \
    iperf \
    netperf \
    netcat \
    ethtool \
    tcpdump
