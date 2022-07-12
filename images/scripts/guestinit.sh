#!/bin/sh
cd /tmp
tar xf /dev/sdb
cd guest
mount -t debugfs nodev /sys/kernel/debug
./run.sh
