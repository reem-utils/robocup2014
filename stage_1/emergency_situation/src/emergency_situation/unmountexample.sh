#!/bin/bash

dev=$(readlink -f /mnt/robocup)
echo $dev
grep -q ^$dev /proc/mounts && umount $dev
umount $dev
