#!/bin/bash
# One-shot boot script for iOS 16 on Xiaomi Redmi Note 7 (lavender)
set -e

FASTBOOT="/data/data/com.termux/files/usr/bin/fastboot"
IMG="/data/local/tmp/shim-boot-springboard.img"
SERIAL="2d71cc89"

echo "[*] Checking for lavender ($SERIAL) in fastboot mode..."
if su -c "$FASTBOOT devices" 2>/dev/null | grep -q "$SERIAL"; then
    echo "[+] Found device: $SERIAL"
else
    echo "[!] Device $SERIAL not detected yet. Waiting for fastboot connection..."
    while true; do
        if su -c "$FASTBOOT devices" 2>/dev/null | grep -q "$SERIAL"; then
            echo "[+] Detected device: $SERIAL"
            break
        fi
        sleep 1
    done
fi

echo "[*] Booting $IMG..."
su -c "$FASTBOOT -s $SERIAL boot $IMG"
echo "[+] Boot command sent! iOS 16 is booting on lavender."
