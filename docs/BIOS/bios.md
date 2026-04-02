# BIOS

## BIOS Full Dump Guide

This guide explains how to perform a full dump of the BIOS (SPI NOR Flash) using a custom GRUB module.

### 1. Prerequisites & Setup
* **The `dd` Module:** Compile the `dd` GRUB module using the source code from [Gabriele Serra's Blog](https://gabrieleserra.ml/blog/2024-01-22-how-to-write-grub-modules.html) or use the pre-compiled `dd.mod`.
* **Storage Preparation:** * Place `dd.mod` into the **first partition** of your internal SSD (ensure the file system is **FAT16/FAT32**).
    * Prepare an empty USB drive with a **64MiB partition** (referred to below as the second partition).

### 2. Dumbing the BIOS via GRUB
Insert the USB drive and power on the device. Enter the GRUB menu and press **`c`** to access the command line.

> [!CAUTION]
> **Time Sensitivity:** Huawei devices have internal watchdog timers. If you do not boot into an HLOS (like Windows or Linux) within a certain timeframe, the system will automatically reboot. Perform the following steps quickly.

```bash
# 1. List devices to identify the storage layout.
# Usually: External USB = (hd0), Internal SSD = (hd1), BIOS SPI NOR Flash = (hd2)
grub> ls

# 2. Load the dd module from the internal SSD partition.
grub> insmod (hd1,gpt1)/dd.mod

# 3. Dump the BIOS to the USB partition.
# Note: Do NOT attempt to output to a loop device; GRUB does not support write operations on them.
grub> dd (hd2) (hd0,gpt2)
```

### 3. Post-Processing in Linux
After the dump is complete, boot into Linux and insert the USB drive. Replace `sdX` with your actual device identifier.

```bash
# 1. Extract the raw dump from the USB partition to an image file.
cat /dev/sdX2 > /tmp/bios.img

# 2. Set up a loop device with the correct sector size for analysis.
losetup --sector-size 4096 -fP /tmp/bios.img
```
