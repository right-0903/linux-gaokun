#!/bin/bash
set -euo pipefail
# =============================================================================
# Filename: img_pack.sh
# Purpose: To be filled
# Usage: sudo ./img_pack.sh
# =============================================================================

# libarchive-tools for bsdtar, qemu-user-static for emulation
# parted for partitioning, arch-install-scripts for arch-chroot/genfstab
apt update && apt install -y zstd curl libarchive-tools qemu-user-static parted arch-install-scripts dosfstools e2fsprogs

CHROOT_DIR='alarm-chroot'
IMAGE_SIZE="${IMAGE_SIZE:-6G}"
ROOTFS_PAD_MIB="${ROOTFS_PAD_MIB:-512}"
SHRINK_IMAGE="${SHRINK_IMAGE:-1}"
ZSTD_LEVEL="${ZSTD_LEVEL:-15}"
GRUB_CMDLINE_LINUX_BASE="${GRUB_CMDLINE_LINUX_BASE:-clk_ignore_unused pd_ignore_unused arm64.nopauth efi=noruntime}"
GRUB_CMDLINE_LINUX_DEFAULT="${GRUB_CMDLINE_LINUX_DEFAULT:-fbcon=rotate:1 loglevel=3}"
GRUB_CMDLINE_LINUX_EXTRA="${GRUB_CMDLINE_LINUX_EXTRA:-}"

cleanup() {
    local errexit_was_set=0
    [[ $- == *e* ]] && errexit_was_set=1
    set +e
    if mountpoint -q "${CHROOT_DIR}/boot/efi" 2>/dev/null; then
        umount "${CHROOT_DIR}/boot/efi"
    fi
    if mountpoint -q "${CHROOT_DIR}" 2>/dev/null; then
        umount -R "${CHROOT_DIR}"
    fi
    if [[ -n "${LOOP_DEV:-}" ]] && losetup "${LOOP_DEV}" >/dev/null 2>&1; then
        losetup -d "${LOOP_DEV}"
    fi
    (( errexit_was_set )) && set -e
}

shrink_rootfs_image() {
    local block_count block_size fs_mib padded_fs_mib root_end_mib disk_size_bytes

    e2fsck -fy "${LOOP_DEV}p2"
    resize2fs -M "${LOOP_DEV}p2"

    read -r block_count block_size < <(dumpe2fs -h "${LOOP_DEV}p2" 2>/dev/null | awk '
        /Block count:/ {count=$3}
        /Block size:/ {size=$3}
        END {print count, size}
    ')

    fs_mib=$(( (block_count * block_size + 1024 * 1024 - 1) / (1024 * 1024) ))
    padded_fs_mib=$(( fs_mib + ROOTFS_PAD_MIB ))
    resize2fs "${LOOP_DEV}p2" "${padded_fs_mib}M"

    root_end_mib=$(( 301 + padded_fs_mib ))
    # Parted still asks for confirmation when shrinking a partition on some CI runners.
    printf 'Yes\n' | parted ---pretend-input-tty "${LOOP_DEV}" unit MiB resizepart 2 "${root_end_mib}"

    cleanup

    disk_size_bytes=$(( (root_end_mib + 1) * 1024 * 1024 ))
    truncate -s "${disk_size_bytes}" archlinuxarm.img
}

trap cleanup EXIT

# handle binfmt_misc, https://access.redhat.com/solutions/1985633
if grep -q 'binfmt_misc' /proc/mounts; then
    echo "binfmt_misc mounted"
else
    mount binfmt_misc -t binfmt_misc /proc/sys/fs/binfmt_misc
fi

if [[ -f /proc/sys/fs/binfmt_misc/status ]] && grep -qx 'disabled' /proc/sys/fs/binfmt_misc/status; then
    echo 1 > /proc/sys/fs/binfmt_misc/status
fi

if [[ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]]; then
    echo "qemu-aarch64 binfmt already registered"
else
    echo ':qemu-aarch64:M::\x7fELF\x02\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\xb7\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/bin/qemu-aarch64-static:FP' > /proc/sys/fs/binfmt_misc/register
fi

write_grub_cfg() {
    local linux_cmdline
    linux_cmdline="$(build_linux_cmdline)"

    cat > "${CHROOT_DIR}/boot/grub/grub.cfg" <<EOF
set default=0
set timeout=5

menuentry 'Arch Linux (gaokun3 NVMe)' {
    insmod part_gpt
    insmod ext2
    insmod gzio
    search --no-floppy --fs-uuid --set=root ${ROOT_UUID}
    devicetree /boot/sc8280xp-huawei-gaokun3.dtb
    linux /boot/vmlinuz-linux-gaokun3 ${linux_cmdline}
    initrd /boot/initramfs-linux-gaokun3.img
}
EOF
}

build_linux_cmdline() {
    local -a args

    args=("root=UUID=${ROOT_UUID}" "rw")

    append_cmdline_words "${GRUB_CMDLINE_LINUX_BASE}" args

    append_cmdline_words "${GRUB_CMDLINE_LINUX_DEFAULT}" args
    append_cmdline_words "${GRUB_CMDLINE_LINUX_EXTRA}" args

    printf '%s\n' "${args[*]}"
}

append_cmdline_words() {
    local value="$1"
    local -n target_ref="$2"
    local -a words

    if [[ -z "${value}" ]]; then
        return
    fi

    read -r -a words <<< "${value}"
    target_ref+=("${words[@]}")
}


# Pick a free loop device instead of assuming a fixed one exists.
LOOP_DEV="$(losetup -f)"

# container setup
truncate -s "${IMAGE_SIZE}" archlinuxarm.img
losetup -P ${LOOP_DEV} archlinuxarm.img
parted ${LOOP_DEV} --script mklabel gpt
parted ${LOOP_DEV} --script mkpart EFI fat32 1MiB 301MiB
parted ${LOOP_DEV} --script set 1 boot on
parted ${LOOP_DEV} --script mkpart ALARM ext4 301MiB 100%
mkfs.fat -F32 ${LOOP_DEV}p1
mkfs.ext4 ${LOOP_DEV}p2
ROOT_UUID="$(blkid -s UUID -o value "${LOOP_DEV}p2")"

# mount, extract rootfs and generate a mount table
mkdir -p ${CHROOT_DIR}
mount ${LOOP_DEV}p2 ${CHROOT_DIR}
MIRROR_URL='http://fl.us.mirror.archlinuxarm.org'
curl -fsSL "$MIRROR_URL/os/ArchLinuxARM-aarch64-latest.tar.gz" -o alarm.tar.gz
bsdtar -xpf alarm.tar.gz -C ${CHROOT_DIR}
mkdir -p ${CHROOT_DIR}/boot/efi
mount ${LOOP_DEV}p1 ${CHROOT_DIR}/boot/efi

###### dirty insert ######

genfstab -U ${CHROOT_DIR} >> ${CHROOT_DIR}/etc/fstab

# github action runner has a swap, we don't need it.
sed -i '/^.*swap.*$/d' ${CHROOT_DIR}/etc/fstab

# many tutorials sugget this
cp /usr/bin/qemu-aarch64-static  ${CHROOT_DIR}/usr/bin/qemu-aarch64-static

# enable ParallelDownloads
sed -i 's/#ParallelDownloads = 5/ParallelDownloads = 4/' ${CHROOT_DIR}/etc/pacman.conf
sed -i 's/^DownloadUser = .*/DownloadUser = root/' ${CHROOT_DIR}/etc/pacman.conf
sed -i 's/^#DisableSandboxFilesystem/DisableSandboxFilesystem/' ${CHROOT_DIR}/etc/pacman.conf
sed -i 's/^#DisableSandboxSyscalls/DisableSandboxSyscalls/' ${CHROOT_DIR}/etc/pacman.conf
echo "Server = $MIRROR_URL"'/$arch/$repo' >> ${CHROOT_DIR}/etc/pacman.d/mirrorlist

# add my repo to install kernel and firmware
echo '[nuvole-arch]' >> ${CHROOT_DIR}/etc/pacman.conf
echo "Server = https://github.com/right-0903/my_arch_auto_pack/releases/download/packages" >> ${CHROOT_DIR}/etc/pacman.conf

# set console font
cat << EOF >> ${CHROOT_DIR}/etc/vconsole.conf
KEYMAP=us
FONT=solar24x32
EOF

# disable all kinds of sleep for now
cat << EOF >> ${CHROOT_DIR}/etc/systemd/sleep.conf
AllowSuspend=no
AllowHibernation=no
AllowSuspendThenHibernate=no
AllowHybridSleep=no
EOF

# initialize the pacman keyring and populate the Arch Linux ARM package signing keys
# https://archlinuxarm.org/platforms/armv8/generic
arch-chroot ${CHROOT_DIR} sh -c 'pacman-key --init && pacman-key --populate archlinuxarm'

# trust my key for my repo
curl https://raw.githubusercontent.com/right-0903/my_arch_auto_pack/refs/heads/main/keys/CA909D46CD1890BE.asc -o ${CHROOT_DIR}/root/CA909D46CD1890BE.asc
arch-chroot ${CHROOT_DIR} sh -c 'pacman-key --add /root/CA909D46CD1890BE.asc && pacman-key --lsign-key CA909D46CD1890BE'

# Upgrade userland first. Defer the custom kernel until its firmware is in place.
arch-chroot ${CHROOT_DIR} sh -c 'pacman -Syu efibootmgr grub wireless-regdb iwd btrfs-progs --noconfirm'

# The custom firmware package overlaps with upstream qcom/atheros firmware files.
arch-chroot ${CHROOT_DIR} sh -c 'pacman -Rdd --noconfirm linux-firmware-atheros || true'
arch-chroot ${CHROOT_DIR} sh -c 'pacman -Rdd --noconfirm linux-firmware-qcom || true'
arch-chroot ${CHROOT_DIR} sh -c 'rm -rf /usr/lib/firmware/ath11k/WCN6855 /usr/lib/firmware/qca /usr/lib/firmware/qcom/a660_gmu.bin /usr/lib/firmware/qcom/a660_sqe.fw'
arch-chroot ${CHROOT_DIR} sh -c "pacman -S linux-firmware-gaokun3 --noconfirm --overwrite '/usr/lib/firmware/ath11k/WCN6855/*,/usr/lib/firmware/qca/*,/usr/lib/firmware/qcom/a660_*'"

# Install the custom kernel only after the device-specific firmware is present.
arch-chroot ${CHROOT_DIR} sh -c 'pacman -S linux-gaokun3 linux-gaokun3-headers --noconfirm'

# Remove the stock kernel after the replacement is installed successfully.
arch-chroot ${CHROOT_DIR} sh -c 'pacman -Rdd --noconfirm linux-aarch64 || true'

# make a copy for this repo
shopt -s nullglob
pkg_cache=(${CHROOT_DIR}/var/cache/pacman/pkg/*.pkg.tar.*)
if (( ${#pkg_cache[@]} )); then
    mv "${pkg_cache[@]}" .
fi
shopt -u nullglob
rm -f ${CHROOT_DIR}/var/cache/pacman/pkg/*

# use early KMS for debugging, this would give us log in the initramfs stage.
if [[ -f ${CHROOT_DIR}/etc/mkinitcpio-gaokun3.conf ]]; then
    sed -i 's/^\(MODULES=(\)/\1\nsimpledrm\nphy-qcom-snps-femto-v2/' ${CHROOT_DIR}/etc/mkinitcpio-gaokun3.conf
elif [[ -f ${CHROOT_DIR}/etc/mkinitcpio.conf ]]; then
    sed -i 's/^\(MODULES=(\)/\1\nsimpledrm\nphy-qcom-snps-femto-v2/' ${CHROOT_DIR}/etc/mkinitcpio.conf
fi
arch-chroot ${CHROOT_DIR} sh -c 'mkinitcpio -P'

# install grub
arch-chroot ${CHROOT_DIR} sh -c 'grub-install --target=arm64-efi --efi-directory=/boot/efi --bootloader-id=arch --no-nvram --recheck'

# fix efi loading
arch-chroot ${CHROOT_DIR} sh -c 'mkdir -p /boot/efi/EFI/Boot && cp /boot/efi/EFI/arch/grubaa64.efi /boot/efi/EFI/Boot/BOOTAA64.EFI'

# Write the known-good configuration directly instead of relying on generic
# templates, which may miss the board DTB or load unavailable arm64 modules.
write_grub_cfg

# do clean
rm -f ${CHROOT_DIR}/usr/bin/qemu-aarch64-static ${CHROOT_DIR}/root/*

# Shrink the finished image before compressing it.
if [[ "${SHRINK_IMAGE}" == "1" ]]; then
    umount -R "${CHROOT_DIR}"
    shrink_rootfs_image
else
    cleanup
fi

# Compress with zstd to make CI finish faster. Use a single .zst artifact.
zstd -T0 -"${ZSTD_LEVEL}" --rm archlinuxarm.img
