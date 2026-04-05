# Huawei Matebook E Go (8cx Gen 3)

This repository contains workarounds for the Huawei Matebook E Go (8cx Gen 3) to support Linux.

This project mainly refers to the following projects:
* [jhovold](https://github.com/jhovold/linux): x13s downstream linux maintainer, upstream support for sc8280xp platform and x13s device.
* [steev](https://github.com/steev/linux): x13s downstream linux maintainer.
* [linaro team](https://patchwork.kernel.org/project/linux-pm/cover/20240614-yoga-ec-driver-v7-0-9f0b9b40ae76@linaro.org/): initial arm64 EC support(C630), and UCSI driver based on arm64 EC.
* [Nikita Travkin](https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/platform/arm64/acer-aspire1-ec.c): anoter arm64 EC driver.
* [aymanbagabas](https://github.com/aymanbagabas/Huawei-WMI): Huawei laptop wmi driver.
* [NewWheat](https://gitlab.com/New-Wheat/linux-for-huawei-matebook-e-2019): yet anoter arm64 EC driver, and help from him.
* [chenxuecong2](https://github.com/chenxuecong2/linux-huawei-matebook-e-go) & [matalama80td3l](https://github.com/matalama80td3l/matebook-e-go-boot-works): initial device tree for the Huawei Matebook E Go.
* [White](https://github.com/whitelewi1-ctrl/matebook-e-go-linux): Dispcc and DPU fixes. hid over i2c fixes.
* [Detach0-0](https://github.com/awarson2233/EGoTouchRev-rebuild/blob/linux/Log/%E5%85%A8%E7%A8%8B.log): Huawei touch driver init sequence.

## Upstream Progress
~Initial dt support will land on 6.14.~(landed)

~Initial EC support will land on 6.15.~(landed)

~Initial UCSI support will land on 6.16.~(landed)

Dsi panel will land on 7.1

## Disclaimer
I am just an amateur, a Linux noob, not educated with formal CS courses. You should never believe what I say below, except this sentence. I am not responsible for your actions; you do this all at your own risk!

## Kernel
Use mainline with [recommended patches](./patch%20sets/recommended). (My AUR [here](https://aur.archlinux.org/pkgbase/linux-gaokun3))

Kernel parameter: `clk_ignore_unused pd_ignore_unused arm64.nopauth efi=noruntime`

## Prebuilt Bianry
See [release](https://github.com/right-0903/linux-ego/releases/tag/packages).

User and password are both `alarm` and both `root`.

## Feature Support

Refer to [this](https://github.com/jhovold/linux/wiki/X13s) as well. If you want to see some spoilers, check [this](./feature_spoliers/).

| Feature | Status | Note |
| ------- | ------ | ---- |
| Audio  | works | see [below](#audio) |
| Backlight | partial | see [below](#backlight) |
| Battery | works | For more attributes, check `/sys/class/power_supply/gaokun-ec-battery/*`  |
| Bluetooth | works | Adding board file for wifi will improve RF performance |
| Camera | x | |
| CPUfreq | works | |
| Display | works |  |
| Display sensors | partial | for rotation detect, light detect, etc. see [slpi](#SLPI) |
| EC | works | Battery and adapter status, wmi, ucsi, modern standby |
| EFI variables | works | |
| Fingerprint reader | x | FTE7001, gpio185 |
| GPU | works | Freedreno 690([FD690](./feature_spoliers/gpu.png)), `vulkan-freedreno` for vulkan support. |
| Hibernation | untested | |
| Keyboard & Touchpad | works | |
| Keyboard fn keys | partial | Assistant key(between `F6` and `F7`), WIFI key(`F9`), and Manager key(`F10`), they don't work. |
| Lid | works |  |
| NVMe SSD | works | |
| PCIe (x4) | works | |
| Power key | works | |
| Remoteproc | works | adsp, cdsp, sdsp, see [below](#DSP-offloading) |
| RTC | works | |
| Suspend | works | s2idle |
| Thermal sensors | works |  |
| Touchscreen over i2c | partial | see [below](#Touchscreen) |
| Touchscreen over spi | works | see [below](#over-spi) |
| TPM | x | |
| USB | works | |
| USB-PD | works | fast charging(45W+) supported with UCSI EC driver |
| USB-C DisplayPort Alt Mode | works | it may be broken nowadays (untested since ~6.14) |
| Video acceleration | partial | see [below](#Video-acceleration) |
| Virtualisation | works | see [below](#Virtualisation)|
| Volumn keys | works | |
| Watchdog | partial | One in sc8280xp works, another in EC not |
| Wi-Fi | works | see [below](#Wi-Fi) |
| WMI related features| partial | charging control, FN lock, smart charge, thermal sensors and so on |

---

## Altmode

> [!CAUTION]
> Recently, UCSI subsystem is buggy for this tablet. Chances are that failed to boot(auto reboot). Only plug into Type-C ports after booting up.

Addition: UCSI failed to register in 6.14, no sure about future ~(I don't own this device anymore, no fix from me)~

~Support 2-lane only for now(only half of clocks are enabled), Johan mentioned [this](https://github.com/jhovold/linux/wiki/X13s#USB) also, but we don't require insert orientation.~

In default, the bandwidth of per lane is limited to 5.4Gbps(HBR2, DisplayPort 1.2 or later). If you want to a higher limit, set this.

```
...
&mdss0_dp0_out {
...
        link-frequencies = /bits/ 64 <1620000000 2700000000 5400000000 8100000000>;
...
};
...
```

According to the [dcoument](https://learn.microsoft.com/en-us/windows/arm/dev-kit/#set-up-display---how-to-connect-monitors) of Windows Dev Kit 2023, USB-C ports support 8.1Gbps per lane(HBR3, DisplayPort 1.3 or later).

In my case, the monitor supports DisplayPort 1.1 with 2.7Gbps per lane, data rate is 4.32Gbps(8b10b coding), basically, it means I can only use 1920x1080@60Hz. However, if we tweak the timings(at your risks), we can support a decent resolution with a rich timings, 24bpp with 2048(+180H.Blanking)x1280(+60V.Blanking)@60hz $\approx$ 4.30Gbps. If the bandwidth limit is exceeded, your bpp will be degraded to 18 (or lower?) to make actual bandwidth under the limit.

If you encounter the following, reboot, it is rare, but still possible.
```
gaokun_ucsi.ucsi gaokun_ec.ucsi.0: error -ETIMEDOUT: PPM init failed
gaokun_ucsi.ucsi gaokun_ec.ucsi.0: con1: failed to register alt modes
```
If there is no issue now, let's plug in wait for seconds.

***Note: if you plug out when there is no other video output device(logical), replug will not work.***

![](./feature_spoliers/usbc_altmode.png)


## Audio
Recently, use the X13s' profile. \
`sed -i 's/LENOVO.*ThinkPad X13s.*/LENOVO.*ThinkPad X13s.*|HUAWEI.*MateBook E.*\"/' /usr/share/alsa/ucm2/Qualcomm/sc8280xp/sc8280xp.conf`

## Backlight
It is adjusted via DCS, but dcs broadcast mode is not implemented. So
in landscape mode, the upper half panel receives the DCS command
first, then the lower, adjusting in async mode may cause the issue.

This IC can be adjusted via I2C, if we can figure out which i2c bus,
we can dim it via I2C.

```bash
$ strings DisplayDxe.efi | grep -i 'bklic'
[BklIcRegsConfig] Failed to Write i2c IcReg:%x ,status:%d
```

## Battery
### Charging control

`echo '72' /sys/class/power_supply/gaokun-ec-battery/smart_charge_delay`

`echo '40' /sys/class/power_supply/gaokun-ec-battery/charge_control_start_threshold`

`echo '70' /sys/class/power_supply/gaokun-ec-battery/charge_control_end_threshold`

`echo '0/1' /sys/class/power_supply/gaokun-ec-battery/battery_adaptive_charge`

That means plugged in after 72 hours, charging level is limited to 70%.

> When the laptop is just connected to a power adapter, it starts charging until the battery level reaches 70% if the battery level is lower than 70%, or the battery level does not increase if the battery level is 70% or higher. This is normal.

> When the power adapter is always connected, the laptop starts charging if its battery level is lower than 40%, and stops charging when the battery level reaches 70%.

Further explanation, check Huawei support [page](https://consumer.huawei.com/en/support/content/en-us15783714/)

The values would not be saved after you shut down and plugged out, you can use `udev` to set the default values, like
```
ACTION=="add", SUBSYSTEM=="power_supply", KERNEL=="gaokun-ec-battery", ATTR{smart_charge_delay}="0", ATTRS{charge_control_start_threshold}="45", ATTRS{charge_control_end_threshold}="50", ATTRS{battery_adaptive_charge}="1"
```

## Camera
* front camera sensor: hi846(cci2, gpio12: cci_i2c_sda, gpio13: cci_i2c_scl, CAMCC_MCLK3_CLK: 24MHz, gpio17, reset: gpio15)
* rear camera sensor: s5k3l6(cci1, gpio10: cci_i2c_sda, gpio11: cci_i2c_scl, CAMCC_MCLK4_CLK: 24MHz, gpio6, reset: gpio7)

Front sensor can be probed, no camera video.

The rear sensor can't be detected.

## Display
Only core power sequence is taken into account (vddi, avdd/vsp, avee/vsn, bl), for the complete sequence, check [here](panel-hx83121a-dkms/dsi_pwr_seq.txt).

**Addition: Some patches are first proposed by [matalama80td3l](https://github.com/matalama80td3l) at [here](https://gitlab.freedesktop.org/drm/msm/-/issues/56). I verified and completed them.**

## DSP offloading
On this platform, we can offload tasks to ADSP or CDSP, [FastRPC reference](https://github.com/qualcomm/fastrpc).

To be filled.

## Touchscreen
### insights
This cascade IC is located at @0x48 @0x49 of i2c4. Its HID interfaces are @0x4f @0x50 respectively.
We can confirm @0x48 with 0x4f are for touchscreen. @0x49 with @0x50 may be for stylus.

In DSDT. gpio174 is used to determinate the transfer mode. Low will return SPI device as active,
high will return I2C hid device(@0x4f) as active, only if it is high, these i2c hid interfaces will
be exposed in i2c4.

In practical, we should set gpio174 before firmware reload(reload will auto trigger internally after TS reset(gpio99),
display reset(gpio38) may trigger TS reset internally.) After that, set gpio174 to any state is safe.

When the firmware was reloaded and the TS IRQ(gpio175) is triggered,
if we set gpio174 low previously, we can read raw tx-rx data via SPI;
if we set gpio174 high previously, we can read HID packets via I2C.

To check:
- [x] handle HID packets via I2C.
- [x] read chip id via I2C.
- [ ] read raw tx-rx data via I2C.
- [x] handle raw tx-rx data via SPI.
- [ ] read HID packets via SPI.

### hid over i2c
Intermittent touch when swiping. non-HLOS use this, such as BIOS configuration.

### over spi
#### firmware
> [!NOTE]
> There are several ts firmwares, your one may not be supported.

```
cfg_version display_version: timestamp
43 08: 20 23 01 10 16 12 15 (boe ?)
43 09: 20 23 01 10 16 16 29 (csot new ?)
43 07: 20 23 01 10 16 07 43 (csot)

41 08: 20 22 09 27 19 32 44 (boe ?)
41 09: 20 22 09 27 19 34 25 (csot new ?)
41 07: 20 22 09 27 19 30 56 (csot)
```

To check our ts firmware, do either

check `C:\ProgramData\Huawei\HuaweiTHP\hx_hal_log_*`

or

```bash
i2ctransfer -y 4 w5@0x48 0x00 0x00 0x15 0x02 0x00
i2ctransfer -y 4 w2@0x48 0x0c 0x00
i2ctransfer -y 4 w1@0x48 0x08 r2@0x48
```

Only `43/41 07` are tested, swiping would not work well on `43 07` when resuming from suspend.

#### algorithm
There are several algorithms, check `ts/*` branches.

In brief, `ts/caidj0`: improved multi-touch tracking.

## FN Lock
`echo 0/1 > /sys/devices/platform/soc@0/ac0000.geniqup/a9c000.i2c/i2c-15/15-0038/fn_lock` \
`0`: disable the fn key(hotkey first, such as press `F1` to decrease brightness) \
`1`: enable the fn key(function key first), you can only trigger hotkey with `Fn + F1-12` \
BTW, you can press `Fn` key(light on) to reverse it temporarily.

## Thermal Sensors from EC
I don't know these sensors for which devices, but you can read them \
`sensors`
```
gaokun_ec_hwmon-i2c-15-38
Adapter: Geni-I2C
temp1:        +47.4°C
...
temp20:       +25.9°C
```

## SLPI
SLPI means Sensor Low Power Island, [more details](https://lore.kernel.org/linux-devicetree/20230406173148.28309-1-me@dylanvanassche.be/).

You can reference the basic setup [here](https://gitlab.com/postmarketOS/pmaports/-/merge_requests/4050).

The sensor config and registry can be found in subdir of `C:\Windows\System32\drivers\DriverData\Qualcomm\fastRPC`, partial files required by `hexagonrpcd` are provided by `qcSensorsConfigQrd8280.cab` / `C:\Window\System32\DriverStore\FileRepository\qcsensorsconfigqrd8280*`. Partial files from Windows should be converted by `dos2unix`, since Linux uses `LF` instead of `CRLF`.

Recently, only accelerometer data can be fetched by
```bash
ssccli --sensor accelerometer
```
but `libssc` does not expose this meter to `iio-sensor-proxy` somehow.
```bash
# monitor-sensor --accel
    Waiting for iio-sensor-proxy to appear
+++ iio-sensor-proxy appeared
=== No accelerometer
```

## Video acceleration
Hardware decoding is supported via the Qualcomm Venus engine using the `v4l2m2m` API.

For `mpv`, a simple user config for low-resource devices is:
```ini
profile=fast
hwdec=v4l2m2m-copy
vo=gpu
swapchain-depth=8
sws-fast=yes
```

Put it in `~/.config/mpv/mpv.conf`, or use it temporarily from the command line:
- **mpv**: `mpv --hwdec=v4l2m2m-copy --vo=gpu --swapchain-depth=8 --sws-fast=yes <video>`

Hardware encoding is not working yet.

## Virtualisation

EL2 / KVM works on gaokun with an EL2 boot flow.

Minimal setup:
- use [`slbounce`](https://github.com/TravMurav/slbounce) to switch into EL2 at `ExitBootServices()`
- use [`qebspil`](https://github.com/stephan-gh/qebspil) before Linux boot, and apply the patches in [`patch sets/el2`](./patch%20sets/el2), if you want DSP / remoteproc-dependent functions (such as audio) to work in EL2
- place the required firmware files and an old enough working `tcblaunch.exe` in the correct EFI path
- boot Linux with the EL2 device tree (for example `sc8280xp-huawei-gaokun3-el2.dtb`)

Kernel side should have virtualization, remoteproc, and Qualcomm PAS/Q6V5 support enabled.
`slbounce` gets the system into EL2, while `qebspil` pre-starts the DSPs so Linux can hand them over correctly later.

## Wi-Fi
Use [the script](https://github.com/qca/qca-swiss-army-knife/blob/master/tools/scripts/ath11k/ath11k-bdencoder),

extract all files from the board file[(remote one)](https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/tree/ath11k/WCN6855/hw2.0/board-2.bin),

```
./ath11k-bdencoder -e board-2.bin
```

edit `board-2.json`, let us use x13s one calibration data,

```
...
"board": [
...
            {
                "names": [
                    "bus=pci,vendor=17cb,device=1103,subsystem-vendor=17cb,subsystem-device=0108,qmi-chip-id=2,qmi-board-id=255,variant=HW_GK3"
                ],
                "data": "bus=pci,vendor=17cb,device=1103,subsystem-vendor=17cb,subsystem-device=0108,qmi-chip-id=2,qmi-board-id=140,variant=LE_X13S.bin"
            },
...
]
...
```

regenerate it,

```
./ath11k-bdencoder -c board-2.json
```

### unit specificed calibration
It seems that HUAWEI calibrated for each unit. To extract the board file(calibration data), you must extract BIOS first.

The board file is stored at the 12th partition of BIOS (a lot of calibration data in this partiton actually, color calibration, wifi calibration, sensor calibration and a Windows activation key). The file starts at 0x1000, size is 0xea04, another file starts at 0xfb00, size is 0xea04, They are exactly the same on my machine.

Note that the unit calibration only works with 2.0 firmware which is provided by the downstream, [qrd driver](https://github.com/WOA-Project/Qualcomm-Reference-Drivers/blob/master/8280_QRD/200.0.68.0/qcwlan8280.cab) is compatible too. And the board file does not seem to be better than the x13s one we mentioned above, the x13s one has a better radio strength. Of course, you can choose to use the x13 one with 2.0 fw.

## BIOS
BIOS is inaccessible in HLOS(like Windows or Linux) for now. If you want to dump BIOS, check [this](./docs/BIOS/bios.md)

There are [patches](https://lore.kernel.org/linux-arm-msm/20260126-scm-storage-v2-v2-0-fa045c7e7699@radxa.com/) to support it on Linux. But nobody tested it on gaokun.
