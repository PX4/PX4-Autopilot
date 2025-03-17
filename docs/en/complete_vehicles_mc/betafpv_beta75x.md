# BetaFPV Beta75X 2S Brushless Whoop

<Badge type="info" text="Discontinued" />

:::warning
This frame has been [discontinued](../flight_controller/autopilot_experimental.md) and is no longer commercially available.
:::

The [BetaFPV Beta75X](https://betafpv.com/products/beta75x-2s-whoop-quadcopter) is a very small quadrotor that can be flown indoors or outdoors, FPV or line-of-sight.

![BetaFPV Beta75X](../../assets/hardware/betafpv_beta75x.jpg)

## Where to Buy

The _Beta75X_ can be bought from a number of vendors, including:

- [GetFPV](https://www.getfpv.com/beta75x-2s-brushless-whoop-micro-quadcopter-xt30-frsky.html)
- [Amazon](https://www.amazon.com/BETAFPV-Beta75X-Brushless-Quadcopter-Smartaudio/dp/B07H86XSPW)

In addition you will need:

- An RC transmitter. _Beta75X_ can ship with a number of receivers. PX4 is compatible with all of them, but make sure to select the version that matches your transmitter.
- LiPo battery charger (vehicle ships with one battery, but you may want spares).
- FPV goggles if you want to fly FPV.
  There are many compatible options, including these ones from [Fatshark](https://www.fatshark.com/product-page/dominator-v3).

  ::: info
  FPV support is completely independent of PX4/flight controller.
  :::

## Flashing PX4 Bootloader

The _Beta75X_ comes preinstalled with Betaflight.

Before loading PX4 firmware you must first install the PX4 bootloader.
Instructions for installing the bootloader can be found in the [Omnibus F4](../flight_controller/omnibus_f4_sd.md#bootloader) topic (this is the flight controller board on the _Beta75X_).

:::tip
You can always [reinstall Betaflight](../advanced_config/bootloader_update_from_betaflight.md#reinstall-betaflight) later if you want!
:::

## Installation/Configuration

Once the bootloader is installed, you should be able to connect the vehicle to _QGroundControl_ via a USB cable.

::: info
At time of writing _Omnibus F4_ is supported on the QGroundControl _Daily Build_, and prebuilt firmware is provided for the master branch only (stable releases are not yet available).
:::

To install and configure PX4:

- [Load PX4 Firmware](../config/firmware.md).
- [Set the Airframe](../config/airframe.md) to _BetaFPV Beta75X 2S Brushless Whoop_.
- Continue with [basic configuration](../config/index.md), including sensor calibration and radio setup.

## Video

<lite-youtube videoid="_-O0kv0Qsh4" title="PX4 running on the BetaFPV Whoop"/>
