# 芯片勘误表

This page lists known issues with silicon (hardware) errata of 3rd-party parts (micro controller, sensors, etc.) used on the Pixhawk board series. Depending on the type of silicon error, these cannot be fixed in software and might impose specific limitations.

## FMUv2 / Pixhawk Silicon Errata

### STM32F427VIT6 (errata)

Flash Bank 2 and full speed USB device exclusive.

Silicon revisions up to rev 2 (revision 3 is the first not affected) can produce errors / data corruption when accessing the 2nd flash bank while there is activity on PA12, which is one of the USB data lines. There is no workaround / software fix for this, except to not use the flash bank #2.
Since USB is needed to program the device, Pixhawk revisions built with silicon revisions < rev 3 can only use up to 1MB of the 2MB flash of the microprocessor.

:::tip
The errata is fixed in later versions, but this may not be detected if you are using an older bootloader.
See [Firmware > FMUv2 Bootloader Update](../config/firmware.md#bootloader) for more information.
:::

## FMUv1 / Pixhawk Silicon Errata

No known issues.
