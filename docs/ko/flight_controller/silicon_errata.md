# 실리콘 정오표

This page lists known issues with silicon (hardware) errata of 3rd-party parts (micro controller, sensors, etc.) used on the Pixhawk board series. 실리콘 오류 유형에 따라 이러한 오류는 소프트웨어에서 수정할 수 없으며 특정 제한이 적용될 수 있습니다.

## FMUV2/ 픽쇼크 실리콘 에라타

### STM32F427VIT6 (정오표)

플래시 뱅크 2 및 전속 USB 장치 전용.

rev 2까지의 실리콘 개정판(개정판 3은 영향을받지 않음)은 USB 데이터 라인중 하나 인 PA12에 활동이있는 동안 두 번째 플래시 뱅크에 액세스시 오류/데이터 손상을 일으킬 수 있습니다. 플래시 뱅크 #2를 사용하지 않는 것 외에는 해결 방법은 없습니다.
Since USB is needed to program the device, Pixhawk revisions built with silicon revisions < rev 3 can only use up to 1MB of the 2MB flash of the microprocessor.

:::tip
The errata is fixed in later versions, but this may not be detected if you are using an older bootloader.
See [Firmware > FMUv2 Bootloader Update](../config/firmware.md#bootloader) for more information.
:::

## FMUv1 / Pixhawk 실리콘 정오표

알려진 문제가 없습니다.
