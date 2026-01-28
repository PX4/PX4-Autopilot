# CUAV HV PM (고전압 전원 모듈)

The CUAV<sup>&reg;</sup> _HV_PM_ power module is a "high voltage" power module independently developed by CUAV.

:::tip
The _HV_PM_ is included in the CUAV V5+/V5 nano kit, but is also be sold separately.
비행 콘트롤러(Pixhack v3, V5 + / V5 nano, Pixhawk)에 따라 케이블이 다릅니다.
다른 비행 컨트롤러에 사용시에는 케이블 핀을 수정하는 경우도 있습니다.
:::

## 사양

- **Higher voltage input:** 10V-60V (3s~14s battery)
- **Accurate battery monitor:**
  - **Voltage detection accuracy:** +-0.1v;
  - **Current detection accuracy:** +-0.2A
- **BEC (5v) max current:** 5A
- **Max (detection) current:** 60A
- **Max output current (ESC/MOTOR PORT):** 60A

## 구매처

[CUAV aliexpress store](https://www.aliexpress.com/item/32841805115.html?spm=2114.12010615.8148356.1.64165998hPvTKQ)

## 핀배열

![HV PM](../../assets/hardware/power_module/cuav_hv/hv_pm.jpg)

## HV PM 활성화

[Battery Estimation Tuning](../config/battery.md) describes how to configure the battery and power module.

The key configuration settings for `HV_PM` are:

- **Voltage divider:** 18
- **Amps per volt:** 24 A/V
