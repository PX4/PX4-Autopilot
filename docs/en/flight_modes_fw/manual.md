# Manual Mode (Fixed-Wing)

<!-- this requires review -->

<img src="../../assets/site/difficulty_hard.png" title="Hard to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;

_Manual mode_ sends manual stick input directly to control allocation for fully manual control.

This is the hardest mode to fly, because nothing is stabilised.
Unlike [Acro Mode](../flight_modes_fw/acro.md), if the roll-pitch stick is centered the vehicle will not automatically stop rotating around the axis — the pilot actually has to move the stick to apply force in the other direction.

::: info
This is the only mode that overrides the FMU (commands are sent via the safety coprocessor).
It provides a safety mechanism that allows full control of throttle, elevator, ailerons and rudder via RC in the event of an FMU firmware malfunction.
:::

## Technical Description

Manual mode where stick input is sent directly to control allocation (for "fully" manual control).

This is the only mode that overrides the FMU (commands are sent via the safety coprocessor). It provides a safety mechanism that allows full control of throttle, elevator, ailerons and rudder via RC in the event of an FMU firmware malfunction.

## Parameters

| Parameter                                                                                    | Description                                                                                                                                                                                |
| -------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="FW_MAN_P_SC"></a>[FW_MAN_P_SC](../advanced_config/parameter_reference.md#FW_MAN_P_SC) | Manual pitch scale. Scale factor applied to the desired pitch actuator command in full manual mode. This parameter allows to adjust the throws of the control surfaces. Default: 1.0 norm. |
| <a id="FW_MAN_R_SC"></a>[FW_MAN_R_SC](../advanced_config/parameter_reference.md#FW_MAN_R_SC) | Manual roll scale. Scale factor applied to the desired roll actuator command in full manual mode. This parameter allows to adjust the throws of the control surfaces. Default: 1.0 norm.   |
| <a id="FW_MAN_Y_SC"></a>[FW_MAN_Y_SC](../advanced_config/parameter_reference.md#FW_MAN_Y_SC) | Manual yaw scale. Scale factor applied to the desired yaw actuator command in full manual mode. This parameter allows to adjust the throws of the control surfaces. Default: 1.0 norm.     |
