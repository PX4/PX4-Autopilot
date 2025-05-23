# 电调（ESC）校准

:::info
These instructions are only relevant to [PWM ESCs](../peripherals/pwm_escs_and_servo.md) and [OneShot ESCs](../peripherals/oneshot.md).
[DShot](../peripherals/dshot.md) and [CAN](../can/index.md) ESCs ([DroneCAN](../dronecan/escs.md)/Cyphal) do not require this kind of calibration.
:::

电子速度控制器(ESC)根据飞行控制器的输入指令（FC）调节电机的速度（和方向）。
电调响应的输入范围是可配置的，甚至同一型号的不同电调之间的默认范围也是不同的。

此校准将使用来自飞行控制器的固定最大值 (2000us) 和最小值 (1000us) PWM 输入更新所有ESC。
因此，载具上的所有ESC/电机都将以同样的方式在整个输入范围内响应飞行控制器的输入。

建议使用此工具校准支持它的所有 PWM/OneShot ESC。

:::info
Calibration is particularly important for low-cost ESC, as they commonly vary a lot in their response to input.

然而，也建议为高质量的控制员提供这种校准。
即使这些都是工厂校准的，而且都应以同样的方式作出反应，但实际上输入范围可能有所不同。
例如，如果控制器在离开工厂后被手动校准，它可能不再以同样的方式动作。
:::

:::warning
If you want to use an ESC that does not support this calibration, then it must be factory calibrated and respond consistently out of the box.
This should be verified using [Actuator Testing](../config/actuators.md#actuator-testing).
Jump to the [actuator configuration step (7)](#actuatorconfig_step) (which is still important).
:::

OneShot ESCs should be [configured to use OneShot](../peripherals/oneshot.md#px4-configuration) before calibration. 您应该在更换ESC后校准，即使您先前已校准。

## 操作前提

校准序列要求您能够保持飞行控制器的供电，同时手动对ESC进行上电循环。

如果使用 Pixhawk 飞行控制器，推荐这样做的方式是通过USB单独为飞行控制器提供电力。 并在需要时连接/断开ESC的电池供电。
Flight control systems that can't power the autopilot via USB will need a [different approach](#problem_power_module).

如果电池通过电源模块连接，校准程序可以检测电池连接并用它来驱动校准序列。
如果无法检测到电池电力，则根据超时执行校准顺序。

## 步骤

校准电调：

1. 卸下螺旋桨。

   :::warning
   Never attempt ESC calibration with propellers on!

   The motors _should_ not spin during ESC calibration.
   但是，如果校准是在ESC 已经供电后开始的，或者如果ESC 对校准序列支持/检测不恰当， 它将响应PWM的输入，以最大速度运行电机。

:::

2. Map the ESCs you're calibrating as motors in the vehicle's [Actuator Configuration](../config/actuators.md).
   只有映射的驱动器才能获得输出，并且只有被映射为电机的ESC将被校准。

3. 拔下电池，断开ESC电源。
   飞行控制器必须保持供电，例如将USB连接到地面站。

4. Open the _QGroundControl_ **Settings > Power**, then press the **Calibrate** button.

   ![ESC Calibration step 1](../../assets/qgc/setup/esc/qgc_esc_calibration.png)

5. 启动校准序列后，在没有错误的情况下，直接给 ESC供电 (您应该被提示):

   ![ESC Calibration step 2](../../assets/qgc/setup/esc/esc_calibration_step_2.png)

   校准将自动开始:

   ![ESC Calibration step 3](../../assets/qgc/setup/esc/esc_calibration_step_3.png)

6. 在校准过程中，您应该听到来自ESC的特定模型不同的嘀音，它表明校准的各个步骤。

   校准完成后会提示您。

   <a id="actuatorconfig_step"></a>
   ![ESC Calibration step 4](../../assets/qgc/setup/esc/esc_calibration_step_4.png)

7. Go back to the [Actuator Configuration](../config/actuators.md) section.

   在ESC 校准后，所有具有相同(重新)校准的 ESC的电机对同样的输入应以同样的方式动作。 驱动器配置中默认的 PWM 输出设置现在应该能开箱即用。

   你需要确认电机确实正常工作。
   由于默认配置值已经设置为保守的设置，您可能也希望调整它们以适用于您的特定的 ESC。

   ::: info
   The steps below are similar to those described in [Actuator Configuration > Motor Configuration](../config/actuators.md#motor-configuration).

:::

   验证以下值：

   - The minimum value for a motor (default: `1100us`) should make the motor spin slowly but reliably, and also spin up reliably after it was stopped.

      You can confirm that a motor spins at minimum (still without propellers) in [Actuator Testing](../config/actuators.md#actuator-testing), by enabling the sliders, and then moving the test output slider for the motor to the first snap position from the bottom.
      当你将滑块从解锁到最小值时，正确的值应该使电机立即和可靠地旋转。

      要找到“最佳”最小值，请将滑块移动到底部(禁用)。
      Then increase the PWM output's `disarmed` setting in small increments (e.g. 1025us, 1050us, etc), until the motor starts to spin reliably (it is better to be a little too high than a little too low).
      Enter this value into the `minimum` setting for all the motor PWM outputs, and restore the `disarmed` output to `1100us`.

   - The maximum value for a motor (default: `1900us`) should be chosen such that increasing the value doesn't make the motor spin any faster.

      You can confirm that the motor spins quickly at the maximum setting in [Actuator Testing](../config/actuators.md#actuator-testing), by moving the associated test output slider to the top position.

      To find the "optimal" maximum value, first move the slider to the bottom (disarmed).
      Then increase the PWM output's `disarmed` setting to near the default maximum (`1900`) - the motors should spin up.
      Listen to the tone of the motor as you increase the PWM maximum value for the output in increments (e.g. 1925us, 1950us, etc).
      The optimal value is found at the point when the sound of the motors does not change as you increase the value of the output.
      Enter this value into the `maximum` setting for all the motor PWM outputs, and restore the `disarmed` output to `1100us`.

   - The disarmed value for a motor (default: `1000us`) should make the motor stop and stay stopped.

      You can confirm this in [Actuator Testing](../config/actuators.md#actuator-testing) by moving the test output slider to the snap position at the bottom of the slider and observing that the motor does not spin.

      If the ESC spins with the default value of 1000us then the ESC is not properly calibrated.
      If using an ESC that can't be calibrated, you should reduce the PWM output value for the output to below where the motor does not spin anymore (such as 950us or 900us).

   ::: info
   VTOL and fixed-wing motors do not need any special PWM configuration.
   With the default PWM configuration they will automatically stop during flight when commanded by the autopilot.

:::

## 故障处理

1. Calibration can state that it has succeeded even when it has failed.

   This happens if you do not power the ESC at the right time, or the ESCs don't support calibration.
   This occurs because PX4 has no feedback from the ESC to know whether or not calibration was successful.
   You have to rely on interpreting the beeps during the calibration and subsequent motor tests to know for sure that the calibration worked.

   <a id="problem_power_module"></a>

2. Calibration cannot be started if you have a power module configured and connected (for safety reasons).

   Unplug power to the ESCs first.
   If you're blocked because a power module is necessary to keep your flight controller alive, but you can (un)power the ESCs separately, you can temporarily disable the detection of the power module just for the ESC calibration using the parameters [BATn_SOURCE](../advanced_config/parameter_reference.md#BAT1_SOURCE). Once the power module that's powering the autopilot is not detected as battery anymore a timing based calibration is possible.

3. PX4 will abort calibration (for safety reasons) if the system detects an increase in current consumption immediately after initiating calibration.
   This requires a power module.
