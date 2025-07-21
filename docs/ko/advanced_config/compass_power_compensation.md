# 나침반 전력 보정

케이블은 나침반 판독 값을 왜곡시킬 수 있는 자기장을 유도하므로, 큰 전류가 흐르는 케이블에서 최대한 멀리 나침반(자력계)을 장착해야합니다.

이 항목에서는 나침반 이동이 현실적으로 불가능할 경우에 유도된 자기장을 보정하는 방법을 설명합니다.

:::tip
Moving the compass away from power-carrying cables is the easiest and most effective way to fix this issue, because the strength of the magnetic fields decreases quadratically with the distance from the cable.
:::

:::info
The process is demonstrated for a multicopter, but is equally valid for other vehicle types.
:::

<a id="when"></a>

## 전력 보정은 언제 적용됩니까?

전력 보정은 아래의 사항들이 모두 해당되는 경우에만 권장됩니다.

1. 나침반과 전력 공급 케이블 사이의 거리를 띄울 수 없을 때.

2. There is a strong correlation between the compass readings and the thrust setpoint, and/or the battery current.

   ![Corrupted mag](../../assets/advanced_config/corrupted_mag.png)

3. 드론 케이블이 모두 제자리에 고정되어 이동할 수 없는 경우. (전류가 흐르는 케이블이 움직일 수 있다면 계산된 보정 매개 변수가 유효하지 않습니다)

<a id="how"></a>

## 나침반 보정 방법

1. 드론의 펌웨어 버전이 전력 보정을 지원하는지 확인하십시오.  (현재 마스터 버전 또는 v.1.11.0 릴리즈)

2. Perform the [standard compass calibration](../config/compass.md#compass-calibration).

3. Set the parameter [SDLOG_MODE](../advanced_config/parameter_reference.md#SDLOG_MODE) to 2 to enable logging of data from boot.

4. Set the parameter [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE) checkbox for _Sensor comparison_ (bit 6) to get more data points.

5. 드론이 움직이지 않도록 고정하고 프로펠러를 부착하십시오(모터가 비행 중과 동일한 전류를 소비하게 됩니다).
   이 예시에서는 끈으로 드론을 고정시킵니다.

   ![strap](../../assets/advanced_config/strap.png)

6. Power the vehicle and switch into [ACRO flight mode](../flight_modes_mc/acro.md) (using this mode ensures the vehicle won't attempt to compensate for movement resulting from the straps).

   - 기체에 시동을 걸고 스로틀을 천천히 최대로 올립니다.
   - 스로틀을 천천히 0까지 낮춥니다.
   - 기체 시동을 끄십시오.
      &#062; <strong x-id="1">참고</strong> 진동을 면밀히 관찰하고, 신중하게 테스트를 진행하십시오.

   ::: info
   Perform the test carefully and closely monitor the vibrations.

:::

7. Retrieve the ulog and use the python script [mag_compensation.py](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/sensors/vehicle_magnetometer/mag_compensation/python/mag_compensation.py) to identify the compensation parameters.

   ```sh
   python mag_compensation.py ~/path/to/log/logfile.ulg <type> [--instance <number>]
   ```

   여기서:

   - `<type>`: `current` or `thrust` (power signal used for compensation)
   - `--instance <number>` (optional): The number is `0` (default) or `1`, the instance of the current or thrust signal to use.

   ::: info
   If your log does not contain battery current measurements, you will need to comment out the respective lines in the Python script, such that it does the calculation for thrust only.

:::

8. 스크립트는 추력 및 전류에 대한 매개변수 식별값을 계산하여 콘솔에 인쇄할 것입니다.
   스크립트에서 나타나는 그림은 각 나침반 인스턴스에 대한 "적합성"과 제안된 값으로 데이터가 보정된 경우의 모습을 보여줍니다.
   전류 측정이 가능한 경우,  일반적으로 전류 보정을 사용하면 더 나은 결과를 얻을 수 있습니다.
   다음은 현재 적합도는 양호하지만 관계가 선형이 아니므로 추력 매개 변수를 사용할 수 없는 로그의 예입니다.

   ![line fit](../../assets/advanced_config/line_fit.png)

9. Once the parameters are identified, the power compensation must be enabled by setting [CAL_MAG_COMP_TYP](../advanced_config/parameter_reference.md#CAL_MAG_COMP_TYP) to 1 (when using thrust parameters) or 2 (when using current parameters).
   또한 각 나침반의 각 축에 대한 보정 매개 변수를 설정해야합니다.

   ![comp params](../../assets/advanced_config/comp_params.png)
