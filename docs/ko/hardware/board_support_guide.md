# 제조사의 PX4 보드 지원 가이드

The PX4 development and test teams fully support and maintain boards that are compliant with the [Pixhawk Standard](https://pixhawk.org/standards/).
표준을 준수하지 않거나 새로운 보드를 제작하는 업체에서는 호환성 차이를 지원하여야 합니다.

This guide outlines the [general requirements](#general_requirements) for board support, along with the additional requirements for the different [board support categories](#board-support-categories).

:::info
Boards that are not compliant with the requirements are [unsupported](#unsupported); they will not be listed on the PX4 website hardware list and will be removed from the codebase.
:::

<a id="general_requirements"></a>

## 일반 요구사항

지원 모든 보드에 대한 일반 요구사항은 다음과 같습니다.

1. 하드웨어는 시장에서 사용할 수 있어야 합니다.

2. 보드에는 UAV에서 PX4와 함께 보드를 사용하는 것이 불가능하거나 위험하게 하는 버그나 허용 범위 초과하는  품질 이상이 없어야 합니다.
   보드는 부품과 조립품의 품질을 보장하기 위해 승인 기준을 통과하여야 합니다.

3. 고객을 지원하고 고객이 연락할 수 있는 명확하고 쉬운 방법이 있어야 합니다.
   다음과 같은 방법이 허용됩니다.

   - PX4 Discord server presence
   - 지원 이메일
   - 전화번호

4. PX4 관리자를 위한 PoC(Point of Contact)(직접 이메일 또는 Slack/Forum/Github에서 사용 가능)

5. The board needs to use the [PX4 bootloader protocol](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/nuttx/src/bootloader).
   For more information on bootloaders see: [PX4 Nuttx Porting Guide > Bootloader](../hardware/porting_guide_nuttx.md#bootloader).

6. 다음 내용을 포함하는 적절한 문서:

   - PX4 핀 정의를 아래에 매핑하는 완전한 핀배열 공개:
      1. 마이크로컨트롤러 핀
      2. 물리적 외부 커넥터
   - A block diagram or full schematic of the main components (sensors, power supply, etc.) that allows to infer software requirements and boot order
   - A manual of the finished product detailing its use

7. There must be a dedicated webpage for the board with PX4, which lists the features and limitations for usage with PX4, and includes or links to the above described documentation.

## 보드 지원 카테고리

보드 지원 범주는 다음과 같습니다. The autopilot boards in each category are listed at: [https://px4.io/autopilots/.](https://px4.io/autopilots/)

:::info
Manufacturer supported boards may be as well/better supported than Pixhawk boards (for example through economies of scale).
:::

## Pixhawk표준

Pixhawk 보드는 Pixhawk 표준을 준수하는 보드입니다. These standards are laid out on [http://pixhawk.org](http://pixhawk.org/), but at high-level require that the board passes electrical tests mandated by the standard and the manufacturer has signed the Pixhawk adopter and trademark agreement.

PX4는 일반적으로 상업적으로 사용 가능한 보드만 지원하므로, 일반적으로 지난 5년 이내에 출시된 보드 표준이 지원됩니다.

<a id="ver_rev_id"></a>

### VER 및 REV ID(하드웨어 개정 및 버전 감지)

FMUv5 이상에는 전기 감지 메커니즘이 있습니다.
선택적 구성 데이터와 결합된 이 감지는 필수 장치 및 전원 공급 장치 구성과 관련하여 하드웨어 구성을 정의합니다. Manufacturers must obtain the VER and REV ID from PX4 board maintainers by issuing a PR to ammend the [DS-018 Pixhawk standard](https://github.com/pixhawk/Pixhawk-Standards) for board versions and revisions.

Because these boards are 100% compliant with the Pixhawk standard, the values assigned for VER and REV ID are the defaults for that FMU Version.

## 지원 제조업체

이러한 보드는 제조업체에서 지원합니다.
이 범주에 해당하려면 보드는 해당 릴리스로부터 4개월 이내에 최신 안정 PX4 릴리스에서 작동하여야 합니다.

- 제조업체가 직접 지원합니다.
- 제조업체는 코어 개발 팀에 최소 2개의 보드를 공급하여야 합니다(테스트 랙 및 테스트 팀에서 사용하기 위하여).

:::tip
While there is no commitment from the PX4 maintainers and the flight test team to support and test boards in this category, we strongly recommended PX4 and manufacturer teams build close working relationships.
이것은 모든 당사자에게 더 나은 결과를 가져올 것입니다.
:::

:::info
These boards will be assigned [VER and REV ID](#ver_rev_id) based on compatibility.
보드가 FMU 사양의 변형이고 동일한 바이너리를 실행 가능하고 제조업체에서 지원하는 약간의 차이가 있는 경우에는 PX4에서 특별 할당을 수행합니다.
Contact the PX4 maintainer at [boards@px4.io](mailto:boards@px4.io) to request more information.
:::

## 실험

These boards are all boards that don't fall in the above categories, or don't fall in those categories _anymore_.
다음 요구 사항이 적용됩니다.

- 보드는 정의된 차량 유형에 대해 최소 하나의 PX4 릴리스에 작동하여야 하지만 반드시 최신 릴리스일 필요는 없습니다.

:::info
Experimental boards that were _previously_ Pixhawk or Manufacturer supported will have/retain their original IDs.
_New_ experimental boards are allocated [VER and REV IDs](#ver_rev_id) based on compatibility, in the same way as Manufacturer Supported boards.
:::

## 미지원

이 범주에는 PX4 프로젝트 또는 제조업체에서 지원하지 않고 "실험적" 지원에 해당하지 않는 모든 보드가 포함됩니다.

- 보드는 우리가 이미 지원하는 것과 문서상 어느 정도 호환되며 "실험적"으로 올리려면, 최소한의 노력이 필요하지만 개발 팀이나 제조업체 모두 현재 이를 추구하지 않습니다.
- Manufacturer/Owner of hardware violates our [Code of Conduct](https://discuss.px4.io/t/code-of-conduct/13655)
- 라이선스 제한으로 인해 보드 지원을 추가하는 데 필요한 도구/libs/drivers/etc가 호환되지 않는 것으로 간주되는 비공개 소스
- 보드가 일반 요구 사항에 명시된 최소 요구 사항을 충족하지 않습니다.

:::info
Unsupported boards will NOT be assigned [VER and REV ID](#ver_rev_id) (and cannot run PX4 FMUvX firmware).
:::

## 릴리스 프로세스

제조업체가 보드가 특정 범주에 속한다고 선언시, 보드는 해당 범주 및 일반 요구 사항에 대한 요구 사항을 준수한다고 가정합니다.

제조업체 지원 또는 실험 범주에 속하는 새 보드가 시장에 출시되면, 제조업체는 PX4 문서를 업데이트하고 PX4에서 보드 릴리스 프로세스를 수행할 책임이 있습니다. 다음 단계를 수행하는 것이 좋습니다.

Contact PX4 board maintainers at [boards@px4.io](mailto:boards@px4.io) and request the following:

1. The assignment of a _board id_ for bootloader and firmware selection in QGC.
2. The assignment of REV and VER ID resistor values.
3. If the board supports USB: Either request the assignment of a USB VID and PID or provide the USB VID and PID.

Integrate the board according to the board porting release process described in the [porting guide](../hardware/porting_guide.md)

:::warning
The board support process may be changed and improved over time.
Hardware manufacturers are encouraged to contribute to this process through the regular hardware call, the Discuss forum or Discord.
:::

## 지원

보드 지원 가이드/프로세스의 일부가 명확하지 않은 경우:

- Ask the community for help on Discord channels under `Hardware` category, or on the discuss forum
- 정규 하드웨어 회의 참석
- Consultancy options are listed here: [https://px4.io/community/consultants/](https://px4.io/community/consultants/)
