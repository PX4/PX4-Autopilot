# PX4 개발

신규 기체 개발, 기체 변경, 비행 알고리즘 수정, 신규 모드 추가, 신규 하드웨어 통합, 그리고 PX4 통신 방법 등을 설명합니다.

:::tip
This section is for software developers and (new) hardware integrators.
기존 기체 조립이나 PX4 비행 방법을 설명하지 않습니다.
:::

다음과 같은 것을 설명합니다.

- Get a [minimum developer setup](../dev_setup/config_initial.md), [build PX4 from source](../dev_setup/building_px4.md) and deploy on [numerous supported autopilots](../flight_controller/index.md).
- Understand the [PX4 System Architecture](../concept/architecture.md) and other core concepts.
- 플라이트 스택과 미들웨어 수정 방법을 배웁니다:
  - Modify flight algorithms and add new [flight modes](../concept/flight_modes.md).
  - Support new [airframes](../dev_airframes/index.md).
- PX4에 새 하드웨어를 조합하는 방법을 배웁니다:
  - 카메라, 거리 센서 등과 같은 신규 센서와 액츄에이터를 지원합니다.
  - 신규 자동조종장치에서 실행하도록 PX4 수정합니다.
- [Simulate](../simulation/index.md), [test](../test_and_ci/index.md) and [debug/log](../debug/index.md) PX4.
- 외부 로보틱스 API와 통신/통합합니다.

## 주요 개발자 링크

- [Support](../contribute/support.md): Get help using the [discussion boards](https://discuss.px4.io//) and other support channels.
- [Weekly Dev Call](../contribute/dev_call.md): A great opportunity to meet the PX4 dev team and discuss platform technical details (including pull requests, major issues, general Q&A).
- [Licences](../contribute/licenses.md): What you can do with the code (free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause)!)
- [Contributing](../contribute/index.md): How to work with our [source code](../contribute/code.md).
