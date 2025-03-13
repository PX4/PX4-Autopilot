# Розробка PX4

Ця секція пояснює як почати підтримувати нові типи літальних апаратів та їх варіанти, модифікувати польотні алгоритми, додавати нові режими, інтегрувати нові пристрої, і комунікувати з польотним контролером PX4 ззовні.

:::tip
This section is for software developers and (new) hardware integrators.
Вона не потрібна якщо Ви будуєте на існуючій рамі або літаєте використовуючи апарат на основі PX4.
:::

Тут пояснюється, як:

- Get a [minimum developer setup](../dev_setup/config_initial.md), [build PX4 from source](../dev_setup/building_px4.md) and deploy on [numerous supported autopilots](../flight_controller/index.md).
- Understand the [PX4 System Architecture](../concept/architecture.md) and other core concepts.
- Дізнатись як модифікувати польотний стек та мідлварь (проміжне ПЗ):
  - Modify flight algorithms and add new [flight modes](../concept/flight_modes.md).
  - Support new [airframes](../dev_airframes/index.md).
- Дізнатися, як інтегрувати PX4 з новим обладнанням:
  - Підтримувати нові сенсори, актуатори, включаючи камері, далекоміри, тощо.
  - Модифікувати PX4 для роботи на новому залізі для автопілотів.
- [Simulate](../simulation/index.md), [test](../test_and_ci/index.md) and [debug/log](../debug/index.md) PX4.
- Комунікувати/інтегрувати з зовнішніми робототехнічними API.

## Ключові посилання для розробника

- [Support](../contribute/support.md): Get help using the [discussion boards](https://discuss.px4.io//) and other support channels.
- [Weekly Dev Call](../contribute/dev_call.md): A great opportunity to meet the PX4 dev team and discuss platform technical details (including pull requests, major issues, general Q&A).
- [Licences](../contribute/licenses.md): What you can do with the code (free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause)!)
- [Contributing](../contribute/index.md): How to work with our [source code](../contribute/code.md).
