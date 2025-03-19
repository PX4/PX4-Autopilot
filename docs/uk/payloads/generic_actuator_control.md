# Керування Загальним Приводом

Ви можете під'єднати довільне обладнання до невикористаних виводів PX4 і керувати ним за допомогою [Пристрою дистанційного керування](#generic-actuator-control-with-rc) або [MAVLink](#generic-actuator-control-with-mavlink) (як окремі команди, так і у [місії](#generic-actuator-control-in-missions)).

Це корисно, коли вам потрібно використовувати тип корисного навантаження, для якого немає пов’язаної команди MAVLink, або для якого PX4 не має відповідної інтеграції.

:::info
Віддавайте перевагу використанню інтегрованого апаратного забезпечення та команд MAVLink відповідно до конкретного апаратного забезпечення, наприклад для [Захватів](../peripherals/gripper.md), замість керування загальним приводом, коли це можливо.
Використання інтегрованого обладнання дозволяє оптимізувати [планування та поведінку місії](../flying/package_delivery_mission.md), оскільки місія може знати ключові факти про апаратне забезпечення, наприклад, скільки часу потрібно для запуску.
:::

## Керування загальним приводом за допомогою MAVLink

[MAV_CMD_DO_SET_ACTUATOR](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ACTUATOR) може бути використано для встановлення значення для до 6 приводів (одночасно).
Цю команду можна використовувати в [місіях](#generic-actuator-control-in-missions), створивши елемент місії «Set actuator», або як окрему команду.

Виводи, якими потрібно керувати, вказуються на екрані конфігурації [Actuators](../config/actuators.md#actuator-outputs) шляхом призначення функцій від `Peripheral via Actuator Set 1` до `Peripheral via Actuator Set 6` для бажаних [виводів приводу](../config/actuators.md#actuator-outputs).

![Налаштування виводу загального приводу в QGC](../../assets/peripherals/qgc_generic_actuator_output_setting_example.png)

`MAV_CMD_DO_SET_ACTUATOR` `param1` до `param6` контролюють виводи, пов'язані `Peripheral via Actuator Set 1` до `Peripheral via Actuator Set 6` відповідно.

Наприклад, на зображенні вище, вивід `AUX5` пов'язано з функцією `Peripheral via Actuator Set 1`.
Щоб керувати приводом, підключеним до `AUX5`, потрібно встановити значення `MAV_CMD_DO_SET_ACTUATOR.param1`.

<!-- PX4 v1.14 bug https://github.com/PX4/PX4-Autopilot/issues/21966 -->

## Керування загальним приводом за допомогою RC

За допомогою каналів пристрою дистанційного керування можна керувати до 6 PWM чи CAN виводами автопілота.
Виводи, якими потрібно керувати, вказуються на екрані конфігурації [Actuators](../config/actuators.md#actuator-outputs) шляхом призначення функцій від `RC AUX 1` до `RC AUX 6` потрібним [виводам приводу](../config/actuators.md#actuator-outputs).

Щоб зіставити певний RC канал із функцією виводу `RC AUX n` (і, отже, з її призначеним виводом), ви використовуєте параметр [RC_MAP_AUXn](../advanced_config/parameter_reference.md#RC_MAP_AUX1), який має той самий номер `n`.

Наприклад, щоб керувати приводом, приєднаним до AUX контакту 3 (скажімо), ви повинні призначити функцію виводу `RC AUX 5` виводу `AUX3`.
Потім ви можете використовувати RC канал для керування виводом `AUX3` за допомогою `RC_MAP_AUX5`.

## Керування загальним приводом у місіях

Щоб використовувати керування загальним приводом у місії, ви повинні спочатку [налаштувати виводи, якими ви хочете керувати за допомогою MAVLink](#generic-actuator-control-with-mavlink).

Потім у _QGroundControl_ ви можете встановити значення виводів приводу в місії за допомогою елементу місії **Set actuator** (це додає [MAV_CMD_DO_SET_ACTUATOR](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ACTUATOR) до завантаженого плану місії).

Важливо зазначити, що при керуванні загальним приводом ані _QGroundControl_, ані PX4 нічого не знають про апаратне забезпечення, яке запускається.
Під час обробки елемента місії PX4 просто встановить виводи відповідно до наданих значень, а потім негайно перейде до наступного елемента місії.
Якщо апаратне забезпечення вимагає часу для активації, і вам потрібно зупинитися на поточній точці маршруту, щоб це сталося, тоді вам потрібно буде спланувати місію з додатковими елементами, щоб досягти бажаної поведінки.

:::info
Це одна з причин переваги інтегрованого апаратного забезпечення!
Це дозволяє будувати місії в загальному вигляді, з будь-якою поведінкою, що залежить від апаратного забезпечення, або часом, керованим конфігурацією політного стека.
:::

Щоб використовувати загальний привід у місії:

1. Створіть елемент місії waypoint, де вам потрібна команда приводу.

2. Змініть елемент місії waypoint на елемент місії «Set actuator»:

   ![Елемент місії Set actuator](../../assets/qgc/plan/mission_item_editors/mission_item_select_set_actuator.png)

   - Виберіть заголовок у редакторі маршрутної точки місії, щоб відкрити редактор **Select Mission Command**.
   - Виберіть категорію **Advanced**, а потім пункт **Set actuator** (якщо елемента немає, спробуйте новішу версію _QGroundControl_ або щоденну збірку).
      Це змінить тип елемента місії на «Set actuator».

3. Виберіть підключені приводи та встановіть їхні значення (вони нормалізовані між -1 і 1).

   ![Елемент місії Set actuator](../../assets/qgc/plan/mission_item_editors/set_actuator.png)

## MAVSDK (приклад скрипту)

Наступний [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html) [приклад коду](https://github.com/mavlink/MAVSDK/blob/main/examples/set_actuator/set_actuator.cpp) показує, як ініціювати випуск корисного навантаження за допомогою методу [`set_actuator()`](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_action.html#classmavsdk_1_1_action_1ad30beac27f05c62dcf6a3d0928b86e4c) плагіну MAVSDK Action.

Значення індексів в `set_actuator()` зіставляються з виводами корисного навантаження MAVLink, визначених для вашого планера.

:::info
MAVSDK надсилає MAVLink команду [MAV_CMD_DO_SET_ACTUATOR](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ACTUATOR).
:::

```cpp
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>

using namespace mavsdk;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage :" << bin_name << " <connection_url> <actuator_index> <actuator_value>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 4) {
        usage(argv[0]);
        return 1;
    }

    const std::string connection_url = argv[1];
    const int index = std::stod(argv[2]);
    const float value = std::stof(argv[3]);

    Mavsdk mavsdk;
    const ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found, exiting.\n";
        return 1;
    }

    // Get discovered system now.
    auto system = fut.get();

    // Instantiate plugins.
    auto action = Action{system};

    std::cout << "Setting actuator...\n";
    const Action::Result set_actuator_result = action.set_actuator(index, value);

    if (set_actuator_result != Action::Result::Success) {
        std::cerr << "Setting actuator failed:" << set_actuator_result << '\n';
        return 1;
    }

    return 0;
}
```

## Тестування

Корисні навантаження, які запускаються сервоприводами та іншими приводами, наприклад захватами, можна протестувати в [стані pre-arm](../getting_started/px4_basic_concepts.md#arming-and-disarming), який вимикає мотори, але дозволяє приводам рухатися.

Це безпечніше, ніж тестування, коли апарат увімкнено.

Корисне навантаження камери можна запустити та перевірити в будь-який час.