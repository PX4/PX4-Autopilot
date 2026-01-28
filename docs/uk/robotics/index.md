# Drone Apps & APIs

API дронів дозволяють вам писати код для керування та інтеграції з транспортними засобами на базі PX4, не потребуючи ретельного розуміння деталей транспортного засобу та стеку польоту, або думати про критичну безпекову поведінку.

Наприклад, ви можете створити нові "розумні" режими польоту, або власні режими геозон, або інтегрувати нове обладнання.
Drone APIs allow you to do this using high level instructions in your programming language of choice, and the code can then run on-vehicle in a [companion computer](../companion_computer/index.md), or from a ground station.
Under the the hood the APIs communicate with PX4 using [MAVLink](../middleware/mavlink.md) or [uXRCE-DDS](../middleware/uxrce_dds.md).

PX4 підтримує наступні інструменти SDK/робототехніки:

- [MAVSDK](../robotics/mavsdk.md)
- [ROS 2](../ros/index.md)
- [ROS 1](../ros/index.md)

## Який API мені слід використовувати?

We recommend using MAVSDK where possible, primarily because it is more intuitive and easier to learn, and can run on more operating systems and less performant-hardware.

Ви можете віддати перевагу ROS, якщо ви вже знаєте, як ним користуватися, або якщо ви хочете використовувати вже існуючі інтеграції (наприклад, завдання комп'ютерного зору).
Загалом, ROS, ймовірно, буде кращим вибором для завдань, які вимагають дуже низької затримки або більш глибокої інтеграції з PX4, ніж це забезпечує MAVLink.

Основна різниця:

- **MAVSDK:**
  - Інтуїтивно зрозумілий та оптимізований для дронів, з невеликою тривалістю навчання та простим налаштуванням.
  - Ви можете писати програми на C++, Python, Swift, Java, Go та інших мовах.
  - Працює на обладнанні з обмеженими ресурсами
  - Працює на широкому спектрі ОС, включаючи Android, Linux, Windows.
  - Комунікація через MAVLink.
    - Стабільний і широко підтримуваний.
    - Обмежено можливостями MAVLink - потрібна інформація може бути не опрацьована.
    - Затримка може бути занадто великою для деяких випадків використання.
- **ROS 2**
  - General-purpose robotics API that has been extended to support drone integration:
    - Conceptually not as well optimised for drones as MAVSDK
    - Значний час навчання
  - Багато готових бібліотек: корисно для повторного використання коду.
  - Підтримуються бібліотеки C++ та Python
  - Працює на Linux
  - ROS 2 is the latest version, which connects via XRCE-DDS.
    - Інтерфейсний шар DDS дозволяє глибоку інтеграцію з будь-яким аспектом PX4, який виставляється як тема UORB (майже з усім).
    - Можна забезпечити значно нижчу затримку.
    - Still under development:
      - At time of writing requires a deeper understanding of PX4 than ROS 1.
      - Message interface with PX4 not stable/maintained across ROS and PX4 releases.

## Deprecated APIs

### ROS 1

While not strictly deprecated, ROS 1 is at its final LTS Release version "Noetic Ninjemys", which reaches end-of-life in May 2025.
That means no new features or bug fixes will be provided, and even security updates will cease in 2025.

ROS 1 still "works" on PX4 because it uses MAVROS, a MAVLINK-ROS abstraction as an integration layer.
This means that ROS 1 has all the limitations of MAVLink, such as a higher latency and a small API surface (and also the benefits, such as a stable interface).

All PX4 investment in ROS is going towards a deep integration with ROS 2.
This will essentially allow ROS 2 applications to be close to indistinguishable from code running in PX4 itself.

:::tip
Use ROS 2 for new projects.
Upgrade to ROS 2 for existing projects as soon as possible.
:::

### DroneKit

DroneKit-Python is a MAVLink API written in Python.
It is not optimised for use with PX4, and has not be maintained for some years.
Legacy docs for using PX4 and DroneKit can be found here: [PX4 v1.12 > DroneKit](https://docs.px4.io/v1.12/en/robotics/dronekit.html).

:::tip
[MAVSDK](https://mavsdk.mavlink.io/) is the recommended MAVLink API for use with PX4.
It is better in almost every way: features, speed, programming language support, maintenance, and so on.
:::
