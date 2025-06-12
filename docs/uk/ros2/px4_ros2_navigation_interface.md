# Інтерфейс навігації PX4 ROS 2

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

:::warning
Експериментальні налаштування
На момент написання цієї статті, деякі частини бібліотеки інтерфейсу PX4 ROS 2 є експериментальними і, отже, можуть бути змінені.
:::

[PX4 ROS 2 Interface бібліотека](../ros2/px4_ros2_interface_lib. d) інтерфейс навігації дозволяє розробникам надсилати дані щодо позиції PX4 безпосередньо з ROS 2 додатків, така як система VIO або система відповідності мап.
Цей інтерфейс надає шар абстракції від PX4 та каркасу обміну повідомленнями uORB і вводить деякі перевірки на розумність стану оцінки оновлень, надісланих через інтерфейс.
Ці вимірювання потім об'єднуються в EKF так само, як внутрішні вимірювання PX4.

Бібліотека надає два класи: LocalPositionMeasurementInterface та GlobalPositionMeasurementInterface, які обидва використовують схожий метод оновлення для надання оновлення локальної позиції або глобальної позиції до PX4 відповідно.
Метод `update` очікує від позиції вимірювання `struct` ([`LocalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1__ros2_1_LocalPositionMeasurement.html) або [`GlobalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx__ros2_1_GlobalitionMeasurement.html)), які розробники можуть народитися власними вимірюваннями.

## Установка та перший тест

Для початку роботи потрібно виконати наступні кроки:

1. Make sure you have a working [ROS 2 setup](../ros2/user_guide.md), with [`px4_msgs`](https://github.com/PX4/px4_msgs) in the ROS 2 workspace.

2. Клонуйте репозиторій в робочий простір:

  ```sh
  cd $ros_workspace/src
  git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib
  ```

  :::info
  Для забезпечення сумісності, використовуйте останні _main_ гілки для PX4, _px4_msgs_ та бібліотеки.
  Дивіться також [here](https://github.com/Auterion/px4-ros2-interface-lib#compatibility-with-px4).

:::

3. Побудуйте робочий простір:

  ```sh
  cd ..
  colcon build
  ```

4. У іншій оболонці запустіть PX4 SITL:

  ```sh
  cd $px4-autopilot
  make px4_sitl gazebo-classic
  ```

  (тут ми використовуємо Gazebo-Classic, але ви можете використовувати будь-яку модель або симулятор)

5. У іншій оболонці запустіть агента micro XRCE (ви можете залишити його запущеним після цього):

  ```sh
  MicroXRCEAgent udp4 -p 8888
  ```

6. Поверніться до терміналу ROS 2, створіть робочу область, яку ви щойно створили (у кроці 3), і запустіть приклад [global_navigation](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/navigation/global_navigation), який періодично надсилає фіктивні оновлення глобальної позиції:

  ```sh
  source install/setup.bash
  ros2 run example_global_navigation_cpp example_global_navigation
  ```

  Ви повинні отримати вивід, подібний до цього, що показує, що глобальний інтерфейс успішно надсилає оновлення позиції:

  ```sh
  [INFO] [1702030701.836897756] [example_global_navigation_node]: example_global_navigation_node running!
  [DEBUG] [1702030702.837279784] [example_global_navigation_node]: Successfully sent position update to navigation interface.
  [DEBUG] [1702030703.837223884] [example_global_navigation_node]: Successfully sent position update to navigation interface.
  ```

7. У PX4 оболонці можна перевірити, що PX4 отримує глобальні оновлення позиції:

  ```sh
  listener aux_global_position
  ```

  Вихід має містити:

  ```sh
  TOPIC: aux_global_position
  aux_global_position
     timestamp: 46916000 (0.528000 seconds ago)
     timestamp_sample: 46916000 (0 us before timestamp)
     lat: 12.343210
     lon: 23.454320
     alt: 12.40000
     alt_ellipsoid: 0.00000
     delta_alt: 0.00000
     eph: 0.31623
     epv: 0.44721
     terrain_alt: 0.00000
     lat_lon_reset_counter: 0
     alt_reset_counter: 0
     terrain_alt_valid: False
     dead_reckoning: False
  ```

8. Тепер ви готові використовувати навігаційний інтерфейс для надсилання своїх оновлень.

## Як користуватися бібліотекою

Для надсилання вимірювання позиції ви заповнюєте структуру позиції з виміряними значеннями.
Потім викликаєте функцію оновлення інтерфейсу з цією структурою як аргументом.

Для базового прикладу, як користуватися цим інтерфейсом, ознайомтеся з [examples](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/navigation) в `Auterion/px4-rosface-lib` репозиторію, наприклад [examples/cpp/navigation/local_navigation](https://github.com/Auterion/px4-ros2-interface-lib/b/main/examples/cpp/navigation/local_navigation/inclation/inclde/local_localation.hppation.hpp) або examples/cpps/cppation/globation](https://github.com/Auter/intertere-face-face-face-facb/mainb/mppation/mppation/example/example/navigation/navigation/navigation/navig/navigation/navigation/navig/navig/navig/navighblob/navig

### Оновлення локальної позиції

Спочатку переконайтеся, що параметр PX4 [`EKF2_EV_CTRL`](../advanced_config/parameter_reference.md#EKF2_EV_CTRL) налаштований належним чином для ефективного використання зовнішніх локальних вимірів, встановивши відповідні біти в `true`:

- 0: Дані горизонтальної позиції
- 1: Дані вертикальної позиції
- 2: Дані швидкості
- 3: Дані кута yaw



1. Створіть [`LocalPositionMeasurementInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4ros2_1_1__ros2_1_1LocalPositionMeasurementInterface.html), надаючи йому речі: ID ROS вузол, а також посилання на швидкість вашого вимірювання.
2. Заповніть [`LocalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1_1_LocalPositionMeasurement.html) `struct` своїми вимірюваннями.
3. Передайте `struct` на `LocalPositionMeasurementInterface` [`update()`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1LocalPositionMeasureInterface.html#a6fd180b944710716d418b2cfe1c0c8e3) метод.

Доступні рамки посилання на положення та швидкість для ваших вимірювань визначаються наступним переліком:

```cpp
enum class PoseFrame
{
  Unknown,
  LocalNED,
  LocalFRD
};

enum class VelocityFrame
{
  Unknown,
  LocalNED,
  LocalFRD,
  BodyFRD
};
```

Структура `LocalPositionMeasment` визначено таким чином:

```cpp
struct LocalPositionMeasurement
{
   rclcpp::Time timestamp_sample {};

   std::optional<Eigen::Vector2f> position_xy {std::nullopt};
   std::optional<Eigen::Vector2f> position_xy_variance {std::nullopt};
   std::optional<float> position_z {std::nullopt};
   std::optional<float> position_z_variance {std::nullopt};

   std::optional<Eigen::Vector2f> velocity_xy {std::nullopt};
   std::optional<Eigen::Vector2f> velocity_xy_variance {std::nullopt};
   std::optional<float> velocity_z {std::nullopt};
   std::optional<float> velocity_z_variance {std::nullopt};

   std::optional<Eigen::Quaternionf> attitude_quaternion {std::nullopt};
   std::optional<Eigen::Vector3f> attitude_variance {std::nullopt};
};
```

Метод `update()` глобального інтерфейсу очікує дотримання наступних умов для `GlobalPositionMeasment`:

- Час відбору вибіркових вимірювань визначений.
- Значення не мають NAN\`\`.
- Якщо надано значення вимірювання, його відповідне значення розбіжності добре визначено (наприклад, якщо `lat_lon` визначено, то необхідно вказати `horizontal_variance`).
- Якщо надано значення вимірювання, його пов'язана рамка посилання не є невідомою (наприклад, якщо визначено `position_xy`, то інтерфейс було ініціалізовано з подовжувачем кадру, відмінного від `PoseFrame:Unknown`).

Наступний фрагмент коду є прикладом вузла ROS 2, який використовує локальний інтерфейс навігації для надсилання оновлень 3D-позиції у рамці посилання Північ-Схід-Дон (NED) до PX4:

```cpp
class MyLocalMeasurementUpdateNode : public rclcpp::Node
{
public:
   MyLocalMeasurementUpdateNode()
   : Node("my_node_name")
   {
      // Set pose measurement reference frame to north-east-down
      const px4_ros2::PoseFrame pose_frame = px4_ros2::PoseFrame::LocalNED;
      // We will only send pose measurements in this example
      // Set velocity measurement reference frame to unknown
      const px4_ros2::VelocityFrame velocity_frame = px4_ros2::VelocityFrame::Unknown;
      // Initialize local interface [1]
      _local_position_measurement_interface =
         std::make_shared<px4_ros2::LocalPositionMeasurementInterface>(*this, pose_frame, velocity_frame);
   }

   void sendUpdate()
   {
      while (running) { // Potentially make method run as a callback or on a timer
         // Generate local position measurement
         rclcpp::Time timestamp_sample  = ...
         Eigen::Vector2f position_xy = ...
         Eigen::Vector2f position_xy_variance = ...
         float position_z = ...
         float position_z_variance = ...

         // Populate the local position measurement struct [2]
         px4_ros2::LocalPositionMeasurement local_position_measurement{};
         local_position_measurement.timestamp_sample = timestamp_sample;
         local_position_measurement.position_xy = position_xy;
         local_position_measurement.position_xy_variance = position_xy_variance;
         local_position_measurement.position_z = position_z;
         local_position_measurement.position_z_variance = position_z_variance;

         // Send measurement to PX4 using the interface [3]
         try {
            _local_position_measurement_interface->update(local_position_measurement);
         } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
            // Handle exceptions caused by invalid local_position_measurement definition
            RCLCPP_ERROR(get_logger(), "Exception caught: %s", e.what());
         }
      }
   }

private:
   std::shared_ptr<px4_ros2::LocalPositionMeasurementInterface> _local_position_measurement_interface;
};
```

###

Спочатку переконайтеся, що параметр PX4 [`EKF2_EV_CTRL`](../advanced_config/parameter_reference.md#EKF2_EV_CTRL) налаштований належним чином для ефективного використання зовнішніх локальних вимірів, встановивши відповідні біти в `true`:

- 0: Дані горизонтальної позиції
- 1: Дані вертикальної позиції

Щоб надіслати глобальне вимірювання на PX4:

1. Створіть [`LocalPositionMeasurementInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4ros2_1_1__ros2_1_1LocalPositionMeasurementInterface.html), надаючи йому речі: ID ROS вузол, а також посилання на швидкість вашого вимірювання.
2. Заповніть [`LocalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1_1_LocalPositionMeasurement.html) `struct` своїми вимірюваннями.
3. Передайте `struct` на `LocalPositionMeasurementInterface` [`update()`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1LocalPositionMeasureInterface.html#a6fd180b944710716d418b2cfe1c0c8e3) метод.

Структуру `LocalPositionMeasment` визначено таким чином:

```cpp
struct GlobalPositionMeasurement
{
   rclcpp::Time timestamp_sample {};

   std::optional<Eigen::Vector2d> lat_lon {std::nullopt};
   std::optional<float> horizontal_variance {std::nullopt};

   std::optional<float> altitude_msl {std::nullopt};
   std::optional<float> vertical_variance {std::nullopt};
};
```

Метод `update()` глобального інтерфейсу очікує дотримання наступних умов для `GlobalPositionMeasment`:

-
- Значення не мають NAN.
- Якщо надано значення вимірювання, його відповідне значення розбіжності добре визначено (наприклад, якщо `lat_lon` визначено, то необхідно вказати `horizontal_variance`).

Наступний фрагмент коду є прикладом вузла ROS 2, який використовує локальний інтерфейс навігації для надсилання оновлень 3D-позиції у рамці посилання Північ-Схід-Дон (NED) до PX4:

```cpp
class MyGlobalMeasurementUpdateNode : public rclcpp::Node
{
public:
   MyGlobalMeasurementUpdateNode()
   : Node("my_node_name")
   {
      // Initialize global interface [1]
      _global_position_measurement_interface =
         std::make_shared<px4_ros2::GlobalPositionMeasurementInterface>(*this);
   }

   void sendUpdate()
   {
      while (running) { // Potentially make method run as a callback or on a timer
         // Generate global position measurement
         rclcpp::Time timestamp_sample  = ...
         Eigen::Vector2d lat_lon = ...
         float horizontal_variance = ...
         float altitude_msl = ...
         float vertical_variance = ...

         // Populate the global position measurement struct [2]
         px4_ros2::GlobalPositionMeasurement global_position_measurement{};
         global_position_measurement.timestamp_sample = timestamp_sample;
         global_position_measurement.lat_lon = lat_lon;
         global_position_measurement.horizontal_variance = horizontal_variance;
         global_position_measurement.altitude_msl = altitude_msl;
         global_position_measurement.vertical_variance = vertical_variance;

         // Send measurement to PX4 using the interface [3]
         try {
            _global_position_measurement_interface->update(global_position_measurement);
         } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
            // Handle exceptions caused by invalid global_position_measurement definition
            RCLCPP_ERROR(get_logger(), "Exception caught: %s", e.what());
         }
      }
   }

private:
   std::shared_ptr<px4_ros2::GlobalPositionMeasurementInterface> _global_position_measurement_interface;
};
```

## Кілька екземплярів інтерфейсу

Використання кількох екземплярів одного інтерфейсу (напр. локально та локально) для надсилання повідомлень про розрахунок буде передавати всі оновлення до однієї теми і що призведе до перехресної розмови.
Це не повинно впливати на об'єднання вимірювань в EKF, але різні джерела вимірювань стануть нерозрізненними.
