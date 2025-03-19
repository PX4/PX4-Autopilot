# Параметри та налаштування

PX4 використовує _param subsystem_ (таблицю значень `float` і `int32_t`) і текстові файли (для скриптів запуску) для створення конфігурації.

This section discusses the _param_ subsystem in detail.
У ньому описано, як відображати, зберігати і завантажувати параметри, а також як їх описувати і робити доступними для наземних станцій.

:::tip
[Запуск системи](../concept/system_startup.md) та роботу скриптів запуску [конфігурації фреймів](../dev_airframes/adding_a_new_frame.md) описано на інших сторінках.
:::

## Використання командного рядка

[Системна консоль PX4](../debug/system_console.md) пропонує інструмент [param](../modules/modules_command.md#param), за допомогою якого можна встановлювати параметри, зчитувати їх значення, зберігати їх, а також експортувати й зберігати у файлах та відновлювати з них.

### Отримання та встановлення параметрів

Команда `param show` виводить усі параметри системи:

```sh
param show
```

Для більшої вибірковості можна використовувати часткове ім'я параметра з символом підстановки "\*":

```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

Ви можете використовувати прапорець `-c`, щоб показати всі параметри, які було змінено (порівняно з їх значеннями за замовчуванням):

```sh
param show -c
```

Ви можете використати `param show-for-airframe`, щоб показати всі параметри, які було змінено від значень за замовчуванням лише для поточного файлу літального апарату (та значень за замовчуванням, які він імпортує).

### Параметри експорту та завантаження

Ви можете зберігати будь-які параметри, які були _змінені_ (які відрізняються від параметрів за замовчуванням).

Стандартна команда `param save` збереже параметри у поточному файлі за замовчуванням:

```sh
param save
```

Якщо вказати аргумент, вона збереже параметри у новому каталозі:

```sh
param save /fs/microsd/vtol_param_backup
```

Існує дві різні команди для _завантаження_ параметрів:

- `param load` спочатку виконує повне скидання всіх параметрів до значень за замовчуванням, а потім перезаписує значення параметрів будь-якими значеннями, збереженими у файлі.
- `param import` просто перезаписує значення параметрів значеннями з файлу, а потім зберігає результат (тобто фактично викликає `param save`).

Команда `load` ефективно скидає параметри до стану, у якому вони були збережені (ми говоримо "ефективно", тому що будь-які параметри, збережені у файлі, буде оновлено, але інші параметри можуть мати інші значення за замовчуванням, визначені у прошивці, ніж під час створення файлу параметрів).

By contrast, `import` merges the parameters in the file with the current state of the vehicle.
Наприклад, можна просто імпортувати файл параметрів, що містить дані калібрування, не перезаписуючи решту конфігурації системи.

Приклади для обох випадків показані нижче:

```sh
# Reset the parameters to when file was saved
param load /fs/microsd/vtol_param_backup
# Optionally save params (not done automatically with load)
param save
```

```sh
# Merge the saved parameters with current parameters
param import /fs/microsd/vtol_param_backup
```

## Створення/визначення параметрів

Опис параметрів складається з двох частин:

- [Метадані параметрів](#parameter-metadata) визначають значення за замовчуванням для кожного параметра у прошивці разом з іншими метаданими для відображення (і редагування) параметрів на наземних станціях керування та у документації.
- [Код C/C++](#c-c-api), який надає доступ до отримання та/або зміни значень параметрів з модулів та драйверів PX4.

Нижче описано кілька підходів до написання метаданих та коду.
Where possible code should use newer [YAML metadata](#yaml-metadata) and [C++ API](#c-api) over the older C parameter/code definitions, as these are more flexible and robust.

Метадані параметрів [компілюються у прошивку](#publishing-parameter-metadata-to-a-gcs),
і надаються наземним станціям за посередництвом служби [MAVLink Component Information service](https://mavlink.io/en/services/component_information.html).

### Назви параметрів:

Назви параметрів не повинні перевищувати 16 ASCII символів.

By convention, every parameter in a group should share the same (meaningful) string prefix followed by an underscore, and `MC_` and `FW_` are used for parameters related specifically to Multicopter or Fixed-wing systems.
Ця конвенція не є обов'язковою.

Назва має збігатися як у коді, так і у метаданих [параметру](#parameter-metadata), щоб правильно асоціювати параметр з його метаданими (включно зі значенням за замовчуванням у прошивці).

### C / C++ API

Існують окремі C і C++ API, які можна використовувати для отримання доступу до значень параметрів з модулів і драйверів PX4.

Однією з важливих відмінностей між API є те, що версія на C++ має більш ефективний стандартизований механізм синхронізації зі змінами значень параметрів (наприклад, з GCS).

Синхронізація важлива, оскільки параметр може бути змінений на інше значення в будь-який момент.
Your code should _always_ use the current value from the parameter store.
If getting the latest version is not possible, then a reboot will be required after the parameter is changed (set this requirement using the `@reboot_required` metadata).

Крім того, версія на C++ має кращу типізацію та менші витрати оперативної пам'яті.
Недоліком є те, що ім'я параметра має бути відоме під час компіляції, тоді як C API може приймати динамічно створене ім'я як рядок.

#### C++ API

The C++ API provides macros to declare parameters as _class attributes_.
You add some "boilerplate" code to regularly listen for changes in the [uORB Topic](../middleware/uorb.md) associated with _any_ parameter update.
Потім код фреймворку (невидимо) відстежує повідомлення uORB, які впливають на атрибути ваших параметрів, і підтримує їх синхронізацію.
У решті коду ви можете просто використовувати визначені атрибути параметрів, і вони завжди будуть актуальними!

Насамперед включіть необхідні заголовки до заголовка класу вашого модуля або драйвера:

- **px4_platform_common/module_params.h** для отримання макросу `DEFINE_PARAMETERS`:

 ```cpp
 #include <px4_platform_common/module_params.h>
 ```

- **parameter_update.h** для доступу до повідомлень uORB `parameter_update`:

 ```cpp
 #include <uORB/topics/parameter_update.h>
 ```

- **Subscription.hpp** для uORB C++ API підписки:

 ```cpp
 #include <uORB/Subscription.hpp>
 ```

Derive your class from `ModuleParams`, and use `DEFINE_PARAMETERS` to specify a list of parameters and their associated parameter attributes.
Назви параметрів мають збігатися з визначеннями метаданих параметрів.

```cpp
class MyModule : ..., public ModuleParams
{
public:
	...

private:

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,   /**< example parameter */
		(ParamFloat<px4::params::ATT_BIAS_MAX>) _att_bias_max  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};
```

Оновіть файл cpp за допомогою шаблону, щоб перевірити наявність повідомлення uORB, пов'язаного з оновленням параметрів.

Періодично викликайте `parameters_update();` у коді, щоб перевірити, чи відбулося оновлення:

```cpp
void Module::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}
```

У наведеному вище методі:

- `_param_update_sub.updated()` повідомляє нам, чи є _будь-яке_ оновлення в uORB-повідомленні `param_update` (але не вказує, який саме параметр змінено).
- Якщо було оновлено "деякий" параметр, ми копіюємо оновлення у `parameter_update_s` (`param_update`), щоб очистити очікуване оновлення.
- Then we call `ModuleParams::updateParams()`.
 This "under the hood" updates all parameter attributes listed in our `DEFINE_PARAMETERS` list.

Атрибути параметрів (`_sys_autostart` і `_att_bias_max` у цьому випадку) можна використовувати для відображення параметрів, і вони будуть оновлюватися щоразу, коли значення параметра змінюватиметься.

:::tip
Шаблон [Програми/Модуля](../modules/module_template.md) використовує новий стиль C++ API, але не включає метадані [параметрів](#parameter-metadata).
:::

#### C API

C API можна використовувати як у модулях, так і в драйверах.

Спочатку включіть параметр API:

```C
#include <parameters/param.h>
```

Then retrieve the parameter and assign it to a variable (here `my_param`), as shown below for `PARAM_NAME`.
The variable `my_param` can then be used in your module code.

```C
int32_t my_param = 0;
param_get(param_find("PARAM_NAME"), &my_param);
```

:::info
Якщо у метаданих параметра було оголошено `PARAM_NAME`, то буде встановлене його значення за замовчуванням, і наведений вище виклик для пошуку параметра завжди буде успішним.
:::

`param_find()` is an "expensive" operation, which returns a handle that can be used by `param_get()`.
If you're going to read the parameter multiple times, you may cache the handle and use it in `param_get()` when needed

```cpp
# Get the handle to the parameter
param_t my_param_handle = PARAM_INVALID;
my_param_handle = param_find("PARAM_NAME");

# Query the value of the parameter when needed
int32_t my_param = 0;
param_get(my_param_handle, &my_param);
```

### Метадані параметра

PX4 використовує розгалужену систему метаданих параметрів для управління дружнім до користувача представленням параметрів, а також для встановлення значень за замовчуванням для кожного параметра у прошивці.

:::tip
Правильні метадані мають вирішальне значення для якісного користувацького досвіду на наземній станції.
:::

Parameter metadata can be stored anywhere in the source tree as either **.c** or **.yaml** parameter definitions (the YAML definition is newer, and more flexible).
Зазвичай він зберігається разом з відповідним модулем.

Система збірки витягує метадані (за допомогою `make parameters_metadata`) для створення [довідника параметрів](../advanced_config/parameter_reference.md) та інформації про параметри, [що використовуються наземними станціями](#publishing-parameter-metadata-to-a-gcs).

:::warning
Після додавання файлу параметрів _new_ вам слід викликати `make clean` перед збіркою, щоб згенерувати нові параметри (файли параметрів додаються як частина кроку конфігурації _cmake_, який відбувається для чистих збірок і якщо файл cmake змінено).
:::

#### Метадані YAML

:::info
На момент написання статті визначення параметрів YAML не можна використовувати у _бібліотеках_.
:::

YAML meta data is intended as a full replacement for the **.c** definitions.
Він підтримує ті самі метадані, а також нові можливості, такі як множинні визначення.

- Схема метаданих параметрів YAML знаходиться тут: [validation/module_schema.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml).
- Приклад використання визначень YAML можна знайти у визначенні параметрів MAVLink: [/src/modules/mavlink/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/module.yaml).
- YAML-файл реєструється у системі збірки cmake шляхом додавання

 ```cmake
 MODULE_CONFIG
 	module.yaml
 ```

 до секції `px4_add_module` файлу `CMakeLists.txt` цього модуля.

#### Мета-дані YAML з багатьма екземплярами (шаблонами)

Шаблонні визначення параметрів підтримуються у [YAML визначеннях параметрів](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml) (шаблонний код параметрів не підтримується).

The YAML allows you to define instance numbers in parameter names, descriptions, etc. using `${i}`.
Наприклад, нижче буде згенеровано MY_PARAM_1_RATE, MY_PARAM_2_RATE і т.д.

```yaml
MY_PARAM_${i}_RATE:
  description:
    short: Maximum rate for instance ${i}
```

Наступні визначення YAML містять початковий та кінцевий індекси.

- `num_instances` (за замовчуванням 1): Кількість інстансів, які потрібно згенерувати (>=1)
- `instance_start` (default 0): First instance number. If 0, `${i}` expands to [0, N-1]\`.

For a full example see the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/module.yaml)

#### c параметр метаданих

Застарілий підхід для визначення метаданих параметрів знаходиться у файлі з розширенням **.c** (на момент написання цієї статті це підхід, який найчастіше використовується у дереві коду).

Розділи метаданих параметрів виглядають так, як показано в наступних прикладах:

```cpp
/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.0005
 * @reboot_required true
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```

```cpp
/**
 * Acceleration compensation based on GPS
 * velocity.
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_ACC_COMP, 1);
```

Макрос `PARAM_DEFINE_*` в кінці визначає тип параметра (`PARAM_DEFINE_FLOAT` або `PARAM_DEFINE_INT32`), ім'я параметра (яке має відповідати імені, що використовується у коді) та значення за замовчуванням у прошивці.

Рядки в блоці коментарів є необов'язковими, і в основному використовуються для керування параметрами відображення та редагування на наземній станції.
The purpose of each line is given below (for more detail see [module_schema.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml)).

```cpp
/**
 * <title>
 *
 * <longer description, can be multi-line>
 *
 * @unit <the unit, e.g. m for meters>
 * @min <the minimum sane value. Can be overridden by the user>
 * @max <the maximum sane value. Can be overridden by the user>
 * @decimal <the minimum sane value. Can be overridden by the user>
 * @increment <the "ticks" in which this value will increment in the UI>
 * @reboot_required true <add this if changing the param requires a system restart.>
 * @boolean <add this for integer parameters that represent a boolean value>
 * @group <a title for parameters that form a group>
 */
```

## Публікація метаданих параметрів у GCS

The parameter metadata JSON file is compiled into firmware (or hosted on the Internet), and made available to ground stations via the [MAVLink Component Metadata service](https://mavlink.io/en/services/component_information.html).
Для отримання додаткової інформації див. <a href="../advanced/px4_metadata.md"> Метадані PX4 (трансляція і публікація)</a>.

This process is the same as for [events metadata](../concept/events_interface.md#publishing-event-metadata-to-a-gcs).
For more information see [PX4 Metadata (Translation & Publication)](../advanced/px4_metadata.md)

## Подальша інформація

- [Finding/Updating Parameters](../advanced_config/parameters.md)
- [Довідник параметрів](../advanced_config/parameter_reference.md)
- [Реалізація параметрів](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/include/px4_platform_common/param.h#L129) (information on `.get()`, `.commit()`, and other methods)
