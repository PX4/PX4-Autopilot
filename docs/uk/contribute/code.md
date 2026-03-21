# Керування вихідним кодом

##

Проект PX4 використовує модель розгалуження Git із трьома гілками:

- [main](https://github.com/PX4/PX4-Autopilot/tree/main) is by default unstable and sees rapid development.
- [beta](https://github.com/PX4/PX4-Autopilot/tree/beta) has been thoroughly tested. Він призначений для тестерів на польоти.
- [stable](https://github.com/PX4/PX4-Autopilot/tree/stable) points to the last release.

We try to retain a [linear history through rebases](https://www.atlassian.com/git/tutorials/rewriting-history) and avoid the [Github flow](https://docs.github.com/en/get-started/using-github/github-flow).
Однак через глобальну команду і швидкий розвиток ми можемо одночасно вдаватися до збоїв.

To contribute new functionality, [sign up for Github](https://docs.github.com/en/get-started/using-github/github-flow), then [fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) the repository, [create a new branch](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-and-deleting-branches-within-your-repository), add your [changes as commits](#commits-and-commit-messages), and finally [send a pull request](#pull-requests).
Changes will be merged when they pass our [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration) tests.

All code contributions have to be under the permissive [BSD 3-clause license](https://opensource.org/license/BSD-3-Clause) and all code must not impose any further constraints on the use.

## Стиль Коду

PX4 uses the [Google C++ style guide](https://google.github.io/styleguide/cppguide.html), with the following (minimal) modifications:

::: info

Not all PX4 source code matches the style guide, but any _new code_ that you write should do so — in both new and existing files.
Якщо ви оновлюєте існуючий файл, від вас не вимагається, щоб весь файл відповідав інструкції зі стилю, а лише код, який ви змінили.

:::

### Вкладки

- Вкладки використовуються для відступу (еквівалентно 8 пробілу).
- Пробіли використовуються для вирівнювання.

### Довжина рядка

- Maximum line length is 140 characters.

### Розширення файлів

- Source files use extension `*.cpp` instead of `*.cc`.

### Іменування  функцій та методів

- `lowerCamelCase()` is used for functions and methods to _visually_ distinguish them from `ClassConstructors()` and `ClassNames`.

### Імена змінних членів приватного класу

- `_underscore_prefixed_snake_case` is used for private class member variable names, as oppose to `underscore_postfixed_`.

### Ключові слова класу

- _zero_ spaces before `public:`, `private:`, or `protected:` keywords.

### Приклад сніпета коду

```cpp
class MyClass {
public:

        /**
         * @brief Description of what this function does.
         *
         * @param[in] input_param Clear description of the input [units]
         * @return Whatever we are returning [units]
         */
        float doSomething(const float input_param) const {
                const float in_scope_variable = input_param + kConstantFloat;
                return in_scope_variable * _private_member_variable;
        }

        void setPrivateMember(const float private_member_variable) { _private_member_variable = private_member_variable; }

        /**
         * @return Whatever we are "getting" [units]
         */
        float getPrivateMember() const { return _private_member_variable; }

private:

        // Clear description of the constant if not completely obvious from the name [units]
        static constexpr float kConstantFloat = ...;

        // Clear description of the variable if not completely obvious from the name [units]
        float _private_member_variable{...};
};
```

## Вбудована документація

PX4 розробників заохочують до створення відповідної документації через джерело.

::: info

Стандарти документації вихідного коду не застосовуються, і код в даний час непослідовно документований.
Ми б хотіли зробити краще!

:::

В даний час у нас є два типи базової документації:

- `PRINT_MODULE_*` methods are used for both module run time usage instructions and for the [Modules & Commands Reference](../modules/modules_main.md) in this guide.
  - The API is documented [in the source code here](https://github.com/PX4/PX4-Autopilot/blob/v1.8.0/src/platforms/px4_module.h#L381).
  - Good examples of usage include the [Application/Module Template](../modules/module_template.md) and the files linked from the modules reference.
- We encourage other in-source documentation _where it adds value/is not redundant_.

  :::tip

  Developers should name C++ entities (classes, functions, variables etc.) such that their purpose can be inferred - reducing the need for explicit documentation.


:::

  - Не додавати документацію, яку тривіально можна вивести з імен об'єктів С++.
  - ЗАВЖДИ вказуйте одиниці змінних, констант і параметрів введення/повернення там, де вони визначені.
  - ЗАВЖДИ вказуйте одиниці змінних, постійних і параметрів введення/повернення там, де вони визначені.
  - [Doxgyen](https://www.doxygen.nl/) tags should be used if documentation is needed: `@class`, `@file`, `@param`, `@return`, `@brief`, `@var`, `@see`, `@note`.
    A good example of usage is [src/modules/events/send_event.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/events/send_event.h).

Будь ласка, уникайте "магічних чисел", наприклад, звідки прийшло це число? А як щодо множника при введенні ручки повороту?

```cpp
if (fabsf(yaw_stick_normalized_input) < 0.1f) {
        yaw_rate_setpoint = 0.0f;
}
else {
        yaw_rate_setpoint = 0.52f * yaw_stick_normalized_input;
}
```

Натомість визначте цифри як константи з відповідним контекстом у заголовку:

```cpp
// Порогове значення мертвої зони для нормалізованого введення повороту
static constexpr float kYawStickDeadzone = 0.1f;

// [рад/с] Порогове значення мертвої зони для нормалізованого введення повороту
static constexpr float kMaxYawRate = math::radians(30.0f);
```

і оновіть реалізацію вихідного коду.

```cpp
if (fabsf(yaw_stick_normalized_input) < kYawStickDeadzone) {
        yaw_rate_setpoint = 0.0f;
}
else {
        yaw_rate_setpoint = kMaxYawRate * yaw_stick_normalized_input;
}
```

## Коміти та повідомлення комітів

PX4 uses [conventional commits](https://www.conventionalcommits.org/) for all commit messages and PR titles.

### Format

```
type(scope): short description of the change
```

Where **type** is the category of change (`feat`, `fix`, `docs`, `refactor`, `perf`, `test`, `build`, `ci`, `style`, `chore`, `revert`) and **scope** is the module or area affected (e.g. `ekf2`, `mavlink`, `navigator`). See the full [types and scopes tables](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention) in CONTRIBUTING.md.

Append `!` before the colon to mark a breaking change: `feat(ekf2)!: remove deprecated API`.

### Приклади

```
feat(ekf2): add height fusion timeout. Fixes #1234

The previous implementation did not handle the case where
height fusion data stops arriving mid-flight. This adds a
configurable timeout that falls back to barometric height.

Tested in SITL with simulated sensor dropout.

Signed-off-by: Your Name <your@email.com>
```

The body of the message can contain several paragraphs. Describe in detail what you changed and why. Link related issues and flight logs. Describe the change and why you made it, rather than paraphrasing the code change.

**Use `git commit -s` to sign off on all of your commits.** This adds `Signed-off-by:` with your name and email as the last line.

## Запити на злиття

Github [Pull Requests (PRs)](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests) are the primary mechanism used to submit new functionality and bug fixes to PX4.

They include the new set of [commits](#commits-and-commit-messages) in your branch (relative the main branch), and a description of the changes.

Опис повинен включати:

- Огляд того, що доставляють  дані зміни; достатньо для розуміння широкої мети коду
- Посилання на пов'язані з питаннями або підтримка інформації.
- Інформація про те як було виконано тестування PR функціональності з посиланнями на журнали польотів.
- Where possible, the results from general [Test Flights](../test_and_ci/test_flights.md) both before and after the change.
