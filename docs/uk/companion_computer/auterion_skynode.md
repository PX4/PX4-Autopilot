# Auterion Skynode

Auterion [Skynode](https://auterion.com/product/skynode/) - це потужний літальний комп'ютер, який об'єднує комп'ютер для виконання завдань, контролер польоту, потокове відео, мережеве з'єднання та клітинний зв'язок в одному добре інтегрованому пристрої.

![Auterion Skynode (Enterprise)](../../assets/companion_computer/auterion_skynode/skynode_small.png)

Вбудоване програмне забезпечення - це Auterion OS, що складається з підприємницької версії PX4, яка працює на контролері польоту, і операційної системи з високорівневим програмним забезпеченням управління, що працює на комп'ютері для виконання завдань.
ОС керується Auterion на виробництві, із клієнтськими програмами, що працюють як "додатки" в безпечному пісочниці в місійному комп'ютері.

Auterion OS і Skynode дозволяють безпроблемну інтеграцію з іншими програмами Auterion та продуктами управління флотом.

Для отримання інформації про Auterion і Skynode, звертайтесь за наступним посиланням:

- [auterion.com](https://auterion.com/)
- [Skynode](https://auterion.com/product/skynode/) (auterion.com)
- Посібник Skynode:
  - [Посібник виробника](https://docs.auterion.com/manufacturers/getting-started/readme)
  - [Посібник розробників додатка](https://docs.auterion.com/developers/getting-started/readme)

## Skynode з Vanilla PX4

Skynode з поставкою Auterion управляється версією PX4.
Якщо ви хочете спробувати більш нове ядро польоту PX4, ви можете встановити "ванільний" PX4 з [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot).

Загалом, вихідний PX4 буде працювати, з наступними обмеженнями:

- Конфігурація саме для вашого продукту може бути відсутня.
  Ви можете втратити конфігурацію для ESC, акумуляторів, конфігурації датчиків тощо.
- Деякі параметри можуть мати інші значення за замовчуванням у випуску PX4, що постачається з Auterion OS.
- Функції, до яких можна отримати доступ за допомогою індивідуальних налаштувань постачальника, які працюють на комп’ютері-супутнику, можуть бути відсутні в PX4.
- Auterion підтримує Skynode під керуванням власної версії PX4, керованої Auterion.

## Побудова/Завантаження Прошивки

Бінарні файли PX4 `px4_fmu-v5x` для Skynode будуються з вихідного коду за допомогою звичайного [середовища розробника](../dev_setup/dev_env.md) і [команд для побудови](../dev_setup/building_px4.md), і завантажуються за допомогою цілей завантаження `upload_skynode_usb` або `upload_skynode_wifi`.

`upload_skynode_usb` та `upload_skynode_wifi` підключаються до Skynode за допомогою SSH через мережевий інтерфейс, використовуючи IP-адреси за замовчуванням (фіксовані) для [USB](https://docs.auterion.com/manufacturers/avionics/skynode/advanced-configuration/connecting-to-skynode) і [WiFi](https://docs.auterion.com/manufacturers/avionics/skynode/advanced-configuration/configuration), і завантажують TAR-архівований бінарний файл на місійний комп'ютер.
Потім місійний комп'ютер розпаковує бінарний файл та встановлює його на контролер польоту.

:::info
Для використання цих цілей завантаження потрібні SSH і TAR, але їх очікується наявність за замовчуванням на Ubuntu та Ubuntu, що працює на Windows у WSL2.
На macOS ви спочатку повинні встановити [gnu-tar](https://formulae.brew.sh/formula/gnu-tar).
:::

Під час процесу завантаження вам доведеться ввести пароль для зображення розробника Skynode двічі.

:::: tabs

:::tab "Skynode підключений через USB"

```
make px4_fmu-v5x upload_skynode_usb
```

:::

:::tab "Skynode підключений через WiFi"

```
make px4_fmu-v5x upload_skynode_wifi
```

:::

::::

## Відновлення прошивки PX4 за промовчанням

Щоб перевстановити оригінальну версію Skynode PX4 при підключенні через USB, виконайте таку команду в репозиторії:

:::: tabs

:::tab "Skynode підключений через USB"

```
./Tools/auterion/upload_skynode.sh --revert
```

:::

:::tab "Skynode підключений через WiFi"

```
./Tools/auterion/upload_skynode.sh --revert --wifi
```

:::

::::
