# Інструментарій на віртуальних машинах Windows

:::warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env.md).
Це може працювати або не працювати з поточними версіями PX4.

Дивіться [Встановлення інструментарію](../dev_setup/dev_env.md) для інформації про середовища та інструменти, що підтримуються основною командою розробників.
:::

Розробники на Windows можуть використовувати інструментарій PX4 у віртуальній машині (VM) з гостьовою операційною системою Linux.
Після створення віртуальної машини, установка та налаштування PX4 у VM така сама, як і на звичайному комп'ютері з Linux.

Хоч використання віртуальної машини - це простий спосіб налаштувати та протестувати середовище для збірки прошивки, користувачі повинні взяти до уваги:

1. Збірка прошивки буде повільніша, ніж нативна збірка на Linux.
2. У симуляції JMAVSim частота кадрів набагато повільніша, ніж на рідному Linux.
   В деяких випадках засіб може розбитися через проблеми, пов'язані з недостатніми ресурсами віртуальної машини.
3. Gazebo та ROS встановлюються, але повільні настільки що ними неможливо користуватись.

:::tip
Allocate as many CPU cores and memory resources to the VM as possible.
:::

Існує кілька способів налаштування VM, яка здатна виконувати PX4 середовище на вашій системі.
Цей посібник допоможе вам налаштувати VMWare.
В кінці є неповний розділ для VirtualBox (і ми запрошуємо до розширення цього розділу когось з членів спільноти).

## Налаштування VMWare

Ефективність VMWare прийнятна для основного застосування (збірки прошивки) але не для запуску ROS чи Gazebo Classic.

1. Download [VMWare Player Freeware](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html)

2. Установіть його на вашу Windows систему

3. Download the desired version of [Ubuntu Desktop ISO Image](https://www.ubuntu.com/download/desktop).
   (see [Linux Instructions Page](../dev_setup/dev_env_linux.md) for recommended Ubuntu version).

4. Open _VMWare Player_.

5. Enable 3D acceleration in the VM's settings: **VM > Settings > Hardware > Display > Accelerate 3D graphics**

   ::: info
   This option is required to properly run 3D simulation environments like jMAVSim and Gazebo Classic.
   Рекомендуємо зробити це перед встановленням Linux у віртуальному середовищі.

:::

6. Перейдіть до створення нової віртуальної машини.

7. У майстрі створення VM оберіть завантажений ISO образ з Ubuntu як носій установки з якого буде автоматично виявлено операційну систему, яку ви хочете використати.

8. Також у майстрі, оберіть ресурси, які ви хочете виділити віртуальній машині під час роботи.
   Виділіть стільки пам'яті та ядер процесора скільки зможете таким чином щоб вашою основною Windows системою можна було продовжити користуватись.

9. Запустіть нову VM в кінці майстра та завершіть встановлення Ubuntu відповідно до інструкцій з установки.
   Запам'ятайте, всі налаштування потрібні тільки для використання у вашій основній операційній системі, тому можна вимкнути будь-який режим збереження екрана та функції безпеки локальні робочої станції, що не збільшують ризик мережевої атаки.

10. Once the new VM is booted up make sure you install _VMWare tools drivers and tools extension_ inside your guest system.
   Це підвищить продуктивність та зручність користування віртуальною машиною:

   - Значно поліпшена продуктивність графіки
   - Належна підтримка використання апаратного забезпечення, наприклад розподілу портів USB (важливо для завантаження прошивок), прокручування коліщатком миші, підтримка звуку
   - Адаптація роздільної здатності дисплею гостя до розміру вікна емулятора
   - Спільний доступ до буфера обміну з основної ОС
   - Спільний доступ до файлів з основної ОС

11. Continue with [PX4 environment setup for Linux](../dev_setup/dev_env_linux.md)

## Налаштування VirtualBox 7

Налаштування VirtualBox схоже на VMWare.
Учасники спільноти, запрошуємо до створення покрокових інструкцій для цього розділу!

### Пропускання USB для QGroundControl / запису прошивки

:::tip
This section has been tested for VirtualBox 7 running Ubuntu 20.04 LTS on a Windows 10 host machine.
:::

One limitation of virtual machines is that you can't automatically connect to a flight controller attached to the host computer USB port in order to [build and upload PX4 firmware from a terminal](../dev_setup/building_px4.md#uploading-firmware-flashing-the-board).
Також, не можна приєднатися до польотного контролера з QGroundControl в віртуальній машині.

Щоб дозволити це, необхідно налаштувати параметри пропуску USB:

1. Переконайтеся, що користувач був доданий в групу dialout в VM використовуючи команду терміналу:

   ```sh
   sudo usermod -a -G dialout $USER
   ```

   Потім перезавантажте Ubuntu у віртуальній машині.

2. Enable serial port(s) in VM: **VirtualBox > Settings > Serial Ports 1/2/3/etc...**

3. Enable USB controller in VM: **VirtualBox > Settings > USB**

4. Add USB filters for the bootloader in VM: **VirtualBox > Settings > USB > Add new USB filter**.

   - Відкрийте меню і під'єднайте USB-кабель, підключений до автопілота.
      Select the `...Bootloader` device when it appears in the UI.

      ::: info
      The bootloader device only appears for a few seconds after connecting USB.
      Якщо він зникає до того як ви змогли обрати його, від'єднайте та повторно під'єднайте USB.

:::

   - Select the `...Autopilot` device when it appears (this happens after the bootloader completes).

5. Select the device in the VM instance's dropdown menu **VirtualBox > Devices > your_device**

If successful, your device will show up with `lsusb` and QGroundControl will connect to the device automatically.
Ви також матимете можливість збирати та завантажувати прошивку за допомогою команди:

```sh
make px4_fmu-v5_default upload
```

### Телеметрія через WiFi для QGroundControl

If using _QGroundControl_ within a virtual machine you should set the VM networking settings to "Bridged Adapter" mode.
Це дає гостьовій ОС прямий доступ до мережевого обладнання на основній машині.
Якщо ви використовуєте режим трансляції мережевих адрес (NAT), який встановлюється за замовчуванням для VirtualBox 7 для Ubuntu 20.04 LTS, це заблокує вихідні UDP-пакети, які використовує QGroundControl для зв'язку з рухомим засобом.
