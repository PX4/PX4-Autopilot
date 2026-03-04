# Модулі Vertiq All-In-One Motor/ESC

Vertiq виготовляє високоефективні пропульсивні системи для комерційних та оборонних БПЛА.
Основний дизайн складається з легкого, тісно інтегрованого двигуна та ESC з вбудованим позиційним датчиком.
З контролем швидкості з закритим зворотним зв'язком для найшвидшого часу реакції, провідною ефективністю, відсутністю стартових та зворотних вібрацій, що дозволяє керування низькою швидкістю та плавну зворотність, а також вбудованим контролером "стоу" для плавного розміщення пропелерів у режимі очікування у вибраному напрямку, модулі Vertiq мають значні переваги перед іншими ESC.

![Vertiq Module Lineup](../../assets/peripherals/esc_vertiq/vertiq_esc_lineup.jpg)

All Vertiq modules support traditional [PWM input, DShot, OneShot, and Multishot communication protocols](https://iqmotion.readthedocs.io/en/latest/communication_protocols/hobby_protocol.html). Vertiq's larger modules also support [DroneCAN control](https://iqmotion.readthedocs.io/en/latest/communication_protocols/dronecan_protocol.html).

## Де купити

Purchasing information can be found on the [Vertiq website](https://www.vertiq.co/).

## Налаштування програмного забезпечення

### Підключення

Підключення вашого модуля Vertiq до виходу ШІМ з вашого керування польотом або шини DroneCAN буде відрізнятися в залежності від вашої моделі.
Будь ласка, перегляньте аркуші даних продукту для інформації щодо підключення.

All Vertiq datasheets can be found at [vertiq.co](https://www.vertiq.co/).

## Налаштування прошивки

Найкращий інструмент для налаштування вашого модуля Vertiq - це додаток IQ Control Center від Vertiq.
You can find instructions for installation in [Getting Started with Speed Modules Using IQ Control Center](https://iqmotion.readthedocs.io/en/latest/control_center_docs/speed_module_getting_started.html).

To get started with traditional PWM input or DShot with your flight controller, please see [PWM and DSHOT Control with a Flight Controller](https://iqmotion.readthedocs.io/en/latest/tutorials/pwm_control_flight_controller.html).

To get started with DroneCAN with your flight controller, please see [DroneCAN Integration with a PX4 Flight Controller](https://iqmotion.readthedocs.io/en/latest/tutorials/dronecan_flight_controller.html).

## Налаштування польотного контролера

### Конфігурація DroneCAN

Instructions for integrating the motor/ESC using with DroneCAN can be found in [Flight Controller Configuration](https://iqmotion.readthedocs.io/en/latest/tutorials/dronecan_flight_controller.html#dronecan-integration-with-a-px4-flight-controller) (in _DroneCAN Integration with a PX4 Flight Controller_).

Ці інструкції допоможуть вам налаштувати правильні параметри для активації драйверів DroneCAN контролера польоту, встановити правильні конфігураційні параметри для зв'язку з модулями Vertiq на шині DroneCAN, налаштування ESC та перевірку того, що ваш контролер польоту може належним чином керувати вашими модулями по протоколу DroneCAN.

### Конфігурація DShot/PWM

Instructions for integrating the motor/ESC using PWM and DShot can be found in [PWM and DShot Control with a Flight Controller](https://iqmotion.readthedocs.io/en/latest/tutorials/pwm_control_flight_controller.html).
DShot рекомендовано.

## Подальша інформація

- <https://www.vertiq.co/> — Learn more about Vertiq modules
- [Vertiq Documentation](https://iqmotion.readthedocs.io/en/latest/index.html) — Additional information about configuring your Vertiq module
