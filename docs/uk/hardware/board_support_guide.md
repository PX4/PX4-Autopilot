# Керівництво з підтримки плати PX4 від виробника

The PX4 development and test teams fully support and maintain boards that are compliant with the [Pixhawk Standard](https://pixhawk.org/standards/).
Виробники, які бажають відхилитися від стандарту або створити зовсім нові плати, можуть це зробити, але їм буде потрібно підтримувати будь-які виникаючі різниці в сумісності.

This guide outlines the [general requirements](#general_requirements) for board support, along with the additional requirements for the different [board support categories](#board-support-categories).

:::info
Boards that are not compliant with the requirements are [unsupported](#unsupported); they will not be listed on the PX4 website hardware list and will be removed from the codebase.
:::

<a id="general_requirements"></a>

## Загальні вимоги

Загальні вимоги до всіх підтримуваних плат є наступні:

1. Апаратне забезпечення повинно бути доступним на ринку.

2. Плати не повинні мати апаратні помилки або неприйнятну якість, що робить неможливим або небезпечним використання плати з PX4 на БЛА.
   Плата повинна пройти критерії прийняття для забезпечення якості деталей та збірки.

3. Чіткий та простий спосіб зв'язатися з службою підтримки клієнтів для клієнтів.
   Один або декілька з наступних приймається:

   - Присутність сервера PX4 у Discord
   - Електронна пошта підтримки
   - Номер телефону

4. Точка контакту (PoC) для зберігачів PX4 (прямий електронний лист або доступний у Slack/Форумі/Github)

5. The board needs to use the [PX4 bootloader protocol](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/nuttx/src/bootloader).
   For more information on bootloaders see: [PX4 Nuttx Porting Guide > Bootloader](../hardware/porting_guide_nuttx.md#bootloader).

6. Достатня документація, яка включає, але не обмежується:

   - Повний підключення, яке стало доступним для громадськості, яке відображає PX4 визначення контактів на:
      1. Піни мікроконтролера
      2. Фізичні зовнішні роз'ємники
   - A block diagram or full schematic of the main components (sensors, power supply, etc.) that allows to infer software requirements and boot order
   - Посібник з використання готового продукту

7. Має бути окрема веб-сторінка для плати з PX4, на якій перераховані функції та обмеження використання з PX4, включаючи або посилання на вищезазначену документацію.

## Категорії підтримки борди

Категорії підтримки плат перераховані нижче. The autopilot boards in each category are listed at: [https://px4.io/autopilots/.](https://px4.io/autopilots/)

:::info
Manufacturer supported boards may be as well/better supported than Pixhawk boards (for example through economies of scale).
:::

## Pixhawk Connector Standard

Плата Pixhawk - це та, яка відповідає стандартам Pixhawk. These standards are laid out on [http://pixhawk.org](http://pixhawk.org/), but at high-level require that the board passes electrical tests mandated by the standard and the manufacturer has signed the Pixhawk adopter and trademark agreement.

PX4 загалом підтримує лише плати, які є комерційно доступними, що зазвичай означає, що стандарти плат, випущені за останні п'ять років, підтримуються.

<a id="ver_rev_id"></a>

### Ідентифікатори VER та REV (Апаратна ревізія та виявлення версії)

У FMUv5 та пізніше є електричний механізм виявлення.
Цей сенсорний зв'язок разом з необов'язковими даними конфігурації буде використовуватися для визначення конфігурації апаратного забезпечення щодо обов'язкової конфігурації пристрою та живлення. Manufacturers must obtain the VER and REV ID from PX4 board maintainers by issuing a PR to ammend the [DS-018 Pixhawk standard](https://github.com/pixhawk/Pixhawk-Standards) for board versions and revisions.

Оскільки ці борди на 100% відповідають стандарту Pixhawk, значення, призначені для VER та REV ID, є значеннями за замовчуванням для цієї версії FMU.

## Manufacturer-Supported Autopilots

Ці плати підтримуються виробником.
Щоб відповідати цій категорії, борді необхідно працювати з останнім стабільним випуском PX4 протягом 4 місяців з моменту випуску.

- Виробник володіє підтримкою
- Виробник повинен постачати принаймні 2 плати команді ядра-розробників (для використання на тестовому стелажі та командою тестування)

:::tip
While there is no commitment from the PX4 maintainers and the flight test team to support and test boards in this category, we strongly recommended PX4 and manufacturer teams build close working relationships.
Це призведе до кращого результату для всіх сторін.
:::

:::info
These boards will be assigned [VER and REV ID](#ver_rev_id) based on compatibility.
Спеціальне завдання буде створено PX4, якщо плата є варіантом специфікації FMU і здатна запускати той самий бінарний файл, з незначними відмінностями, підтримуваними виробником.
Contact the PX4 maintainer at [boards@px4.io](mailto:boards@px4.io) to request more information.
:::

## Експериментальні налаштування

These boards are all boards that don't fall in the above categories, or don't fall in those categories _anymore_.
Застосовуються наступні вимоги:

- Плата повинна працювати щонайменше з одним випуском PX4 для визначеного типу транспортного засобу, але не обов'язково з останнім випуском.

:::info
Experimental boards that were _previously_ Pixhawk or Manufacturer supported will have/retain their original IDs.
_New_ experimental boards are allocated [VER and REV IDs](#ver_rev_id) based on compatibility, in the same way as Manufacturer Supported boards.
:::

## Непідтримувано

Ця категорія включає всі плати, які не підтримуються проектом PX4 або виробником, і що виходять за межі "експериментальної" підтримки.

- Плата на папері в певній мірі сумісна з чимось, що ми вже підтримуємо, і для того, щоб підняти її до рівня "експериментального", потрібно буде мінімальних зусиль, проте або команда розробників, або виробник наразі цим не займаються
- Manufacturer/Owner of hardware violates our [Code of Conduct](https://discuss.px4.io/t/code-of-conduct/13655)
- Закритий вихідний код, де будь-які необхідні інструменти/бібліотеки/драйвери тощо, необхідні для підтримки плати, вважаються несумісними через ліцензійні обмеження
- Плата не відповідає мінімальним вимогам, визначеним у Загальних вимогах

:::info
Unsupported boards will NOT be assigned [VER and REV ID](#ver_rev_id) (and cannot run PX4 FMUvX firmware).
:::

## Процес випуску

Припускається, що коли виробник заявляє, що плата відноситься до певної категорії, ця дошка відповідає вимогам для цієї категорії та загальним вимогам.

Коли на ринок виводиться нова плата, яка входить до категорії, підтримуваної виробником або експериментальної, виробник відповідає за оновлення документації PX4 та виконання процесу випуску плати в PX4. Ми рекомендуємо виконати наступні кроки:

Contact PX4 board maintainers at [boards@px4.io](mailto:boards@px4.io) and request the following:

1. The assignment of a _board id_ for bootloader and firmware selection in QGC.
2. Призначення значень резисторів REV та VER ID.
3. Якщо плата підтримує USB: або запросіть призначення USB VID та PID, або надайте USB VID та PID.

Integrate the board according to the board porting release process described in the [porting guide](../hardware/porting_guide.md)

:::warning
The board support process may be changed and improved over time.
Виробники апаратного забезпечення закликаються брати участь у цьому процесі через регулярний виклик апаратного забезпечення, форум обговорень або Discord.
:::

## Підтримка

Якщо частини керівного посібника/процесу не є зрозумілими:

- Ask the community for help on Discord channels under `Hardware` category, or on the discuss forum
- Беріть участь у щотижневому виклику з апаратного забезпечення
- Consultancy options are listed here: [https://px4.io/community/consultants/](https://px4.io/community/consultants/)
