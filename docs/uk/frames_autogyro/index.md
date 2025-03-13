# Автогиро Каркаси

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for autogyro frames is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!
:::

An [Autogyro](https://en.wikipedia.org/wiki/Autogyro) is a type of [rotary-wing](https://en.wikipedia.org/wiki/Rotorcraft).
Порівняно з іншими конструкціями він має наступні переваги:

- Можливість злітати та сідати, використовуючи лише дуже коротку злітну смугу (порівняно з фіксованим крилом).
- Висока стійкість до погодних умов, особливо поривів вітру.
- Володіння неприводним ротором, що дозволяє йому працювати в режимі авторотації (один із аварійних режимів гелікоптера).
  Отже, йому не потрібно активно змінювати режим польоту у разі відмови (йому не потрібен парашут або інші активно працюючі пристрої).
  Таким чином, політ за своєю суттю є стабільним у будь-який час.
- Absence of a [downwash](https://en.wikipedia.org/wiki/Downwash) during take-off or landing creating an unwanted swirling of dust.
- [Low lift-to-drag ratio](https://en.wikipedia.org/wiki/Lift-to-drag_ratio) that can be adjusted by construction parameters.
  Ця здатність може бути корисною, оскільки безпілотний автогир не може летіти дуже далеко у разі відмови (як у випадку звичайного літака), проте польот все ще безпечний, і літак не падає (як у випадку багтороторного або гелікоптера).

## Підтримувані конструкції

PX4 підтримує кілька автогирних конструкцій.
The set of supported configurations can be seen in [Airframes Reference > Autogyro](../airframes/airframe_reference.md#autogyro).

### DIY Конструкції

У цьому розділі містяться журнали збірки/інструкції щодо складання та налаштування ряду Autogyro конструкцій.

- [ThunderFly Auto-G2 (Holybro pix32)](../frames_autogyro/thunderfly_auto_g2.md)

### Повні конструкції з попередніми налаштуваннями PX4

У цьому розділі перелічені транспортні засоби, які продаються повністю зібраними та готові до польоту (RTF), з встановленим PX4.

- [ThunderFly TF-G2](https://www.thunderfly.cz/tf-g2.html)
