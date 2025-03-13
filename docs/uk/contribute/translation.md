# Переклади

We'd love your help to translate _QGroundControl_, PX4 Metadata (in QGC), and our guides for PX4, _QGroundControl_ and MAVLink!

Our docs (and _QGroundControl_) use the [Crowdin](https://crowdin.com) online tool for translation.
Crowdin автоматично імпортує вихідні теми з Github і представляє нові і змінені рядки для перекладу та/або перегляду (фінального затвердження).

Crowdin експортує перекладені документи назад на Github як "Pull Request" (яку команда розробників періодично переглядає та приймає).
Експортований вивід містить вихідний документ з будь-яким перекладом і затвердженим текстом замінено перекладеними рядками (i. . Якщо рядок не перекладений/змінений, то він буде відображатися англійською).

:::tip
You will need a (free) [Crowdin account](https://crowdin.com/join) account to join the translation team!
:::

:::info
The benefit of this system is that the translation closely tracks the source documents.
Читачі не будуть введені в оману через застарілі і неоновлені переклади.
:::

## Початок роботи

Кроки з приєднання до нашої команди з перекладу такі:

1. Join Crowdin: [https://crowdin.com/join](https://crowdin.com/join)

2. Відкрийте проект, до якого хочете приєднатися:
   - [QGroundControl](https://crowdin.com/project/qgroundcontrol) — QGroundControl UI and hard coded strings.
   - [PX4-Metadata-Translations](https://crowdin.com/project/px4-metadata-translations) — PX4 parameter and event descriptions in QGroundControl.
   - [PX4 User Guide](https://crowdin.com/project/px4-user-guide)
   - [QGroundControl Developer Guide](https://crowdin.com/project/qgroundcontrol-developer-guide)
   - [QGroundControl User Guide](https://crowdin.com/project/qgroundcontrol-user-guide)
   - [MAVLink Guide](https://crowdin.com/project/mavlink)

3. Виберіть мову, якою ви хочете перекладати

4. Click the **Join** button (next to the text _You must join the translators team to be able to participate in this project_)

   ::: info
   You will be notified once your application to join is accepted.

:::

5. Починайте перекладати!

## Особливі примітки

### Не змінюйте текст префікса!

Vuepress uses `:::` to mark the beginning of notes, tips and warning:

```html
:::tip
The text for the tip.
:::
```

The text for `:::tip` or `:::warning` etc. should not be modified as it defines the colour of the notebox.

## Додавання нової мови

Якщо мова, яку ви хочете перекласти, не доступна тоді вам потрібно буде запросити його звернутися до власника проекту (на головній сторінці проекту є посилання на кожен проект).

:::warning
Підтримка перекладу дуже тяжка!
Перш ніж ви попросите нас створити нову мову, знайдіть кількох інших людей, щоб допомогти вам перекласти!
:::

## Отримання допомоги

The _Crowdin_ interface is self explanatory, but there is plenty of additional information on the [knowledgeable](https://support.crowdin.com/).

You can also ask for help from translators and developers in the Dronecode community using [our support channels](../contribute/support.md).
