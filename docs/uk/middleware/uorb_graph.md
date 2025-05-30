# Граф публікації/підписки uORB

Ця сторінка надає графік публікації/підписки uORB, який показує комунікацію між модулями.
Він базується на інформації, яка витягується безпосередньо з вихідного коду.
Usage instructions are provided [below](#graph-properties).

<iframe :src="withBase('/middleware/index.html')" frameborder="0" width="1300" height="1450px" style="text-align: center; margin-left: 0px; margin-right: 0px;"></iframe>

<script setup>
import { withBase } from 'vitepress';
</script>

## Властивості графа

Граф має наступні властивості:

- Модулі відображаються сірим кольором із закругленими кутами, а теми - у вигляді кольорових прямокутних блоків.
- Пов'язані модулі та теми з'єднані лініями.
  Пунктирні лінії вказують на те, що модуль публікує тему, суцільні лінії вказують на те, що модуль підписується на тему, а пунктирно-пунктирні лінії вказують на те, що модуль і публікує, і підписується на тему.
- Деякі модулі та теми виключені:
  - Topics that are subscribed/published by many modules: `parameter_update`, `mavlink_log` and `log_message`.
  - Набір зареєстрованих тем.
  - Теми, які не мають підписника або публікації.
  - Modules in **src/examples**.
- При наведенні на модуль/тему відображаються всі його зв'язки.
- Подвійне клацання на темі відкриває визначення її повідомлення.
- Переконайтеся, що вікно вашого браузера достатньо широке, щоб відобразити весь графік (меню бічної панелі можна приховати за допомогою іконки у верхньому лівому куті).
  Ви також можете збільшити зображення.
- The _Preset_ selection list allows you to refine the list of modules that are shown.
- The _Search_ box can be used to find particular modules/topics (topics that are not selected by the search are greyed-out).

