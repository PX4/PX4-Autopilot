# Симуляція аварійного стану машини

<Badge type="tip" text="PX4 v1.14" />

Цю сторінку можна використовувати для моделювання дій машини стану PX4 аварійного режиму за всіх можливих конфігурацій та умов.

Симуляція запускає той самий код у браузері, який виконується на транспортному засобі в реальному часі (симуляція автоматично синхронізується з останньою версією коду).
Note that any delayed action (`COM_FAIL_ACT_T`) will also be delayed in the simulation.

Щоб використовувати це:

1. Спочатку налаштуйте параметри зліва.
   Початкові значення відповідають значенням за замовчуванням PX4.
2. Встановіть тип транспортного засобу
3. Set the other values in the **State** or any of the flags under **Conditions**
   - The **Intended Mode** corresponds to the commanded mode via RC or GCS (or external script).
      Станова машина аварійного відновлення може перевизначити це у разі аварійного відновлення.
4. Check the action under **Output**
5. Check what happens when changing mode or **Move the RC sticks**
6. Грайте з різними налаштуваннями та умовами!

Симуляцію також можна виконати локально для тестування конкретної версії або набору змін:

```sh
make run_failsafe_web_server
```

<iframe :src="withBase('/config/failsafe/index.html')" frameborder="0" height="1400px" style="text-align: center; margin-left: -20px; margin-right: -230px;" width="1200"></iframe>

<script setup>
import { withBase } from 'vitepress';
</script>

