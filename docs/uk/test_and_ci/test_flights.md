# Польотні тести

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">Ця сторінка може бути застарілою. <a href="https://docs.px4.io/main/en/test_and_ci/test_flights.html">Переглянути останню версію</a>.</p>
  </div>
</div>

Тестові польоти є важливим етапом для забезпечення якості.

When submitting [Pull Requests](../contribute/code.md#pull-requests) for new functionality or bug fixes you should provide information about the feature-relative tests performed, along with accompanying flight logs.

Для значних змін у системі ви також повинні виконати загальні польотні тести за допомогою тестових карток, перерахованих нижче.

## Тестові картки

Ці тестові картки визначають "стандартні" польотні тести.
Їх виконує тестова команда в рамках тестування випуску та для більш значних змін у системі.

- [MC_01 - Manual modes](../test_cards/mc_01_manual_modes.md)
- [MC_02 - Full Autonomous](../test_cards/mc_02_full_autonomous.md)
- [MC_03 - Auto Manual Mix](../test_cards/mc_03_auto_manual_mix.md)
- [MC_04 - Failsafe Testing](../test_cards/mc_04_failsafe_testing.md)
- [MC_05 - Indoor Flight (Manual Modes)](../test_cards/mc_05_indoor_flight_manual_modes.md)
