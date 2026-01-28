# Спільнота

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">Ця сторінка може бути застарілою. <a href="https://docs.px4.io/main/en/contribute/">Переглянути останню версію</a>.</p>
  </div>
</div>

Ласкаво просимо до Спільноти PX4!

:::tip
We pledge to adhere to the [PX4 code of conduct](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md), which aims to foster an open and welcoming environment.
:::

Цей розділ містить інформацію про те, як ви можете зустрітися зі спільнотою та зробити внесок до PX4:

- [Dev Call](../contribute/dev_call.md) - Discuss architecture, pull requests, impacting issues with the dev team
- [Maintainers](./maintainers.md) - Maintainers of PX4 Subsystems and Ecosystem
- [Support](../contribute/support.md) - Get help and raise issues
- [Source Code Management](../contribute/code.md) - Work with PX4 code
- [Documentation](../contribute/docs.md) - Improve the docs
- [Translation](../contribute/translation.md) - Translate using Crowdin
- [Terminology/Notation](../contribute/notation.md) - Terms and symbols used in the docs
- [Licenses](../contribute/licenses.md) - PX4 and Pixhawk licensing
