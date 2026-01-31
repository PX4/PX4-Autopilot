# Community

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">This page may be out out of date. <a href="https://docs.px4.io/main/en/contribute/">See the latest version</a>.</p>
  </div>
</div>

Welcome to the PX4 Community!

:::tip
We pledge to adhere to the [PX4 code of conduct](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md), which aims to foster an open and welcoming environment.
:::

This section contains information about how you can meet with the community and contribute to PX4:

- [Dev Call](../contribute/dev_call.md) - Discuss architecture, pull requests, impacting issues with the dev team
- [Maintainers](./maintainers.md) - Maintainers of PX4 Subsystems and Ecosystem
- [Support](../contribute/support.md) - Get help and raise issues
- [Source Code Management](../contribute/code.md) - Work with PX4 code
- [Documentation](../contribute/docs.md) - Improve the docs
- [Translation](../contribute/translation.md) - Translate using Crowdin
- [Terminology/Notation](../contribute/notation.md) - Terms and symbols used in the docs
- [Licenses](../contribute/licenses.md) - PX4 and Pixhawk licensing
