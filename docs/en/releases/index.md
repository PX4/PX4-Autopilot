# Releases

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">This page is on a release branch, and hence probably out of date. <a href="https://docs.px4.io/main/en/releases/">See the latest version</a>.</p>
  </div>
  <Redirect to="https://docs.px4.io/main/en/releases/" />
</div>

A list of PX4 release notes, they contain a list of the changes that went into each release, explaining the included features, bug fixes, deprecations and updates in detail.

- [main](../releases/main.md) (changes planned for v1.19 or later)
- [v1.18](../releases/1.18.md) (changes in v1.18, since v1.17)
- [v1.17](../releases/1.17.md)
- [v1.16](../releases/1.16.md)
- [v1.15](../releases/1.15.md)
- [v1.14](../releases/1.14.md)
- [v1.13](../releases/1.13.md)
- [v1.12](../releases/1.12.md)

The full archive of releases for the PX4 autopilot project can be found on [GitHub](https://github.com/PX4/PX4-Autopilot/releases).

:::info
For maintainers, see [Release Process](../releases/release_process.md) for the tagging and publishing workflow.
:::
