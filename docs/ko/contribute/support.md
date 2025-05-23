# 지원

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">This page may be out out of date. <a href="https://docs.px4.io/main/en/contribute/support.html">See the latest version</a>.</p>
  </div>
</div>

이 절에서는 핵심 개발팀과 커뮤니티의 지원을 받는 방법을 알려드립니다.

## 포럼 및 채팅

The core development team and community are active on the following channels:

- [PX4 Discuss Forum](https://discuss.px4.io/) - Post here first!
- [PX4 Discord](https://discord.gg/dronecode) - Post here if you don't get a response in discuss within a few days (include a link to your forum topic).

:::tip
The Discuss Forum is much preferred because it is indexed by search engines and serves as a common knowledge base.
:::

## 문제 진단

문제의 원인이 무엇인지 확실하지 않고 문제 분석의 도움을 원하시면,

- Upload logs to [Flight Log Review](https://logs.px4.io/)
- Open a discussion on [PX4 Discuss](https://discuss.px4.io/c/flight-testing/) with a flight report and links to logs.
- The dev team may prompt you to [raise an issue](#issue-bug-reporting) if the problem is caused by a bug.

## Issue & Bug Reporting

- Upload logs to [Flight Log Review](https://logs.px4.io/)
- [Open a Github Issue](https://github.com/PX4/PX4-Autopilot/issues).
  This must include a flight report with as much detail as possible (enough for the issue to be reproduced) and links to your logs on Flight review.

## 주간 온라인 개발자 회의

:::tip
Developers are most welcome to attend the [weekly dev call](../contribute/dev_call.md) (and other [developer events](../index.md#calendar-events)) to engage more deeply with the project.
:::

The [Dev Call](../contribute/dev_call.md) is a weekly meeting attended by the PX4 dev team to discuss platform technical details, coordinate activities and perform in-depth analysis.

There is also space in the agenda to discuss pull requests, major impacting issues and Q&A.
