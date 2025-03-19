# uORB Pub/Sub 그래프

모듈들간의 통신을 보여주는 uORB Pub/Sub 그래프를 제공합니다.
It is based on information that is extracted directly from the source code.
Usage instructions are provided [below](#graph-properties).

<iframe :src="withBase('/middleware/index.html')" frameborder="0" width="1300" height="1450px" style="text-align: center; margin-left: 0px; margin-right: 0px;"></iframe>

<script setup>
import { withBase } from 'vitepress';
</script>

## 그래프 속성

그래프는 다음과 같은 특성을 갖고 있습니다.

- 모듈은 모서리가 둥근 회색으로 표시되는 반면 주제는 색상이 지정된 직사각형 상자로 표시됩니다.
- 연관있는 모듈과 토픽들은 선으로 연결됩니다.
  Dashed lines indicate that the module publishes the topic, solid lines indicate that the module subscribes to the topic, while dot-dashed lines indicate that the module both publishes and subscribes to the topic.
- 몇개의 모듈과 토픽들은 제외합니다.
  - Topics that are subscribed/published by many modules: `parameter_update`, `mavlink_log` and `log_message`.
  - 기록되는 토픽들의 집합.
  - 퍼블리셔나 섭스크라이버가 없는 토픽들.
  - Modules in **src/examples**.
- 모듈이나 토픽에 마우스 오버시 그것에 연결된 모든 것이 강조됩니다.
- 토픽을 더블클릭해 오픈하면 메시지 정의가 보여집니다.
- 브라우저 창이 전체 그래프를 표시할 수 있을 만큼 충분히 넓은지 확인합니다(사이드바 메뉴는 왼쪽 상단 모서리에 있는 아이콘으로 숨길 수 있음).
  이미지를 줌으로 확대할 수도 있습니다.
- The _Preset_ selection list allows you to refine the list of modules that are shown.
- The _Search_ box can be used to find particular modules/topics (topics that are not selected by the search are greyed-out).

