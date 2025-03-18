# uORB Publication/Subscription Graph

This page provides a uORB publication/subscription graph that shows the communication between modules.
It is based on information that is extracted directly from the source code.
Usage instructions are provided [below](#graph-properties).

<iframe :src="withBase('/middleware/index.html')" frameborder="0" width="1300" height="1450px" style="text-align: center; margin-left: 0px; margin-right: 0px;"></iframe>

<script setup>
import { withBase } from 'vitepress';
</script>


## Graph Properties

The graph has the following properties:

- Modules are shown in gray with rounded corners while topics are displayed as coloured rectangular boxes.
- Associated modules and topics are connected by lines.
  Dashed lines indicate that the module publishes the topic, solid lines indicate that the module subscribes to the topic, while dot-dashed lines indicate that the module both publishes and subscribes to the topic.
- Some modules and topics are excluded:
  - Topics that are subscribed/published by many modules: `parameter_update`, `mavlink_log` and `log_message`.
  - The set of logged topics.
  - Topics that have no subscriber or no publisher.
  - Modules in **src/examples**.
- Hovering over a module/topic highlights all its connections.
- Double-clicking on a topic opens its message definition.
- Make sure your browser window is wide enough to display the full graph (the sidebar menu can be hidden with the icon in the top-left corner).
  You can also zoom the image.
- The *Preset* selection list allows you to refine the list of modules that are shown.
- The *Search* box can be used to find particular modules/topics (topics that are not selected by the search are greyed-out).

