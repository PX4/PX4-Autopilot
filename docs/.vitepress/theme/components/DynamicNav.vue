<template>
  <!-- Desktop nav bar layout -->
  <div v-if="!screen" class="VPDynamicNav bar">
    <template v-for="(item, i) in nav" :key="i">
      <VPNavBarMenuLink v-if="!item.items" :item="item" />
      <VPNavBarMenuGroup v-else :item="item" />
    </template>
  </div>

  <!-- Mobile screen layout -->
  <div v-else class="VPDynamicNav screen">
    <template v-for="(item, i) in nav" :key="i">
      <VPNavScreenMenuLink v-if="!item.items" :item="item" />
      <VPNavScreenMenuGroup v-else :text="item.text" :items="item.items" />
    </template>
  </div>
</template>

<script setup>
import { ref, onMounted } from "vue";
import { inBrowser, useData } from "vitepress";
import VPNavBarMenuLink from "vitepress/dist/client/theme-default/components/VPNavBarMenuLink.vue";
import VPNavBarMenuGroup from "vitepress/dist/client/theme-default/components/VPNavBarMenuGroup.vue";
import VPNavScreenMenuLink from "vitepress/dist/client/theme-default/components/VPNavScreenMenuLink.vue";
import VPNavScreenMenuGroup from "vitepress/dist/client/theme-default/components/VPNavScreenMenuGroup.vue";
import localData from "../../navbar.json";

defineProps({ screen: { type: Boolean, default: false } });

const { theme } = useData();
const nav = ref(localData.nav);

onMounted(() => {
  if (!inBrowser) return;

  const remoteUrl = theme.value.dynamicNavUrl;
  const cacheKey = theme.value.dynamicNavCacheKey || remoteUrl || "vp-dynamic-nav";

  try {
    const cached = localStorage.getItem(cacheKey);
    if (cached) {
      const data = JSON.parse(cached);
      if (Array.isArray(data.nav)) nav.value = data.nav;
    }
  } catch {}

  if (!remoteUrl) return;

  fetch(remoteUrl)
    .then((res) => {
      if (!res.ok) throw new Error("non-200");
      return res.json();
    })
    .then((data) => {
      if (Array.isArray(data.nav)) {
        nav.value = data.nav;
        localStorage.setItem(cacheKey, JSON.stringify(data));
      }
    })
    .catch(() => {});
});
</script>

<style>
/*
 * Reposition the dynamic nav within VitePress's content-body flex container.
 * DOM order: [nav-bar-content-before(0)] [search(1, flex-grow:1)] [menu(2)]
 * [translations(3)] [appearance(4)] [social-links(5)] [extra(6)].
 * order:2 places .VPDynamicNav.bar after search and before the language selector.
 */
.VPNavBar .content-body .VPDynamicNav.bar { order: 2; }
.VPNavBar .content-body .translations     { order: 3; }
.VPNavBar .content-body .appearance       { order: 4; }
.VPNavBar .content-body .social-links     { order: 5; }
.VPNavBar .content-body .extra            { order: 6; }

/* Separator between dynamic nav and the language selector, matching
   VitePress's .menu + .translations::before pattern. Uses general sibling
   (~) because flex order differs from DOM order. */
.VPNavBar .content-body .VPDynamicNav.bar ~ .translations::before {
  margin-right: 8px;
  margin-left: 8px;
  width: 1px;
  height: 24px;
  background-color: var(--vp-c-divider);
  content: "";
}
</style>
