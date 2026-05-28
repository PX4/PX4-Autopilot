<template>
  <!-- Mobile (nav-screen) layout -->
  <div v-if="screen" class="VPVersionSwitcher screen">
    <div class="screen-title">Version</div>
    <nav>
      <a
        v-for="v in versions"
        :key="v.link"
        :href="v.link"
        class="screen-item"
      >{{ v.text }}</a>
    </nav>
  </div>

  <!-- Desktop (nav-bar) layout -->
  <div v-else class="VPVersionSwitcher bar" ref="containerRef">
    <button
      class="bar-trigger"
      :aria-expanded="isOpen"
      @click="isOpen = !isOpen"
    >
      Version
      <svg class="chevron" :class="{ open: isOpen }" width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
        <polyline points="6 9 12 15 18 9" />
      </svg>
    </button>
    <div v-if="isOpen" class="bar-menu">
      <a
        v-for="v in versions"
        :key="v.link"
        :href="v.link"
        class="bar-item"
        @click="isOpen = false"
      >{{ v.text }}</a>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from "vue";
import { inBrowser } from "vitepress";
import localData from "../../versions.json";

const props = defineProps({
  screen: { type: Boolean, default: false },
});

const CACHE_KEY = "px4-docs-versions-cache";
const REMOTE_URL =
  "https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/docs/.vitepress/versions.json";

const versions = ref(localData.versions);
const isOpen = ref(false);
const containerRef = ref(null);

function handleClickOutside(e) {
  if (containerRef.value && !containerRef.value.contains(e.target)) {
    isOpen.value = false;
  }
}

onMounted(() => {
  if (!inBrowser) return;

  // Use cache immediately if available
  try {
    const cached = localStorage.getItem(CACHE_KEY);
    if (cached) {
      const data = JSON.parse(cached);
      if (Array.isArray(data.versions)) versions.value = data.versions;
    }
  } catch {}

  // Always attempt a fresh fetch; update cache on success
  fetch(REMOTE_URL)
    .then((res) => {
      if (!res.ok) throw new Error("non-200");
      return res.json();
    })
    .then((data) => {
      if (Array.isArray(data.versions)) {
        versions.value = data.versions;
        localStorage.setItem(CACHE_KEY, JSON.stringify(data));
      }
    })
    .catch(() => {});

  document.addEventListener("click", handleClickOutside);
});

onUnmounted(() => {
  if (inBrowser) document.removeEventListener("click", handleClickOutside);
});
</script>

<style scoped>
/* ── Desktop bar variant ── */
.bar {
  position: relative;
  display: flex;
  align-items: center;
}

/* Separator line matching VitePress's .menu + .translations::before style */
.bar::before {
  content: "";
  display: block;
  width: 1px;
  height: 24px;
  background-color: var(--vp-c-divider);
  margin-left: 8px;
  margin-right: 8px;
}

.bar-trigger {
  display: flex;
  align-items: center;
  gap: 4px;
  padding: 0 8px;
  height: 32px;
  border-radius: 6px;
  border: none;
  background: transparent;
  color: var(--vp-c-text-1);
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: color 0.2s, background-color 0.2s;
  white-space: nowrap;
}

.bar-trigger:hover {
  color: var(--vp-c-brand-1);
}

.chevron {
  transition: transform 0.2s;
}

.chevron.open {
  transform: rotate(180deg);
}

.bar-menu {
  position: absolute;
  top: calc(100% + 8px);
  right: 0;
  min-width: 140px;
  background: var(--vp-c-bg-soft);
  border: 1px solid var(--vp-c-divider);
  border-radius: 8px;
  padding: 6px;
  box-shadow: var(--vp-shadow-3);
  z-index: 100;
}

.bar-item {
  display: block;
  padding: 6px 10px;
  border-radius: 4px;
  color: var(--vp-c-text-1);
  font-size: 13px;
  font-weight: 400;
  text-decoration: none;
  transition: background-color 0.2s, color 0.2s;
  white-space: nowrap;
}

.bar-item:hover {
  background: var(--vp-c-default-soft);
  color: var(--vp-c-brand-1);
}

/* ── Mobile screen variant ── */
.screen {
  border-top: 1px solid var(--vp-c-divider);
  padding: 12px 0 0;
  margin-top: 12px;
}

.screen-title {
  padding: 0 12px 8px;
  font-size: 12px;
  font-weight: 600;
  color: var(--vp-c-text-2);
  text-transform: uppercase;
  letter-spacing: 0.04em;
}

.screen-item {
  display: block;
  padding: 8px 12px;
  color: var(--vp-c-text-1);
  font-size: 14px;
  font-weight: 500;
  text-decoration: none;
  border-radius: 6px;
  transition: background-color 0.2s, color 0.2s;
}

.screen-item:hover {
  background: var(--vp-c-default-soft);
  color: var(--vp-c-brand-1);
}
</style>

<style>
/*
 * Reposition the desktop version switcher within VitePress's content-body flex
 * container. The DOM order is: [nav-bar-content-before] [search(flex-grow:1)]
 * [menu] [translations] [appearance] [social-links] [extra] [nav-bar-content-after].
 * Without these overrides, the slot content appears to the LEFT of the search
 * bar. Setting order:3 on the switcher and bumping subsequent items ensures it
 * renders between the menu items and the language selector.
 */
.VPNavBar .content-body .VPVersionSwitcher.bar { order: 3; }
.VPNavBar .content-body .translations          { order: 4; }
.VPNavBar .content-body .appearance            { order: 5; }
.VPNavBar .content-body .social-links          { order: 6; }
.VPNavBar .content-body .extra                 { order: 7; }
</style>
