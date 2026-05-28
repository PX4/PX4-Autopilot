<template>
  <!-- Mobile (nav-screen) layout -->
  <div v-if="screen" class="VPDynamicNav screen">
    <template v-for="(item, i) in nav" :key="i">
      <a v-if="!item.items" :href="item.link" class="screen-link" @click.prevent="navigate(item.link)">{{ item.text }}</a>
      <div v-else class="screen-group">
        <div class="screen-group-title">{{ item.text }}</div>
        <a
          v-for="sub in item.items"
          :key="sub.link"
          :href="sub.link"
          class="screen-item"
          @click.prevent="navigate(sub.link)"
        >{{ sub.text }}</a>
      </div>
    </template>
  </div>

  <!-- Desktop (nav-bar) layout -->
  <div v-else class="VPDynamicNav bar" ref="containerRef">
    <template v-for="(item, i) in nav" :key="i">
      <!-- Simple link -->
      <a v-if="!item.items" :href="item.link" class="bar-link" @click.prevent="navigate(item.link)">{{ item.text }}</a>
      <!-- Dropdown group -->
      <div v-else class="bar-group">
        <button
          class="bar-trigger"
          :aria-expanded="openIndex === i"
          @click="toggleOpen(i)"
        >
          {{ item.text }}
          <svg
            class="chevron"
            :class="{ open: openIndex === i }"
            width="12"
            height="12"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2.5"
          >
            <polyline points="6 9 12 15 18 9" />
          </svg>
        </button>
        <div v-if="openIndex === i" class="bar-menu">
          <a
            v-for="sub in item.items"
            :key="sub.link"
            :href="sub.link"
            class="bar-item"
            @click.prevent="openIndex = null; navigate(sub.link)"
          >{{ sub.text }}</a>
        </div>
      </div>
    </template>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from "vue";
import { inBrowser } from "vitepress";
import localData from "../../navbar.json";

const props = defineProps({
  screen: { type: Boolean, default: false },
});

const CACHE_KEY = "px4-docs-navbar-cache";
const REMOTE_URL =
  "https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/docs/.vitepress/navbar.json";

const nav = ref(localData.nav);
const openIndex = ref(null);
const containerRef = ref(null);

function toggleOpen(i) {
  openIndex.value = openIndex.value === i ? null : i;
}

// Use window.location.href to bypass VitePress's global click interceptor,
// which catches same-origin <a> clicks and routes them through Vue Router,
// causing cross-version navigation to fail silently.
function navigate(url) {
  if (inBrowser) window.location.href = url;
}

function handleClickOutside(e) {
  if (containerRef.value && !containerRef.value.contains(e.target)) {
    openIndex.value = null;
  }
}

onMounted(() => {
  if (!inBrowser) return;

  // Apply cache immediately if available
  try {
    const cached = localStorage.getItem(CACHE_KEY);
    if (cached) {
      const data = JSON.parse(cached);
      if (Array.isArray(data.nav)) nav.value = data.nav;
    }
  } catch {}

  // Always attempt a fresh fetch; update cache on success
  fetch(REMOTE_URL)
    .then((res) => {
      if (!res.ok) throw new Error("non-200");
      return res.json();
    })
    .then((data) => {
      if (Array.isArray(data.nav)) {
        nav.value = data.nav;
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
  display: flex;
  align-items: center;
}

.bar-link {
  display: flex;
  align-items: center;
  padding: 0 8px;
  height: var(--vp-nav-height);
  color: var(--vp-c-text-1);
  font-size: 14px;
  font-weight: 500;
  text-decoration: none;
  transition: color 0.2s;
  white-space: nowrap;
}

.bar-link:hover {
  color: var(--vp-c-brand-1);
}

.bar-group {
  position: relative;
}

.bar-trigger {
  display: flex;
  align-items: center;
  gap: 4px;
  padding: 0 8px;
  height: var(--vp-nav-height);
  border: none;
  background: transparent;
  color: var(--vp-c-text-1);
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: color 0.2s;
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
  left: 0;
  min-width: 160px;
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

.screen-link {
  display: block;
  padding: 8px 12px;
  color: var(--vp-c-text-1);
  font-size: 14px;
  font-weight: 500;
  text-decoration: none;
  border-radius: 6px;
  transition: background-color 0.2s, color 0.2s;
}

.screen-link:hover {
  background: var(--vp-c-default-soft);
  color: var(--vp-c-brand-1);
}

.screen-group {
  margin-bottom: 8px;
}

.screen-group-title {
  padding: 4px 12px;
  font-size: 12px;
  font-weight: 600;
  color: var(--vp-c-text-2);
  text-transform: uppercase;
  letter-spacing: 0.04em;
}

.screen-item {
  display: block;
  padding: 6px 12px 6px 20px;
  color: var(--vp-c-text-1);
  font-size: 14px;
  font-weight: 400;
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
 * Reposition the dynamic nav within VitePress's content-body flex container.
 * DOM order: [nav-bar-content-before(0)] [search(1, flex-grow:1)] [menu(2)]
 * [translations(3)] [appearance(4)] [social-links(5)] [extra(6)].
 * order:2 moves .VPDynamicNav.bar after search and any residual .menu,
 * placing it right before the language selector.
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
