<script setup>
import { onMounted, ref } from "vue";
import { useData, inBrowser } from "vitepress";

const DISPLAY_MS = 2000;

const { site } = useData();
const visible = ref(false);
const message = ref("");

// Layout mounts once per real page load and persists across in-session SPA
// navigation, so this fires once per hard load/refresh, never per page view.
onMounted(() => {
  if (!inBrowser) return;

  message.value = `Now viewing ${site.value.title}`;
  visible.value = true;
  setTimeout(() => {
    visible.value = false;
  }, DISPLAY_MS);
});
</script>

<template>
  <Transition name="version-toast-fade">
    <div v-if="visible" class="version-toast vp-doc" role="status">
      <h1>{{ message }}</h1>
      <button
        class="version-toast-close"
        aria-label="Dismiss"
        @click="visible = false"
      >
        &times;
      </button>
    </div>
  </Transition>
</template>

<style scoped>
.version-toast {
  position: fixed;
  top: 33vh;
  left: 50%;
  transform: translate(-50%, -50%);
  z-index: 90;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 24px;
  width: 90vw;
  max-width: 688px; /* matches VitePress's default doc content column width */
  padding: 24px 32px;
  /* Opaque page-surface base with the theme's translucent warning tint
     layered on top, so the tint stays theme-adaptive without looking washed out. */
  background-color: var(--vp-c-bg);
  background-image: linear-gradient(var(--vp-c-warning-soft), var(--vp-c-warning-soft));
  color: var(--vp-c-warning-1);
  border-radius: 8px;
  box-shadow: var(--vp-shadow-3);
  text-align: center;
}

/* The "vp-doc" class on .version-toast makes this pick up VitePress's own
   .vp-doc h1 font-size/line-height rule (including its responsive
   breakpoint) instead of duplicating those numbers here. h1 margin is
   already reset to 0 by VitePress's base.css. */

.version-toast-close {
  background: none;
  border: none;
  padding: 0;
  color: inherit;
  cursor: pointer;
  font-size: 32px;
  line-height: 1;
}

.version-toast-fade-enter-active,
.version-toast-fade-leave-active {
  transition: opacity 0.3s ease;
}

.version-toast-fade-enter-from,
.version-toast-fade-leave-to {
  opacity: 0;
}
</style>
