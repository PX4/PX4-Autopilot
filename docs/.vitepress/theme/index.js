// https://vitepress.dev/guide/custom-theme
import { h } from "vue";

import DefaultTheme from "vitepress/theme";
import "./style.css";

// To support medium-zoom - like setup()
import { onMounted, watch, nextTick } from "vue";
import { useRoute } from "vitepress";
import mediumZoom from "medium-zoom";

// For https://www.npmjs.com/package/lite-youtube-embed
import { inBrowser } from "vitepress";
import "lite-youtube-embed/src/lite-yt-embed.css";
if (inBrowser) {
  // @ts-ignore
  import("lite-youtube-embed");
}

// Support redirect plugin
import Redirect from "./components/Redirect.vue";

import { createDynamicNav } from "vp-dynamic-nav";

// Tabs: https://github.com/Red-Asuka/vitepress-plugin-tabs
import { Tab, Tabs } from "vue3-tabs-component";
import "@red-asuka/vitepress-plugin-tabs/dist/style.css";

/** @type {import('vitepress').Theme} */
export default {
  extends: createDynamicNav(DefaultTheme),
  enhanceApp({ app, router, siteData }) {
    app.component("Redirect", Redirect); //Redirect plugin
    //Tabs: https://github.com/Red-Asuka/vitepress-plugin-tabs
    app.component("Tab", Tab);
    app.component("Tabs", Tabs);
    // Global build time variable
    app.config.globalProperties.$buildTime = JSON.stringify(
      new Date().toISOString()
    );
    // Switching version in the navbar keeps the current page. The "Version"
    // links in navbar.json point at each version's root (e.g.
    // https://docs.px4.io/v1.16/en/), so swap the path of the current page in.
    // Pages that do not exist in the target version 404.
    if (inBrowser) {
      addEventListener(
        "click",
        (e) => {
          if (e.defaultPrevented || e.button || e.ctrlKey || e.metaKey || e.shiftKey || e.altKey) return;
          const link = e.target.closest?.(".VPNav a[href]");
          // Only bare version roots, i.e. the version switcher entries: the site
          // logo is root-relative and "Support" has a deeper path.
          const version = link?.getAttribute("href").match(/^(https?:\/\/[^/]+\/(?:main|v\d+\.\d+)\/)(?:[a-z]{2}\/)?$/);
          if (!version || new URL(link.href).origin !== location.origin) return;
          if (!/^\/(?:main|v\d+\.\d+)\//.test(location.pathname)) return;
          e.preventDefault();
          location.href = version[1] + location.pathname.split("/").slice(2).join("/") + location.hash;
        },
        true
      );
    }
  },

  // to support medium zoom: https://github.com/vuejs/vitepress/issues/854
  setup() {
    const route = useRoute();
    const initZoom = () => {
      //mediumZoom("[data-zoomable]", { background: "var(--vp-c-bg)" });
      mediumZoom(".main img", { background: "var(--vp-c-bg)" });
    };
    onMounted(() => {
      initZoom();
      // Re-scroll to hash after fonts/layout settle. Firefox does not re-scroll
      // after late layout shifts on huge pages (e.g. parameter_reference), so
      // the initial anchor jump lands far off. Run only on initial load.
      if (inBrowser && location.hash) {
        const id = decodeURIComponent(location.hash.slice(1));
        const fontsReady = document.fonts?.ready ?? Promise.resolve();
        const loadReady =
          document.readyState === "complete"
            ? Promise.resolve()
            : new Promise((r) =>
                window.addEventListener("load", r, { once: true })
              );
        Promise.all([fontsReady, loadReady]).then(() => {
          requestAnimationFrame(() =>
            requestAnimationFrame(() => {
              const el = document.getElementById(id);
              if (el) el.scrollIntoView();
            })
          );
        });
      }
    });
    watch(
      () => route.path,
      () => nextTick(() => initZoom())
    );
  },
  //end to support medium zoom
};
