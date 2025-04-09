import { defineConfig } from "vitepress";
const getSidebar = require("./get_sidebar.js");

import openEditor from "open-editor"; // Open file locally via edit

// Tabs: https://github.com/Red-Asuka/vitepress-plugin-tabs
import tabsPlugin from "@red-asuka/vitepress-plugin-tabs";

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: "PX4 Guide (main)",
  description: "PX4 User and Developer Guide",
  base: process.env.BRANCH_NAME
    ? "/" + process.env.BRANCH_NAME + "/"
    : "/px4_user_guide/",
  srcExclude: [
    "de/**/*.md",
    "ja/**/*.md",
    "ru/**/*.md",
    "tr/**/*.md",
    //"ko/**/*.md",
    //"zh/**/*.md",
    //"uk/**/*.md",
    "**/_*.md", //Remove source docs that start with "_" (included/not rendered)
  ],
  ignoreDeadLinks: true,
  markdown: {
    math: true,
    config: (md) => {
      // use more markdown-it plugins!
      tabsPlugin(md); //https://github.com/Red-Asuka/vitepress-plugin-tabs
    },
  },

  vite: {
    plugins: [
      {
        // Open file locally via edit
        name: "open-in-editor",
        configureServer(server) {
          server.middlewares.use("/__open-in-editor", (req, res, next) => {
            if (!req.url) return next();
            const q = new URL(req.url, "http://a.com").searchParams;
            const file = q.get("file");
            if (!file) return next();
            const line = Number.parseInt(q.get("line")) || 1;
            const column = Number.parseInt(q.get("column")) || 1;
            // Open editor if EDITOR environment variable is set
            if (typeof process.env.EDITOR !== "undefined") {
              openEditor([{ file, line, column }]);
            } else {
              console.warn(
                "EDITOR environment variable is not set. Skipping opening file."
              );
            }
            res.statusCode = 204;
            res.end();
          });
        },
      },
    ],
  },

  locales: {
    en: {
      label: "English",
      // other locale specific properties...
      themeConfig: {
        sidebar: getSidebar.sidebar("en"),
        editLink: {
          text:
            /* We get a github link if CI env is defined,
            or if it isn't defined and a local editor isn't defined) */
            typeof process.env.CI !== "undefined" ||
            typeof process.env.EDITOR === "undefined"
              ? "Edit on GitHub"
              : "Open in your editor",
          pattern:
            typeof process.env.CI !== "undefined" ||
            typeof process.env.EDITOR === "undefined"
              ? ({ filePath, frontmatter }) => {
                  if (frontmatter.newEditLink) {
                    //newEditLink defines a frontmatter key you can use to append a path to main
                    return `https://github.com/PX4/PX4-Autopilot/edit/main/docs/${frontmatter.newEditLink}`;
                  } else {
                    return `https://github.com/PX4/PX4-Autopilot/edit/main/docs/${filePath}`;
                  }
                }
              : (c) =>
                  `${
                    window.location.origin
                  }/__open-in-editor?file=${encodeURIComponent(
                    c.frontmatter.newEditLink
                      ? c.frontmatter.newEditLink
                      : c.filePath
                  )}`,
        },
      },
    },

    zh: {
      label: "中文 (Chinese)",
      lang: "zh-CN", // optional, will be added  as `lang` attribute on `html` tag
      themeConfig: {
        sidebar: getSidebar.sidebar("zh"),
      },
      // other locale specific properties...
    },
    ko: {
      label: "한국어 (Korean)",
      lang: "ko-KR", // optional, will be added  as `lang` attribute on `html` tag
      themeConfig: {
        sidebar: getSidebar.sidebar("ko"),
      },

      // other locale specific properties...
    },

    uk: {
      label: "Мови (Ukrainian)",
      lang: "uk-UA", // optional, will be added  as `lang` attribute on `html` tag
      themeConfig: {
        sidebar: getSidebar.sidebar("uk"),
      },

      // other locale specific properties...
    },
  },
  //Logs every page loaded on build. Good way to catch errors not caught by other things.
  async transformPageData(pageData, { siteConfig }) {
    console.log(pageData.filePath);
  },

  //

  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    logo: "/px4-logo.svg",
    sidebar: getSidebar.sidebar("en"),

    editLink: {
      pattern: "https://crowdin.com/project/px4-user-guide",
      text: "Edit translation on Crowdin",
    },

    search: {
      provider: "local",
      /*
      provider: process.env.BRANCH_NAME ? "algolia" : "local",
      options: {
        appId: "HHWW7I44JO",
        apiKey: "48919e1dffc6e0ce4c0d6331343d2c0e",
        indexName: "px4",
        searchParameters: {
          facetFilters: [`version:${process.env.BRANCH_NAME}`],
        },
      },
      */
    },

    nav: [
      {
        text: "Dronecode",
        items: [
          {
            text: "PX4",
            link: "https://px4.io/",
            ariaLabel: "PX4 website link",
          },
          {
            text: "QGroundControl",
            link: "http://qgroundcontrol.com/",
          },
          {
            text: "MAVSDK",
            link: "https://mavsdk.mavlink.io/",
          },
          {
            text: "MAVLINK",
            link: "https://mavlink.io/en/",
          },
          {
            text: "QGroundControl Guide",
            link: "https://docs.qgroundcontrol.com/master/en/qgc-user-guide/",
          },
          {
            text: "Dronecode Camera Manager",
            link: "https://camera-manager.dronecode.org/en/",
          },
        ],
      },
      {
        text: "Support",
        link: "https://docs.px4.io/main/en/contribute/support.html",
      },
      {
        text: "Version",
        items: [
          { text: "main", link: "https://docs.px4.io/main/en/" },
          { text: "v1.15 (stable)", link: "https://docs.px4.io/v1.15/en/" },
          { text: "v1.14", link: "https://docs.px4.io/v1.14/en/" },
          { text: "v1.13", link: "https://docs.px4.io/v1.13/en/" },
          { text: "v1.12", link: "https://docs.px4.io/v1.12/en/" },
          { text: "v1.11", link: "https://docs.px4.io/v1.11/en/" },
        ],
      },
    ],

    socialLinks: [
      { icon: "github", link: "https://github.com/PX4/PX4-Autopilot" },
    ],
  },

  head: [
    [
      "script",
      {
        async: "",
        src: "https://www.googletagmanager.com/gtag/js?id=G-91EWVWRQ93",
      },
    ],
    [
      "script",
      {},
      `window.dataLayer = window.dataLayer || [];
      function gtag(){dataLayer.push(arguments);}
      gtag('js', new Date());
      gtag('config', 'G-91EWVWRQ93');`,
    ],
  ],

  vue: {
    template: {
      compilerOptions: {
        isCustomElement: (tag) => tag === "lite-youtube",
      },
    },
  },
});
