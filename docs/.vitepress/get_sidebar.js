// .vuepress/get_sidebar.js

import path from "path";
import fs from "fs";
import readline from "readline";

function getEntryArray(parent) {
  let returnEntryArray = null;
  //console.log("getEntryArray(): parent");
  //console.log(parent);
  //Gets returns the child array
  //if (parent === "undefined") {
  //  return [];
  //}

  if (Array.isArray(parent)) {
    //This parent is an array, so return it (it is the sidebar top level)
    returnEntryArray = parent;
    //console.log("isArray");
  }
  parent.items = parent?.items ? parent.items : [];
  //console.log("isNotArray");
  //console.log("returnEntryArray:");
  returnEntryArray = parent.items;
  //console.log(returnEntryArray);
  return returnEntryArray;
}

function parseGitbookSidebarToVuepress(sidebarContent, lang) {
  const lines = sidebarContent.split("\n");

  const newSidebar = [];

  let first_iteration = true;
  let indent_divider = 0;

  const parents = [];
  //console.log("parents - empty after creaton");
  //console.log(parents);

  const topLevelParent = {
    title: "DUMMY",
    path: "DUMMY",
    //level: "0",
    //items: [],
  };
  topLevelParent.items = [];
  //console.log("topLevelParent - empty after creaton");
  //console.log(topLevelParent);

  parents.push(topLevelParent); // So last item is always the sidebar
  //console.log("parents after pushing topLevelParent:");
  //console.log(parents);

  let current_parent;

  let lastlevel = 0;
  let last_item;

  //let currentSection;
  //let currentLevel = 0;

  lines.forEach((line) => {
    if (line.startsWith("#") || line.trim() === "") {
      // Ignore lines that start with "#" or are empty
      // Note, we should perhaps create sections for these. TBD.
      return;
    }

    //console.log("DEBUG: Line: " + line);

    let regex = /(\s*?)[\*-]\s\[(.*?)\]\((.*?)\)/g;

    let indent_level = "";
    let link_title = "";
    let link_url = "";
    let link_path = "";

    try {
      //console.log(`DEBUG: Lastitem at start:  ${JSON.stringify(last_item)}`);
      let match = regex.exec(line);
      //console.log('0: '+ match[0])
      indent_level = match[1].length;
      link_title = match[2];
      link_url = match[3].trim();
      link_path = link_url;
    } catch (err) {
      //Just skip empty lines that don't match
      console.log(err);
      console.log("DEBUG: Couldn't match line, skip line using return");
      return;
    }

    try {
      /*
      if (link_url.endsWith("README.md")) {
        link_url = link_url.replace("README.md", "");
      }
      */
      if (link_url.endsWith(".md")) {
        link_url = link_url.replace(".md", ".html");
        //link_path = link_path.replace(".md", ".html");
      }

      if (!link_url.startsWith("http")) {
        if (lang) {
          link_url = `/${lang}/${link_url}`;
        } else {
          link_url = `/${link_url}`;
        }
      }

      //Tidy up some of the title escaping that isn't used by VuePress
      link_title = link_title.replace("\\(", "(");
      link_title = link_title.replace("\\)", ")");
      link_title = link_title.replace("\\_", "_");

      //set indent_divider level (easier to think in levels, numbers of zero prefixes)
      if ((indent_divider == 0) & (indent_level > 0.0)) {
        indent_divider = indent_level;
      }
      if (indent_divider > 0) {
        indent_level = indent_level / indent_divider;
      }

      let entry = {
        text: link_title,
        link: link_url,
        //collapsed: true,
        //link_path,
        //level: indent_level,
        //collapsible: true,
      };
      //console.log("entry:");
      //console.log(entry);

      //console.log("parents:");
      //console.log(parents);
      //console.log("current_parent - before pop");
      //console.log(current_parent);
      current_parent = parents.pop();
      //console.log("current_parent - after pop");
      //console.log(current_parent);

      //console.log(`XX ST: indent: ${indent_level} lastlevel: ${lastlevel}`);
      if (indent_level == lastlevel) {
        //console.log(`XX EQ:`);
        //console.log(current_parent);
        const parentArray = getEntryArray(current_parent);
        parentArray.push(entry);
        //console.log("current_parent after pushing last entry");
        //console.log(current_parent);

        //console.log("eq parents before pushing back current parent");
        //console.log(parents);
        parents.push(current_parent);
        //console.log("EQ parents after pushing current parent");
        //console.log(parents);

        //console.log("XX EQ3");
      } else if (indent_level > lastlevel) {
        //console.log("XX NEST");
        //console.log(
        //  `  DEBUG:NEST: lastlev: ${lastlevel}/indent lev: ${indent_level}`
        // );
        // This is a child of the last element added to current parent

        //console.log(current_parent);
        //console.log("NEST: current_parent - after pop");
        //console.log("NEST: parents before");
        //console.log(parents);

        const parentArray = getEntryArray(current_parent);

        //console.log(
        //  "parentArray - we want to add entry to last entry in this array."
        //);
        //console.log(parentArray);

        //Get the last element
        //console.log(`how many elements in array ${parentArray.length}`);
        //
        const lastElement = parentArray.pop();
        lastElement.collapsed = true;
        //console.log(`NEST: last element in array:`);
        //console.log(lastElement);
        const lastElementArray = getEntryArray(lastElement);
        //console.log(`NEST: last element array:`);
        lastElementArray.push(entry);
        //console.log(`NEST: last element after pushing event:`);
        //console.log(lastElement);
        // Push the last element back on to the parentArray
        parentArray.push(lastElement);
        //push current parent onto the parents array
        parents.push(current_parent);
        // push the element we just edited back onto the array so it can be current parent.
        parents.push(lastElement);
        //console.log("NEST: parents after");
        //console.log(parents);
      } else if (indent_level < lastlevel) {
        // gone up a level
        //console.log(`XX UP`);
        while (indent_level < lastlevel--) {
          //console.log(`pop ${lastlevel}`);
          //Here we have no item. Group has finished (with a group)
          //So add the current parent (finished) to its parent.
          //console.log('DEBUG: Gone UP from level: '+ indent_level + ' TO: ' + lastlevel)

          current_parent = parents.pop();
        }
        const parentArray = getEntryArray(current_parent);
        parentArray.push(entry);
        //console.log("current_parent after pushing last entry");
        //console.log(current_parent);

        //console.log("eq parents before pushing back current parent");
        //console.log(parents);
        parents.push(current_parent);
      }

      //console.log("AFTER ADDING Entry:");
      //console.log("entry:");
      //console.log(entry);
      //console.log("parents:");
      //console.log(parents);
    } catch (err) {
      console.log(err);
      console.log(` DEBUG SOME ACTUAL PROBLEMXX2: ${err}`);
    }

    //last_item = entry;

    lastlevel = indent_level; //reset
    //console.log(`END OF ONE LINE:${indent_level} lastlevel: ${lastlevel}`);
    //console.log("PARENTS");
    //console.log(JSON.stringify(parents));
  });

  //all lines done
  //console.log("All lines done parents:");
  //console.log(JSON.stringify(parents));

  //console.log("topLevelParent");
  //console.log(topLevelParent);
  return topLevelParent.items;
}

module.exports = {
  sidebar: function (lang) {
    const summaryfile_path = path.resolve(__dirname, "..", lang, "SUMMARY.md");
    //console.log("DEBUG: summaryfile_path: " + summaryfile_path);
    let data = "";

    try {
      // read contents of the file
      data = fs.readFileSync(summaryfile_path, "UTF-8");
    } catch (err) {
      console.log(`DEBUG: ${lang} - SIDEBAR  DEFINITION NOT FOUND`);
    }

    const module_sidebar = parseGitbookSidebarToVuepress(data, lang);
    //console.log(`DEBUG: Before`);
    //console.log(`DEBUG: ${JSON.stringify(module_sidebar)}`);
    //console.log(`DEBUG: After`);
    return module_sidebar;
  },
};
