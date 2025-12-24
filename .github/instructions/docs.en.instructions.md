---
applyTo: "docs/en/**"
---

# Review Guidelines docs/en Tree

## File System & Structure

- **Naming:** Use `lowercase_with_underscores` for all filenames. No spaces.
- **Hierarchy:** Markdown files must reside exactly in a first-level category folder. 
  - Valid: `docs/en/category/file.md`
  - Invalid: `docs/en/category/subcategory/file.md`
- **Text Files:** Any `.txt` or `.text` files must start with an underscore (e.g., `_notes.txt`).
- **Assets:** All images/non-docs must be in `/docs/assets/`. Deep nesting is permitted here.
- **Formats:** Prefer **SVG** for diagrams and **PNG** for screenshots. Flag JPG files.

## Markdown & Style

- **Headings:** Use Title Case ("First Letter Capitalisation"). 
  - The Page Title must be the only H1 (`#`). All others must be `##` or lower.
  - Do not apply bold or italic styling inside a heading.
- **Formatting:** 
  - **Bold:** Only for UI elements (buttons, menu items).
  - **Inline Code:** Use backticks for file paths, parameters, and CLI commands (e.g., `prettier`).
    Backticks are optional for hyperlinked CLI commands and tool names.
  - **Italics (Emphasis):** Use for application names (e.g., *QGroundControl*).
    Emphasis is optional for hyperlinked applications.
- **Structure:** End every line at the end of a sentence (Semantic Line Breaks).

## Linking & Navigation

- **Standard Links:** Use standard inline syntax: `[link text](../category/filename.md)`.
  Note relative link. 
- **Table Links:** To keep tables readable, use reference-style links.
  - Definition: `[Link Name]: https://example.com` (placed below the table).
  - Usage: `[Link Name]` within the table cell.
- **Images:** All image links must include a descriptive, accessible alt-text in the brackets: `![Detailed description of the image content](../../assets/path/to/image.png)`.
  Note that all images should be relative references to images stored in the assets folder, which should be two folders below the any markdown file (as they are stored in a "category" subfolder) 

- **Standard Links:** Use standard inline syntax: `[link text](../category/filename.md)`. Note the use of relative links.
- **Table Links:** To keep tables easier to edit, prefer reference-style links.
  - Definition: `[Link Name]: https://example.com` (placed below the table).
  - Usage: `[Link Name]` within the table cell.
- **Images:** All image links must include a descriptive, accessible alt-text: `![Detailed description of the image content](../../assets/path/to/image.png)`.
  - **Note:** All images must be relative references to the `/docs/assets/` folder. Since documents are nested in a category folder, this is usually two levels up (`../../assets/`).

## Quality Control

- **Prettier Check:** Ensure Prettier rules have been applied. If there is evidence of inconsistent indentation or spacing, request the author run `npx prettier --write .` before merging.
- **Language:** Enforce **UK English** spelling and grammar.
