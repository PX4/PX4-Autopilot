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
  - **Italics (Emphasis):** For tool names (e.g., *QGroundControl*).
  - **Inline Code:** Use backticks for file paths, parameters, and CLI commands (e.g., `prettier`).
- **Structure:** End every line at the end of a sentence (Semantic Line Breaks).

## Quality Control

- **Formatting:** Ensure Prettier rules have been applied.
- **Language:** Enforce **UK English** spelling and grammar.
