# Coding Style (Arduino)

Effective: 2025-11-01 — applies to new/modified code. Do not retro-format existing files unless explicitly requested.

- Indentation: 2 spaces (no tabs). Leading tabs will be converted to two spaces.
- Braces: K&R / attach style — opening brace on the same line for control statements and function definitions.

  Example
  ```c++
  void setup() {
    if (ready) {
      go();
    } else {
      wait();
    }
  }
  ```

- Spacing:
  - One space before `(` in control keywords: `if (x) {`, `while (y) {`.
  - One space before `{` when attached: `) {`.
- Lines end with `\n`.
- Keep changes minimal: do not churn formatting on untouched lines in large edits.

## Tooling

Use `tools/format_c_style.py` to lightly normalize new edits:
- Converts leading tabs to 2 spaces.
- Attaches a standalone `{` to the previous control or function signature line with a single space.
- Normalizes spacing before inline `{`.

This tool is conservative and won’t reflow code. It only makes small local changes that align with Arduino IDE’s Auto Format defaults.
