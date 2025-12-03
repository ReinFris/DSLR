## 📦 Project Setup & Environment

* This repo uses **PlatformIO + VS Code extension** for building firmware.

* Toolchain: GCC (or whichever compiler PlatformIO uses for target board).

* Build commands (from root):

  ```
  pio run            # build
  pio run --target upload     # compile & upload to board
  pio run --target clean      # clean build artifacts
  ```

* Testing (if applicable — e.g. native / host-based tests):
  Specify here any test commands, test environment setup etc.

* If you rely on third-party libraries: document them here (e.g. `lib_deps` from platformio.ini, or manually included headers).

## 🔧 Git / Repo Etiquette

* Branch naming: use `feature/…`, `bugfix/…`, `hotfix/…` style.
* Before pushing a branch, always build and (if possible) test on a real board or simulator.
* Commit often — after each working feature or small incremental change (especially before hardware flashing).
* When using `git rebase` or rewriting history, ensure the code still builds cleanly.

## 🧰 Code Style & Conventions (C++ / Embedded)

* Use **modern C++ where feasible**, but avoid heavy dynamic allocation (heap) — prefer stack or statically allocated buffers, since embedded environment often lacks dynamic memory safely.

* Use clear, descriptive naming for variables/functions — e.g. `led_on()`, `sensor_read()` rather than cryptic short names.

* Keep source files modular: aim for **max ~500–600 lines per `.cpp/.h` file** — easier to navigate, easier for Claude to reason on a single file. (Community feedback from Claude users suggests this helps clarity.)

* Minimize header dependencies; use forward declarations where possible to reduce compile time and coupling.

* Prefer `constexpr`, `static inline`, or `constexpr inline` for compile-time constants and small utility functions.

* Clearly separate hardware-abstraction layers (HAL) vs application logic: e.g. dedicated files for pin definitions, board init, driver interfaces, and higher-level logic.

* Document hardware-specific quirks (pinouts, board variants, flash sizes, MCU features) in code comments or README; duplicate brief notes here if relevant.

* (Optional) Linting / style enforcement: consider using a linter for C++ (or formatting tool). For example, you could run cpplint to catch style deviations.

## 🧪 Build / Test / Debug Workflow

* After implementing a change: run `pio run` → check for compile warnings/errors.
* If you have board test firmware: track version/commit and perform a real-world test (upload + run) before merging.
* Document any manual test procedures (e.g. “connect board via USB, open serial monitor at 115200 baud, press reset, send command X, expect response Y”) — this is especially helpful if others will collaborate.
* If you add new dependencies or update platformio.ini, note the change here (library versions, special flags, board definitions, etc.).

