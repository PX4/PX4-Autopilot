---
applyTo: "src/modules/commander/**,src/modules/logger/**,src/systemcmds/**,platforms/**,src/modules/dataman/**"
---

# System Review Guidelines

In addition to the core code review guidelines:

- Race conditions and concurrency: no partial fixes, demand complete solutions
- Semaphore/scheduling edge cases; understand RTOS guarantees
- State machine sequential-logic bugs (consecutive RTL, armed/disarmed alternation)
- Use uORB-driven scheduling (`SubscriptionCallback`), not extra threads
- `param_set` triggers auto-save; no redundant `param_save_default`
- Flash/memory efficiency: avoid `std::string` on embedded, minimize SubscriptionData usage
- Constructor initialization order matters
