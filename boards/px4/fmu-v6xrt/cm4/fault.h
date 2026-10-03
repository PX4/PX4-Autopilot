#pragma once

#include <stddef.h>
#include <stdint.h>

/* Fault injection for the robustness pass. Reached through the hello
 * endpoint as "!fault <kind>" (nsh: remote_core fault <kind>). Compiled in with
 * FAULT_INJECT=1 (Makefile default); kinds are listed by "!fault list".
 */

/* Set by "lockup": the exception handler faults again, escalating to
 * a Cortex-M lockup, which SRC_SRMR must keep from resetting the chip.
 */
extern volatile uint32_t cm4_fault_escalate;

/* Writes the outcome into reply[cap]. Lethal kinds do not return. */
void fault_inject(const char *kind, char *reply, size_t cap);

/* Layer hook: handle kind (or append names for "list") and return 1;
 * return 0 for kinds it does not know. Weak default knows none.
 */
int fault_inject_app(const char *kind, char *reply, size_t cap);

/* "<kind>: <text>" into reply[cap]. */
void fault_reply(char *reply, size_t cap, const char *kind, const char *text);
