target remote :3333
mon reset halt
mon poll
mon cortex_m maskisr auto
set mem inaccessible-by-default off
set print pretty
source Debug/PX4