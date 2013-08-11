target remote :3333

# Don't let GDB get confused while stepping
define hook-step
  mon cortex_m maskisr on
end
define hookpost-step
  mon cortex_m maskisr off
end

mon init
mon stm32_init
# mon reset halt
mon poll
mon cortex_m maskisr auto
set mem inaccessible-by-default off
set print pretty
source Debug/PX4

echo PX4 resumed, press ctrl-c to interrupt\n
continue
