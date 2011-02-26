file build/template.elf
target remote :3333
monitor reset halt
tbreak main

define reset
mon reset init
end

define reload
load
reset
continue
end

define hook-step
mon cortex_m3 maskisr on
end

define hookpost-step
mon cortex_m3 maskisr off
end

define hook-stepi
mon cortex_m3 maskisr on
end

define hookpost-stepi
mon cortex_m3 maskisr off
end

define hook-next
mon cortex_m3 maskisr on
end

define hookpost-next
mon cortex_m3 maskisr off
end
