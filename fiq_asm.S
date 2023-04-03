#include <asm/assembler.h>
#include <linux/linkage.h>

.text
.global ps2pipad_fiq_end

ENTRY(ps2pipad_fiq)
stmdb sp!, {r0-r3, lr}
blx r8
ldmia sp!, {r0-r3, lr}
subs pc, lr, #4

ps2pipad_fiq_end:
END(ps2pipad_fiq)
