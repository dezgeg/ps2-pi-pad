#include <asm/assembler.h>
#include <linux/linkage.h>

.text
.global ps2pipad_fiq_end

ENTRY(ps2pipad_fiq)
// todo: only push r0-r3 ? (r4-r7 callee saved in EABI)
stmdb sp!, {r0-r12, lr}
// todo: needed?
mov     fp, sp

blx r8

ldmia sp!, {r0-r12, lr};
subs pc, lr, #4;

ps2pipad_fiq_end:
END(ps2pipad_fiq)
