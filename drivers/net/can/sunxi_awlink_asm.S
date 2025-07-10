	.arch armv7-a
	.fpu softvfp
	.eabi_attribute 20, 1	@ Tag_ABI_FP_denormal
	.eabi_attribute 21, 1	@ Tag_ABI_FP_exceptions
	.eabi_attribute 23, 3	@ Tag_ABI_FP_number_model
	.eabi_attribute 24, 1	@ Tag_ABI_align8_needed
	.eabi_attribute 25, 1	@ Tag_ABI_align8_preserved
	.eabi_attribute 26, 2	@ Tag_ABI_enum_size
	.eabi_attribute 30, 4	@ Tag_ABI_optimization_goals
	.eabi_attribute 34, 1	@ Tag_CPU_unaligned_access
	.eabi_attribute 18, 2	@ Tag_ABI_PCS_wchar_t
	.file	"sunxi_awlink_asm.c"

	.syntax divided
	.syntax unified
	.arm
	.syntax unified
	.text
	.align	2
	.global	awlink_asm_write_cmdreg
	.syntax unified
	.arm
	.type	awlink_asm_write_cmdreg, %function
awlink_asm_write_cmdreg:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}	@
	.save {r4, r5, lr}
	mov	r3, #4	@ tmp141,
	.pad #20
	sub	sp, sp, #20	@,,
	mov	r5, r0	@ mod_reg_val, mod_reg_val
	mov	r4, r1	@ mod_reg_addr, mod_reg_addr
	str	r3, [sp, #8]	@ tmp141, mod_reg_val
	mov	r3, #6	@ tmp142,
	str	r3, [sp, #12]	@ tmp142, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65592, mod_reg_val
	add	r3, r3, #1	@ D.65592, D.65592,
	str	r3, [sp, #8]	@ D.65592, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65592, mod_reg_addr
	add	r3, r3, #51	@ D.65592, D.65592,
	str	r3, [sp, #12]	@ D.65592, mod_reg_addr
	ldr	r3, [sp, #12]	@ D.65592, mod_reg_addr
	cmp	r3, #10	@ D.65592,
	beq	.L10	@,
	ldr	r3, [sp, #8]	@ D.65592, mod_reg_val
	cmp	r3, #2	@ D.65592,
	ldrne	r3, [sp, #8]	@ D.65592, mod_reg_val
	bne	.L3	@,
.L10:
	ldr	r3, [sp, #12]	@ D.65592, mod_reg_addr
.L3:
	.syntax divided
@ 553 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r5, [r4, #4]	@ mod_reg_val, MEM[(volatile u32 *)_6]
@ 0 "" 2
	.arm
	.syntax unified
	mov	r3, #14	@ tmp143,
	str	r3, [sp]	@ tmp143, mod_reg_val
	mov	r3, #12	@ tmp144,
	str	r3, [sp, #4]	@ tmp144, mod_reg_addr
	ldr	r2, [sp]	@ D.65592, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65592, mod_reg_addr
	cmp	r2, r3	@ D.65592, D.65592
	ldrlt	r3, [sp]	@ D.65592, mod_reg_val
	ldrlt	r2, [sp, #4]	@ D.65592, mod_reg_addr
	addlt	r3, r3, r3, lsl #1	@ D.65592, D.65592, D.65592,
	addlt	r3, r3, r2	@ D.65592, D.65592, D.65592
	strlt	r3, [sp, #4]	@ D.65592, mod_reg_addr
	ldr	r3, [sp]	@ D.65592, mod_reg_val
	add	r3, r3, #1	@ D.65592, D.65592,
	str	r3, [sp]	@ D.65592, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65592, mod_reg_addr
	add	r3, r3, #51	@ D.65592, D.65592,
	str	r3, [sp, #4]	@ D.65592, mod_reg_addr
	ldr	r3, [sp, #4]	@ D.65592, mod_reg_addr
	cmp	r3, #10	@ D.65592,
	beq	.L11	@,
	ldr	r2, [sp, #4]	@ D.65592, mod_reg_addr
	ldr	r3, [sp]	@ D.65592, mod_reg_val
	eor	r3, r3, r2	@ D.65592, D.65592, D.65592
	str	r3, [sp]	@ D.65592, mod_reg_val
	movw	r3, #819	@ tmp148,
	ldr	r2, [sp]	@ D.65592, mod_reg_val
	cmp	r2, r3	@ D.65592, tmp148
	ldrne	r3, [sp]	@ D.65592, mod_reg_val
	bne	.L1	@,
.L11:
	ldr	r3, [sp, #4]	@ D.65592, mod_reg_addr
.L1:
	add	sp, sp, #20	@,,
	@ sp needed	@
	pop	{r4, r5, pc}	@
	.fnend
	.size	awlink_asm_write_cmdreg, .-awlink_asm_write_cmdreg
	.align	2
	.global	awlink_asm_fun0
	.syntax unified
	.arm
	.type	awlink_asm_fun0, %function
awlink_asm_fun0:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp116,
	add	r2, sp, #24	@ tmp117,,
	stm	sp, {r1, r3}	@,,
	str	r3, [sp, #20]	@ tmp116,
	str	r2, [sp, #16]	@ tmp117,
	mov	r2, r0	@, num
	str	r3, [sp, #12]	@ tmp116,
	str	r3, [sp, #8]	@ tmp116,
	ldr	r1, .L15	@,
	ldr	r0, .L15+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L16:
	.align	2
.L15:
	.word	-65529
	.word	-1308622748
	.fnend
	.size	awlink_asm_fun0, .-awlink_asm_fun0
	.align	2
	.global	awlink_asm_fun1
	.syntax unified
	.arm
	.type	awlink_asm_fun1, %function
awlink_asm_fun1:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp114,
	add	r2, sp, #24	@ tmp115,,
	str	r3, [sp, #20]	@ tmp114,
	str	r2, [sp, #16]	@ tmp115,
	mov	r2, r0	@, num
	str	r3, [sp, #12]	@ tmp114,
	str	r3, [sp, #8]	@ tmp114,
	str	r3, [sp, #4]	@ tmp114,
	str	r3, [sp]	@ tmp114,
	ldr	r1, .L19	@,
	ldr	r0, .L19+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L20:
	.align	2
.L19:
	.word	-65533
	.word	-1308622748
	.fnend
	.size	awlink_asm_fun1, .-awlink_asm_fun1
	.align	2
	.global	awlink_asm_fun2
	.syntax unified
	.arm
	.type	awlink_asm_fun2, %function
awlink_asm_fun2:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp114,
	add	r2, sp, #24	@ tmp115,,
	str	r3, [sp, #20]	@ tmp114,
	str	r2, [sp, #16]	@ tmp115,
	mov	r2, r0	@, num
	str	r3, [sp, #12]	@ tmp114,
	str	r3, [sp, #8]	@ tmp114,
	str	r3, [sp, #4]	@ tmp114,
	str	r3, [sp]	@ tmp114,
	ldr	r1, .L23	@,
	ldr	r0, .L23+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L24:
	.align	2
.L23:
	.word	-65532
	.word	-1308622748
	.fnend
	.size	awlink_asm_fun2, .-awlink_asm_fun2
	.align	2
	.global	awlink_asm_fun3
	.syntax unified
	.arm
	.type	awlink_asm_fun3, %function
awlink_asm_fun3:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp113,
	add	r2, sp, #24	@ tmp114,,
	str	r3, [sp, #20]	@ tmp113,
	str	r2, [sp, #16]	@ tmp114,
	mov	r2, r3	@,
	str	r3, [sp, #12]	@ tmp113,
	str	r3, [sp, #8]	@ tmp113,
	str	r3, [sp, #4]	@ tmp113,
	str	r3, [sp]	@ tmp113,
	ldr	r1, .L27	@,
	ldr	r0, .L27+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L28:
	.align	2
.L27:
	.word	-65531
	.word	-1308622748
	.fnend
	.size	awlink_asm_fun3, .-awlink_asm_fun3
	.align	2
	.global	awlink_asm_fun4
	.syntax unified
	.arm
	.type	awlink_asm_fun4, %function
awlink_asm_fun4:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp113,
	add	r2, sp, #24	@ tmp114,,
	str	r3, [sp, #20]	@ tmp113,
	str	r2, [sp, #16]	@ tmp114,
	mov	r2, r3	@,
	str	r3, [sp, #12]	@ tmp113,
	str	r3, [sp, #8]	@ tmp113,
	str	r3, [sp, #4]	@ tmp113,
	str	r3, [sp]	@ tmp113,
	ldr	r1, .L31	@,
	ldr	r0, .L31+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L32:
	.align	2
.L31:
	.word	-65530
	.word	-1308622748
	.fnend
	.size	awlink_asm_fun4, .-awlink_asm_fun4
	.align	2
	.global	awlink_asm_fun5
	.syntax unified
	.arm
	.type	awlink_asm_fun5, %function
awlink_asm_fun5:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp118,
	add	r2, sp, #24	@ tmp119,,
	str	r3, [sp, #20]	@ tmp118,
	str	r3, [sp, #12]	@ tmp118,
	str	r3, [sp, #8]	@ tmp118,
	str	r3, [sp, #4]	@ tmp118,
	str	r3, [sp]	@ tmp118,
	str	r2, [sp, #16]	@ tmp119,
	mov	r2, r0	@, num
	ldr	r1, .L35	@,
	ldr	r0, .L35+4	@,
	bl	__arm_smccc_smc	@
	ldr	r1, [sp, #32]	@, res.a2
	ldr	r0, [sp, #28]	@, res.a1
	bl	ioremap	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L36:
	.align	2
.L35:
	.word	-65528
	.word	-1308622748
	.fnend
	.size	awlink_asm_fun5, .-awlink_asm_fun5
	.align	2
	.global	awlink_asm_start
	.syntax unified
	.arm
	.type	awlink_asm_start, %function
awlink_asm_start:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
	.save {r4, lr}
	mov	r3, r1	@ ctrlmode, ctrlmode
	.pad #88
	sub	sp, sp, #88	@,,
	mov	r1, #4	@ tmp161,
	mov	r4, r0	@ mod_reg_addr, mod_reg_addr
	str	r1, [sp, #24]	@ tmp161, mod_reg_val
	mov	r1, #10	@ tmp162,
	str	r1, [sp, #28]	@ tmp162, mod_reg_addr
	ldr	r1, [sp, #28]	@ D.65630, mod_reg_addr
	ldr	r0, [sp, #24]	@ D.65630, mod_reg_val
	ldr	r1, [sp, #28]	@ D.65630, mod_reg_addr
	add	r0, r0, r1	@ D.65630, D.65630, D.65630
	str	r0, [sp, #28]	@ D.65630, mod_reg_addr
	ldr	r1, [sp, #28]	@ D.65630, mod_reg_addr
	cmp	r1, #2	@ D.65630,
	ldr	r1, [sp, #24]	@ D.65630, mod_reg_val
	beq	.L39	@,
	cmp	r1, #10	@ D.65630,
	ldr	r1, [sp, #28]	@ D.65630, mod_reg_addr
	beq	.L39	@,
	str	r1, [sp, #24]	@ D.65630, mod_reg_val
	movw	r1, #819	@ tmp163,
	ldr	r0, [sp, #24]	@ D.65630, mod_reg_val
	cmp	r0, r1	@ D.65630, tmp163
	ldreq	r1, [sp, #28]	@ D.65630, mod_reg_addr
	ldrne	r1, [sp, #24]	@ D.65630, mod_reg_val
.L39:
	mov	r0, #0	@ tmp166,
	add	r1, sp, #72	@ tmp167,,
	str	r0, [sp, #20]	@ tmp166,
	str	r1, [sp, #16]	@ tmp167,
	str	r0, [sp, #12]	@ tmp166,
	str	r0, [sp, #8]	@ tmp166,
	str	r0, [sp, #4]	@ tmp166,
	str	r0, [sp]	@ tmp166,
	ldr	r1, .L54	@,
	ldr	r0, .L54+4	@,
	bl	__arm_smccc_smc	@
	ldr	r3, [sp, #72]	@ res.a0, res.a0
	cmp	r3, #0	@ res.a0,
	bne	.L42	@,
	mov	r3, #14	@ tmp173,
	str	r3, [sp, #40]	@ tmp173, mod_reg_val
	mov	r3, #6	@ tmp174,
	str	r3, [sp, #44]	@ tmp174, mod_reg_addr
	movw	r3, #819	@ tmp175,
	ldr	r2, [sp, #44]	@ D.65630, mod_reg_addr
	mul	r2, r3, r2	@ D.65630, tmp175, D.65630
	str	r2, [sp, #40]	@ D.65630, mod_reg_val
	ldr	r2, [sp, #44]	@ D.65630, mod_reg_addr
	mul	r2, r3, r2	@ D.65630, tmp175, D.65630
	str	r2, [sp, #40]	@ D.65630, mod_reg_val
	ldr	r2, [sp, #40]	@ D.65630, mod_reg_val
	mov	r2, #2	@ tmp177,
	str	r2, [sp, #32]	@ tmp177, mod_reg_val
	mov	r2, #16	@ tmp178,
	str	r2, [sp, #36]	@ tmp178, mod_reg_addr
	ldr	r2, [sp, #36]	@ D.65630, mod_reg_addr
	mul	r2, r3, r2	@ D.65630, tmp175, D.65630
	str	r2, [sp, #32]	@ D.65630, mod_reg_val
	ldr	r2, [sp, #32]	@ D.65630, mod_reg_val
	cmp	r2, r3	@ D.65630, tmp175
	beq	.L53	@,
	ldr	r3, [sp, #32]	@ D.65630, mod_reg_val
	cmp	r3, #2	@ D.65630,
	bne	.L45	@,
.L53:
	ldr	r3, [sp, #36]	@ D.65630, mod_reg_addr
	b	.L37	@
.L45:
	ldr	r3, [sp, #32]	@ D.65630, mod_reg_val
	cmp	r3, #10	@ D.65630,
	ldr	r3, [sp, #36]	@ D.65630, mod_reg_addr
	ldrne	r3, [sp, #32]	@ D.65630, mod_reg_val
	b	.L37	@
.L42:
	mov	r3, #512	@ tmp181,
	str	r3, [sp, #48]	@ tmp181, mod_reg_val
	mov	r3, #50	@ tmp182,
	str	r3, [sp, #52]	@ tmp182, mod_reg_addr
	ldr	r3, [sp, #52]	@ D.65630, mod_reg_addr
	add	r3, r3, #100	@ D.65630, D.65630,
	str	r3, [sp, #52]	@ D.65630, mod_reg_addr
	ldr	r3, [sp, #52]	@ D.65630, mod_reg_addr
	sub	r3, r3, #100	@ D.65630, D.65630,
	str	r3, [sp, #52]	@ D.65630, mod_reg_addr
	ldr	r3, [sp, #48]	@ D.65630, mod_reg_val
	ldr	r2, [sp, #52]	@ D.65630, mod_reg_addr
	add	r3, r3, r2	@ D.65630, D.65630, D.65630
	cmp	r3, #384	@ D.65630,
	bhi	.L52	@,
	mov	r3, #256	@ tmp185,
	str	r3, [sp, #56]	@ tmp185, mod_reg_val
	mov	r3, #66	@ tmp186,
	str	r3, [sp, #60]	@ tmp186, mod_reg_addr
	ldr	r3, [sp, #56]	@ D.65630, mod_reg_val
	ldr	r2, [sp, #60]	@ D.65630, mod_reg_addr
	add	r3, r3, r2	@ D.65630, D.65630, D.65630
	cmp	r3, #384	@ D.65630,
	bcs	.L49	@,
.L52:
	.syntax divided
@ 626 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	mov	r3, #48	@ tmp188,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r4, #16]	@ tmp188,
@ 0 "" 2
	.arm
	.syntax unified
.L49:
	mov	r3, #14	@ tmp189,
	str	r3, [sp, #64]	@ tmp189, mod_reg_val
	mov	r3, #2	@ tmp190,
	str	r3, [sp, #68]	@ tmp190, mod_reg_addr
	ldr	r2, [sp, #64]	@ D.65630, mod_reg_val
	ldr	r3, [sp, #68]	@ D.65630, mod_reg_addr
	cmp	r2, r3	@ D.65630, D.65630
	ldrlt	r3, [sp, #68]	@ D.65630, mod_reg_addr
	ldrge	r3, [sp, #64]	@ D.65630, mod_reg_val
.L37:
	add	sp, sp, #88	@,,
	@ sp needed	@
	pop	{r4, pc}	@
.L55:
	.align	2
.L54:
	.word	-65535
	.word	-1308622748
	.fnend
	.size	awlink_asm_start, .-awlink_asm_start
	.align	2
	.global	awlink_asm_set_bittiming
	.syntax unified
	.arm
	.type	awlink_asm_set_bittiming, %function
awlink_asm_set_bittiming:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}	@
	.save {r4, r5, lr}
	mov	r3, #8	@ tmp133,
	.pad #20
	sub	sp, sp, #20	@,,
	mov	r4, r2	@ cfg, cfg
	mov	r5, r0	@ mod_reg_addr, mod_reg_addr
	str	r3, [sp]	@ tmp133, mod_reg_val
	mov	r3, #16	@ tmp134,
	str	r3, [sp, #4]	@ tmp134, mod_reg_addr
	ldr	r2, [sp]	@ D.65642, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65642, mod_reg_addr
	cmp	r2, r3	@ D.65642, D.65642
	blt	.L67	@,
	ldr	r3, [sp]	@ D.65642, mod_reg_val
	cmp	r3, #2	@ D.65642,
	bne	.L59	@,
.L67:
	ldr	r3, [sp, #4]	@ D.65642, mod_reg_addr
	b	.L58	@
.L59:
	ldr	r3, [sp]	@ D.65642, mod_reg_val
	and	r3, r3, #8	@ D.65642, D.65642,
	str	r3, [sp, #4]	@ D.65642, mod_reg_addr
	ldr	r3, [sp]	@ D.65642, mod_reg_val
.L58:
	tst	r1, #4	@ mod_reg_val,
	ldrne	r3, [r4]	@ *cfg_6(D), *cfg_6(D)
	orrne	r3, r3, #8388608	@ D.65643, *cfg_6(D),
	strne	r3, [r4]	@ D.65643, *cfg_6(D)
	mov	r3, #1	@ tmp138,
	str	r3, [sp, #8]	@ tmp138, mod_reg_val
	mov	r3, #11	@ tmp139,
	str	r3, [sp, #12]	@ tmp139, mod_reg_addr
	ldr	r2, [sp, #8]	@ D.65642, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65642, mod_reg_addr
	cmp	r2, r3	@ D.65642, D.65642
	ldrge	r3, [sp, #8]	@ D.65642, mod_reg_val
	ldrlt	r3, [sp, #12]	@ D.65642, mod_reg_addr
	andge	r3, r3, #8	@ D.65642, D.65642,
	strge	r3, [sp, #12]	@ D.65642, mod_reg_addr
	ldrge	r3, [sp, #8]	@ D.65642, mod_reg_val
	.syntax divided
@ 638 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r4]	@ D.65643, *cfg_6(D)
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #20]	@ D.65643, MEM[(volatile u32 *)_15]
@ 0 "" 2
	.arm
	.syntax unified
	add	sp, sp, #20	@,,
	@ sp needed	@
	pop	{r4, r5, pc}	@
	.fnend
	.size	awlink_asm_set_bittiming, .-awlink_asm_set_bittiming
	.align	2
	.global	awlink_asm_clean_transfer_err
	.syntax unified
	.arm
	.type	awlink_asm_clean_transfer_err, %function
awlink_asm_clean_transfer_err:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.pad #16
	sub	sp, sp, #16	@,,
	mov	r3, #2	@ tmp139,
	str	r3, [sp, #8]	@ tmp139, mod_reg_val
	mov	r3, #1	@ tmp140,
	str	r3, [sp, #12]	@ tmp140, mod_reg_addr
	movw	r3, #819	@ tmp141,
	ldr	ip, [sp, #8]	@ D.65654, mod_reg_val
	cmp	ip, r3	@ D.65654, tmp141
	ldreq	r3, [sp, #12]	@ D.65654, mod_reg_addr
	ldrne	r3, [sp, #8]	@ D.65654, mod_reg_val
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r3, [r0, #28]	@ errors, MEM[(volatile u32 *)_4]
@ 0 "" 2
@ 646 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb 
@ 0 "" 2
	.arm
	.syntax unified
	mov	r0, #10	@ tmp143,
	str	r0, [sp]	@ tmp143, mod_reg_val
	mov	r0, #8	@ tmp144,
	str	r0, [sp, #4]	@ tmp144, mod_reg_addr
	ldr	r0, [sp]	@ D.65654, mod_reg_val
	add	ip, r0, #1	@ D.65654, D.65654,
	tst	r0, #8	@ D.65654,
	str	ip, [sp]	@ D.65654, mod_reg_val
	ldreq	ip, [sp]	@ D.65654, mod_reg_val
	ldreq	r0, [sp, #4]	@ D.65654, mod_reg_addr
	muleq	r0, r0, ip	@ D.65654, D.65654, D.65654
	streq	r0, [sp, #4]	@ D.65654, mod_reg_addr
	ldr	r0, [sp]	@ D.65654, mod_reg_val
	and	r0, r0, #8	@ D.65654, D.65654,
	str	r0, [sp, #4]	@ D.65654, mod_reg_addr
	ldr	r0, [sp]	@ D.65654, mod_reg_val
	and	r0, r0, #243	@ D.65654, D.65654,
	str	r0, [sp, #4]	@ D.65654, mod_reg_addr
	ldr	r0, [sp]	@ D.65654, mod_reg_val
	cmp	r0, #10	@ D.65654,
	ldrne	ip, [sp, #4]	@ D.65654, mod_reg_addr
	ldrne	r0, [sp]	@ D.65654, mod_reg_val
	ldreq	r0, [sp, #4]	@ D.65654, mod_reg_addr
	eorne	r0, r0, ip	@ D.65654, D.65654, D.65654
	strne	r0, [sp]	@ D.65654, mod_reg_val
	ldrne	r0, [sp]	@ D.65654, mod_reg_val
	uxtb	r0, r3	@ errors, errors
	ubfx	r3, r3, #16, #8	@ D.65656, errors,,
	strh	r0, [r1]	@ movhi	@ errors, *t_err_9(D)
	strh	r3, [r2]	@ movhi	@ D.65656, *r_err_14(D)
	add	sp, sp, #16	@,,
	@ sp needed	@
	bx	lr	@
	.fnend
	.size	awlink_asm_clean_transfer_err, .-awlink_asm_clean_transfer_err
	.align	2
	.global	awlink_asm_start_xmit
	.syntax unified
	.arm
	.type	awlink_asm_start_xmit, %function
awlink_asm_start_xmit:
	.fnstart
	@ args = 8, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}	@
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	tst	r2, #1073741824	@ id,
	ldr	r8, [r3]	@ dlc, *flag_6(D)
	mov	r6, r3	@ flag, flag
	mov	r5, r0	@ mod_reg_addr, mod_reg_addr
	mov	r4, r2	@ id, id
	ldr	r7, [sp, #40]	@ mod_reg_val, mod_reg_val
	orrne	r3, r8, #64	@ D.65676, dlc,
	ldr	r9, [sp, #44]	@ pdata, pdata
	strne	r3, [r6]	@ D.65676, *flag_6(D)
	cmp	r2, #0	@ id,
	bge	.L77	@,
	ldr	r3, [r6]	@ *flag_6(D), *flag_6(D)
	orr	r3, r3, #128	@ D.65676, *flag_6(D),
	str	r3, [r6]	@ D.65676, *flag_6(D)
	mov	r3, #84	@ tmp195,
	str	r3, [r7]	@ tmp195, *mod_reg_val_17(D)
	.syntax divided
@ 668 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #21, #8	@ D.65674, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #68]	@ D.65674, MEM[(volatile u32 *)_24]
@ 0 "" 2
	.arm
	.syntax unified
	mov	r3, #13	@ tmp197,
	str	r3, [sp]	@ tmp197, mod_reg_val
	mov	r3, #2	@ tmp198,
	str	r3, [sp, #4]	@ tmp198, mod_reg_addr
	ldr	r3, [sp]	@ D.65675, mod_reg_val
	add	r3, r3, #1	@ D.65675, D.65675,
	str	r3, [sp]	@ D.65675, mod_reg_val
	ldr	r2, [sp]	@ D.65675, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65675, mod_reg_addr
	cmp	r2, r3	@ D.65675, D.65675
	ldrlt	r3, [sp]	@ D.65675, mod_reg_val
	ldrlt	r2, [sp, #4]	@ D.65675, mod_reg_addr
	addlt	r3, r3, r3, lsl #1	@ D.65675, D.65675, D.65675,
	addlt	r3, r3, r2	@ D.65675, D.65675, D.65675
	strlt	r3, [sp, #4]	@ D.65675, mod_reg_addr
	ldr	r3, [sp]	@ D.65675, mod_reg_val
	.syntax divided
@ 670 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #13, #8	@ D.65674, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #72]	@ D.65674, MEM[(volatile u32 *)_30]
@ 0 "" 2
@ 671 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #5, #8	@ D.65674, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #76]	@ D.65674, MEM[(volatile u32 *)_35]
@ 0 "" 2
@ 672 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r4, r4, #3	@ D.65674, id,
	bl	arm_heavy_mb	@
	uxtb	r4, r4	@ D.65674, D.65674
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r4, [r5, #80]	@ D.65674, MEM[(volatile u32 *)_40]
@ 0 "" 2
	.arm
	.syntax unified
	b	.L79	@
.L77:
	mov	r3, #76	@ tmp205,
	str	r3, [r7]	@ tmp205, *mod_reg_val_17(D)
	.syntax divided
@ 675 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #3, #8	@ D.65674, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #68]	@ D.65674, MEM[(volatile u32 *)_46]
@ 0 "" 2
@ 676 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r4, r4, #5	@ D.65674, id,
	bl	arm_heavy_mb	@
	uxtb	r4, r4	@ D.65674, D.65674
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r4, [r5, #72]	@ D.65674, MEM[(volatile u32 *)_51]
@ 0 "" 2
	.arm
	.syntax unified
.L79:
	mov	r4, #0	@ i,
.L80:
	cmp	r4, r8	@ i, dlc
	beq	.L86	@,
	.syntax divided
@ 680 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r7]	@ *mod_reg_val_17(D), *mod_reg_val_17(D)
	ldrb	r2, [r9, r4]	@ zero_extendqisi2	@ D.65674, MEM[base: pdata_55(D), index: _87, offset: 0B]
	add	r3, r3, r4, lsl #2	@ D.65674, *mod_reg_val_17(D), i,
	add	r3, r5, r3	@ D.65678, mod_reg_addr, D.65674
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r2, [r3]	@ D.65674, MEM[(volatile u32 *)_63]
@ 0 "" 2
	.arm
	.syntax unified
	add	r4, r4, #1	@ i, i,
	b	.L80	@
.L86:
	.syntax divided
@ 682 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r6]	@ D.65674, *flag_6(D)
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #64]	@ D.65674, MEM[(volatile u32 *)_68]
@ 0 "" 2
	.arm
	.syntax unified
	add	sp, sp, #12	@,,
	@ sp needed	@
	pop	{r4, r5, r6, r7, r8, r9, pc}	@
	.fnend
	.size	awlink_asm_start_xmit, .-awlink_asm_start_xmit
	.align	2
	.global	awlink_asm_rx
	.syntax unified
	.arm
	.type	awlink_asm_rx, %function
awlink_asm_rx:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, lr}	@
	.save {r4, lr}
	.pad #8
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #64]	@ __v, MEM[(volatile u32 *)_3]
@ 0 "" 2
@ 688 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb 
@ 0 "" 2
	.arm
	.syntax unified
	tst	lr, #128	@ __v,
	uxtb	ip, lr	@ fi, __v
	beq	.L88	@,
	mov	lr, #84	@ tmp157,
	str	lr, [r2]	@ tmp157, *mod_reg_val_9(D)
	mov	r2, #11	@ tmp158,
	str	r2, [sp]	@ tmp158, mod_reg_val
	mov	r2, #8	@ tmp159,
	str	r2, [sp, #4]	@ tmp159, mod_reg_addr
	ldr	r2, [sp]	@ D.65689, mod_reg_val
	cmp	r2, #10	@ D.65689,
	beq	.L94	@,
	ldr	lr, [sp]	@ D.65689, mod_reg_val
	ldr	r2, [sp, #4]	@ D.65689, mod_reg_addr
	eor	r2, r2, lr	@ D.65689, D.65689, D.65689
	str	r2, [sp, #4]	@ D.65689, mod_reg_addr
	ldr	r2, [sp]	@ D.65689, mod_reg_val
	cmp	r2, #10	@ D.65689,
	bne	.L91	@,
.L94:
	ldr	r2, [sp, #4]	@ D.65689, mod_reg_addr
	b	.L90	@
.L91:
	ldr	r2, [sp]	@ D.65689, mod_reg_val
	add	r2, r2, #1	@ D.65689, D.65689,
	str	r2, [sp]	@ D.65689, mod_reg_val
	ldr	r2, [sp]	@ D.65689, mod_reg_val
	add	r2, r2, #1	@ D.65689, D.65689,
	str	r2, [sp]	@ D.65689, mod_reg_val
	ldr	r2, [sp]	@ D.65689, mod_reg_val
.L90:
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r4, [r0, #68]	@ __v, MEM[(volatile u32 *)_12]
@ 0 "" 2
@ 692 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb 
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r2, [r0, #72]	@ __v, MEM[(volatile u32 *)_16]
@ 0 "" 2
@ 693 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb 
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #76]	@ __v, MEM[(volatile u32 *)_21]
@ 0 "" 2
@ 694 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb 
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r0, [r0, #80]	@ __v, MEM[(volatile u32 *)_26]
@ 0 "" 2
@ 695 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb 
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r2, r2, #13	@ D.65688, __v,
	ubfx	r0, r0, #3, #5	@ D.65688, __v,,
	orr	r2, r2, r4, lsl #21	@ D.65688, D.65688, __v,
	orr	r2, r2, #-2147483648	@ D.65688, D.65688,
	orr	r2, r2, lr, lsl #5	@ D.65688, D.65688, __v,
	orr	r2, r2, r0	@ D.65688, D.65688, D.65688
	b	.L95	@
.L88:
	mov	lr, #76	@ tmp173,
	str	lr, [r2]	@ tmp173, *mod_reg_val_9(D)
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #68]	@ __v, MEM[(volatile u32 *)_37]
@ 0 "" 2
@ 699 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb 
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r2, [r0, #72]	@ __v, MEM[(volatile u32 *)_40]
@ 0 "" 2
@ 700 "drivers/net/can/sunxi_awlink_asm.c" 1
	dsb 
@ 0 "" 2
	.arm
	.syntax unified
	ubfx	r2, r2, #5, #3	@ D.65688, __v,,
	orr	r2, r2, lr, lsl #3	@ D.65688, D.65688, __v,
.L95:
	str	r2, [r3]	@ D.65688, *id_32(D)
	strb	ip, [r1]	@ fi, *r_fi_47(D)
	add	sp, sp, #8	@,,
	@ sp needed	@
	pop	{r4, pc}	@
	.fnend
	.size	awlink_asm_rx, .-awlink_asm_rx
	.align	2
	.global	awlink_asm_probe
	.syntax unified
	.arm
	.type	awlink_asm_probe, %function
awlink_asm_probe:
	.fnstart
	@ args = 0, pretend = 0, frame = 112
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}	@
	.save {r4, r5, r6, r7, lr}
	.pad #140
	sub	sp, sp, #140	@,,
	add	r2, sp, #136	@ tmp149,,
	mvn	r3, #0	@ tmp145,
	mov	r7, r0	@ node, node
	mov	r4, r1	@ num, num
	str	r3, [r2, #-88]!	@ tmp145, res.a0
	mov	r3, #0	@ tmp148,
	str	r2, [sp, #16]	@ tmp149,
	mov	r2, r1	@, num
	str	r3, [sp, #20]	@ tmp148,
	str	r3, [sp, #12]	@ tmp148,
	str	r3, [sp, #8]	@ tmp148,
	str	r3, [sp, #4]	@ tmp148,
	str	r3, [sp]	@ tmp148,
	ldr	r1, .L112	@,
	ldr	r0, .L112+4	@,
	bl	__arm_smccc_smc	@
	ldr	r6, [sp, #48]	@ res.a0, res.a0
	cmp	r6, #0	@ res.a0,
	beq	.L97	@,
.L99:
	mvn	r0, #0	@ D.65708,
	b	.L98	@
.L97:
	mov	r2, r6	@, res.a0
	ldr	r1, .L112+8	@,
	mov	r0, r7	@, node
	bl	of_get_property	@
	mov	r5, r0	@ be_addr,
	mov	r0, r7	@, node
	bl	of_irq_find_parent	@
	cmp	r0, #0	@ p
	beq	.L99	@,
	mov	r3, #3	@ tmp156,
	str	r0, [sp, #64]	@ p, oirq.np
	str	r3, [sp, #68]	@ tmp156, oirq.args_count
	mov	r3, #11	@ tmp158,
	str	r3, [sp, #40]	@ tmp158, mod_reg_val
	mov	r3, #13	@ tmp159,
	str	r3, [sp, #44]	@ tmp159, mod_reg_addr
	ldr	r3, [sp, #40]	@ D.65708, mod_reg_val
	str	r6, [sp, #72]	@ res.a0, oirq.args
	and	r3, r3, #243	@ D.65708, D.65708,
	str	r3, [sp, #44]	@ D.65708, mod_reg_addr
	ldr	r3, [sp, #40]	@ D.65708, mod_reg_val
	add	r3, r3, #1	@ D.65708, D.65708,
	str	r3, [sp, #40]	@ D.65708, mod_reg_val
	ldr	r3, [sp, #44]	@ D.65708, mod_reg_addr
	add	r3, r3, #51	@ D.65708, D.65708,
	str	r3, [sp, #44]	@ D.65708, mod_reg_addr
	ldr	r3, [sp, #44]	@ D.65708, mod_reg_addr
	cmp	r3, #10	@ D.65708,
	beq	.L111	@,
	ldr	r3, [sp, #40]	@ D.65708, mod_reg_val
	cmp	r3, #2	@ D.65708,
	beq	.L111	@,
	ldr	r3, [sp, #40]	@ D.65708, mod_reg_val
	cmp	r3, #2	@ D.65708,
	bne	.L103	@,
.L111:
	ldr	r3, [sp, #44]	@ D.65708, mod_reg_addr
	b	.L101	@
.L103:
	ldr	r2, [sp, #44]	@ D.65708, mod_reg_addr
	ldr	r3, [sp, #40]	@ D.65708, mod_reg_val
	eor	r3, r3, r2	@ D.65708, D.65708, D.65708
	str	r3, [sp, #40]	@ D.65708, mod_reg_val
	ldr	r3, [sp, #40]	@ D.65708, mod_reg_val
.L101:
	mov	r3, #13	@ tmp161,
	add	r4, r4, #21	@ mod_reg_val, num,
	str	r3, [sp, #32]	@ tmp161, mod_reg_val
	mov	r3, #8	@ tmp162,
	str	r3, [sp, #36]	@ tmp162, mod_reg_addr
	add	r1, sp, #64	@ tmp171,,
	ldr	r2, [sp, #36]	@ D.65708, mod_reg_addr
	mov	r0, r5	@, be_addr
	ldr	r3, [sp, #32]	@ D.65708, mod_reg_val
	str	r4, [sp, #76]	@ mod_reg_val, oirq.args
	eor	r3, r3, r2	@ D.65708, D.65708, D.65708
	str	r3, [sp, #32]	@ D.65708, mod_reg_val
	ldr	r3, [sp, #32]	@ D.65708, mod_reg_val
	mov	r3, #4	@ tmp163,
	str	r3, [sp, #80]	@ tmp163, oirq.args
	bl	of_irq_parse_raw	@
	mov	r3, #15	@ tmp165,
	add	r0, sp, #64	@ tmp172,,
	str	r3, [sp, #24]	@ tmp165, mod_reg_val
	mov	r3, #12	@ tmp166,
	str	r3, [sp, #28]	@ tmp166, mod_reg_addr
	movw	r3, #819	@ tmp167,
	ldr	r2, [sp, #24]	@ D.65708, mod_reg_val
	cmp	r2, r3	@ D.65708, tmp167
	ldreq	r3, [sp, #28]	@ D.65708, mod_reg_addr
	ldrne	r3, [sp, #24]	@ D.65708, mod_reg_val
	bl	irq_create_of_mapping	@
.L98:
	add	sp, sp, #140	@,,
	@ sp needed	@
	pop	{r4, r5, r6, r7, pc}	@
.L113:
	.align	2
.L112:
	.word	-65534
	.word	-1308622748
	.word	.LC0
	.fnend
	.size	awlink_asm_probe, .-awlink_asm_probe
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"reg\000"
	.ident	"GCC: (Linaro GCC 5.3-2016.05) 5.3.1 20160412"
	.section	.note.GNU-stack,"",%progbits
