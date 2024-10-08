add_library(openrisc openrisc/or1k.c openrisc/or1k_du_adv.c openrisc/or1k_tap_mohor.c openrisc/or1k_tap_vjtag.c openrisc/or1k_tap_xilinx_bscan.c openrisc/jsp_server.c)

#add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/xscale_debug.inc COMMAND ${SH_WRAPPER} ARGS -c "${CMAKE_CURRENT_SOURCE_DIR}/../helper/bin2char.sh < ${CMAKE_CURRENT_SOURCE_DIR}/xscale/debug_handler.bin > ${CMAKE_CURRENT_BINARY_DIR}/xscale_debug.inc" MAIN_DEPENDENCY xscale/debug_handler.bin)

add_library(target
	adi_v5_jtag.c
	adi_v5_swd.c
	algorithm.c
	arm11.c
	arm11_dbgtap.c
	arm720t.c
	arm7tdmi.c
	arm7_9_common.c
	arm920t.c
	arm926ejs.c
	arm946e.c
	arm966e.c
	arm9tdmi.c
	armv4_5.c
	armv4_5_cache.c
	armv4_5_mmu.c
	armv7a.c
	armv7a_cache.c
	armv7a_cache_l2x.c
	armv7m.c
	armv7m_trace.c
	armv8.c
	armv8_cache.c
	armv8_dpm.c
	armv8_opcodes.c
	arm_adi_v5.c
	arm_cti.c
	arm_disassembler.c
	arm_dpm.c
	arm_jtag.c
	arm_semihosting.c
	arm_simulator.c
	arm_tpiu_swo.c
	avr32_ap7k.c
	avr32_jtag.c
	avr32_mem.c
	avr32_regs.c
	avrt.c
	breakpoints.c
	cortex_a.c
	cortex_m.c
	dsp563xx.c
	dsp563xx_once.c
	dsp5680xx.c
	embeddedice.c
	etb.c
	etm.c
	etm_dummy.c
	fa526.c
	feroceon.c
	hla_target${HLA_MULTICORE_SUFFIX}.c
	image.c
	lakemont.c
	ls1_sap.c
	mips32.c
	mips32_dmaacc.c
	mips32_pracc.c
	mips_ejtag.c
	mips_m4k.c
	quark_d20xx.c
	quark_x10xx.c
	register.c
	rtt.c
	smp.c
	target.c
	target_request.c
	testee.c
	trace.c
	x86_32_common.c
	xscale.c
	aarch64.c
	a64_disassembler.c
	stm8${HLA_MULTICORE_SUFFIX}.c
	arm_dap.c
	semihosting_common.c
	riscv/batch.c
	riscv/program.c
	riscv/riscv.c
	riscv/riscv_semihosting.c
	riscv/riscv-011.c
	riscv/riscv-013.c
	mem_ap.c
	esirisc.c
	esirisc_jtag.c
	armv7a_mmu.c
	esirisc_trace.c
	mips_mips64.c
	mips64.c
	mips64_pracc.c
	adi_v5_dapdirect.c
	arc.c
	arc_cmd.c
	arc_jtag.c
	arc_mem.c
	espressif/esp_semihosting.c
	espressif/esp_xtensa.c
	espressif/esp_xtensa_semihosting.c
	espressif/esp_xtensa_smp.c
	espressif/esp32.c
	espressif/esp32s2.c
	espressif/esp32s3.c
	xtensa/xtensa.c
	xtensa/xtensa_chip.c
	xtensa/xtensa_debug_module.c
	espressif/esp_xtensa_apptrace.c
	espressif/esp32_apptrace.c
	espressif/esp32_sysview.c
	xtensa/xtensa_fileio.c
	espressif/esp.c
	espressif/esp_algorithm.c
	espressif/esp_xtensa_algorithm.c)
	
SET_TARGET_PROPERTIES(target PROPERTIES COMPILE_FLAGS -I${CMAKE_CURRENT_BINARY_DIR})