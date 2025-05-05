ENTRY(0)

MEMORY {
  RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 4M
  /* FLASH (rwx) : ORIGIN = 0x80000000, LENGTH = 4M */
}

SECTIONS {
    .exception_vectors : ALIGN(32) {
      KEEP(*(.exception_vectors));
    } > RAM

    .text : ALIGN(4) {
      *(.text*)
      *(.rodata*)
    } > RAM

    .data : {
      *(.data*)
    } > RAM

    .bss.stack ALIGN(8) : 
    {
		KEEP(*(.bss.stack))
        . += 16384;
    __supervisor_stack_stop = .;
        . += 16384;
    __undefined_stack_stop = .;
        . += 16384;
    __fiq_stack_stop = .;
        . += 16384;
    __irq_stack_stop = .;
        . += 16384;
    __abort_stack_stop = .;
        . += 16384;
    __system_stack_stop = .;
        . += 16384;
    __secure_stack_stop = .;
        . += 16384;
    __user_stack_stop = .;
	} > RAM
}