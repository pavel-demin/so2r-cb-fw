ENTRY(Reset_Handler)

_data_flash_start = LOADADDR(.data);

_stack_end = ORIGIN(ram) + LENGTH(ram);

MEMORY {
  flash (rx) : o = 0x08000000, l = 64k
  ram (wx) : o = 0x20000000, l = 32k
}

SECTIONS {
  .text : {
    . = ALIGN(4);
    KEEP(* (.vector_table))
    * (.text*)
    * (.rodata*)
    . = ALIGN(4);
  } >flash

  .data : {
    . = ALIGN(4);
    _data_start = .;
    * (.data*)
    . = ALIGN(4);
    _data_end = .;
  } >ram AT>flash

  .bss : {
    . = ALIGN(4);
    _bss_start = .;
    * (.bss*)
    * (COMMON)
    . = ALIGN(4);
    _bss_end = .;
  } >ram
}
