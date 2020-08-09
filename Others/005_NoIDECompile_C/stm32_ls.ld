ENTRY(Reset_Handler)

MEMORY
{
  FLASH(rx):ORIGIN =0x08000000, LENGTH = 512K
  SRAM(rwx):ORIGIN =0x20000000, LENGTH = 128K
}

SECTIONS
{
  /*.text section of all input files*/
  /* rom data*/
  /*location is flast*/
  .text : 
  {
    *(.isr_vector)		
    *(.text) 		
	*(.text.*)
	*(.init)
	*(.fini)
    *(.rodata)		
	*(.rodata.*)
	. = ALIGN(4); /*the current address align for 4 bytes*/
    _etext = .; /* compute 'end_of_text' size*/
  } > FLASH	

  _la_data = LOADADDR(.data);	
  .data :
  {
    _sdata = .;
	*(.data)
	*(.data.*)
	. = ALIGN(4);
	_edata = .;
  }> SRAM AT> FLASH
  
    .bss :
  {
    _sbss = .;
	__bss_start__ = _sbss;
	*(.bss)
	*(.bss.*)
	*(COMMON) /*Store the COMMON section into bss section for global variable without initialize*/
	. = ALIGN(4);
    _ebss = .;
	__bss_end__ = _ebss;
	. = ALIGN(4);
	end = .;	
  } > SRAM
}