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
    *(.rodata)		
	. = ALIGN(4); /*the current address align for 4 bytes*/
    _etext = .; /* compute 'end_of_text' size*/
  } > FLASH	

  _la_data = LOADADDR(.data);	
  .data :
  {
    _sdata = .;
	*(.data)
	. = ALIGN(4);
	_edata = .;
  }> SRAM AT> FLASH
  
    .bss :
  {
    _sbss = .;
	*(.bss)
	*(COMMON) /*Store the COMMON section into bss section for global variable without initialize*/
	. = ALIGN(4);
    _ebss = .;
  } > SRAM
}