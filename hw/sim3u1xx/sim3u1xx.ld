MEMORY
{
    sret (W!RX) : ORIGIN = 0x20000000, LENGTH = 0x0020
    sram (W!RX) : ORIGIN = 0x20000020, LENGTH = 0x7FDF
    flash (RX) : ORIGIN = 0x00000000, LENGTH = 0x3fffc
}

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        _text = .;
        PROVIDE(stext = .);
        KEEP(*(.isr_vector))
        KEEP(*(.init))

        /* For SiM3 Startup Code */
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data)) ;
        LONG(  SIZEOF(.data));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        . = ALIGN(4);
        _etext = .;
        _sidata = _etext;
        PROVIDE(etext = .);
            _fini = . ;
                *(.fini)

    } >flash

    .sret (NOLOAD) : {
        . = ALIGN(4);
        _ssret = .;
        *(.sret .sret.*)
        . = ALIGN(4);
        _esret = .;
    } >sret

    .data : AT (_etext)
    {
        . = ALIGN(4);
        _sdata = .;
        *(.ramfunc .ramfunc.* .fastrun .fastrun.*)
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        . = ALIGN(4);
        _edata = .;
    } >sram

        .ARM.extab :
        {
            *(.ARM.extab*)
        } >sram

        __exidx_start = .;
        .ARM.exidx :
        {
            *(.ARM.exidx*)
        } >sram
        __exidx_end = .;

    PROVIDE( flash_used_size = SIZEOF(.text) + SIZEOF(.data) + SIZEOF(.ARM.extab) + SIZEOF(.ARM.exidx) );

    .bss (NOLOAD) : {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >sram

    end = .;
    PROVIDE( _estack = 0x20008000 );
    PROVIDE(_vStackTop = 0x20008000);

}

