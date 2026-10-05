/*
 * STM32G431RBTx GCC 链接脚本
 *
 * 由 Keil 的 MDK-ARM/AxDr/AxDr.sct 转换而来:
 *     Flash : 0x08000000, 128 KB (0x20000)
 *     RAM   : 0x20000000,  22 KB (0x5800)   SRAM1 + SRAM2
 *
 * 关于 _RAM_FUNC:
 *     common.h 里 _RAM_FUNC = __attribute__((section(".RamFunc")))
 *     但原来的 Keil scatter 并没有给 .RamFunc 分配 RAM 执行区,
 *     所以那些函数实际还是从 Flash 运行的(等价于普通函数)。
 *     这里保持一致: 把 .RamFunc 也放进 Flash。
 *     将来若确实想让它们跑在 RAM, 需要改成
 *         .ramfunc : { *(.RamFunc*) } >RAM AT> FLASH
 *     并在 Reset_Handler 里加一段从 Flash 拷贝到 RAM 的代码。
 */

ENTRY(Reset_Handler)

/* 栈顶: RAM 最高地址 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* 堆/栈最小保留量 (对应 .ioc 里 HeapSize=0x200 / StackSize=0x400) */
_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 22K
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 10K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
}

SECTIONS
{
  /* 中断向量表必须放在 Flash 最前面 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* _RAM_FUNC 段 —— 保持与 Keil 相同, 放在 Flash */
  .RamFunc :
  {
    . = ALIGN(4);
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* .data 的初值存放在 Flash, 运行时由 Reset_Handler 搬到 RAM */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 堆栈保留区: 同时用于链接时检查 RAM 是否够用 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
