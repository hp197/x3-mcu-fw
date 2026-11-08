/* AT32F415-class MCU (motor-controller) application: slot 0 of flash @ 0x08001000 (60 KB),
   32 KB SRAM. The x3 bootloader launches this image; its vector table sits at 0x08001000.
   Compatible with Artery's startup_at32f415.s (expects _estack, _sidata, _sdata/_edata,
   _sbss/_ebss). Mirrors the x3-vcu-fw linker, with the simpler MCU slot-0 header
   (just the banner; no OTA key / build-id cells). */
_estack = 0x20008000;      /* top of 32 KB SRAM */

MEMORY
{
    /* Slot 0, 0x08001000-0x0800FFFF. Slot 1 above it holds the OTA staging copy. */
    FLASH (rx)  : ORIGIN = 0x08001000, LENGTH = 60K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

/* Reset_Handler is the gc-sections root; without it the linker never pulls the startup object
   (vector table + Reset_Handler) out of libcmsis.a. */
ENTRY(Reset_Handler)

SECTIONS
{
    /* Vector table first in slot 0: the bootloader reads the initial SP and Reset_Handler at the
       image start, and main sets VTOR to it. VTOR needs the table aligned to its vector count
       rounded up to a power of two: 92 -> 128 vectors, 512 bytes. */
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    ASSERT(ADDR(.isr_vector) == ORIGIN(FLASH), "vector table must be first in slot 0")
    ASSERT(ADDR(.isr_vector) % 512 == 0, "VTOR needs 512-byte alignment (92 vectors)")

    /* Code between the vector table and the slot-0 banner at 0x08001400: whole objects that fit
       the 656-byte gap. If they grow too large the .app_header address below overlaps and the
       link fails. */
    .text_low : {
        *runtime.o(.text*)
    } > FLASH

    /* Slot-0 header (config.h APP_HEADER_SECTION): the 16-byte banner at 0x08001400. The MCU
       header is just the banner; application code follows immediately after it. */
    .app_header 0x08001400 : {
        KEEP(*(.app_header.banner))
        . = ALIGN(4);
    } > FLASH
    ASSERT(ADDR(.app_header) == 0x08001400, "slot-0 banner must sit at 0x08001400")

    .text : {
        *(.text*) *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    _sidata = LOADADDR(.data);

    .data : {
        . = ALIGN(4); _sdata = .;
        *(.data*)
        . = ALIGN(4); _edata = .;
    } > RAM AT > FLASH

    .bss : {
        . = ALIGN(4); _sbss = .; __bss_start__ = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4); _ebss = .; __bss_end__ = .;
    } > RAM

    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) *(.init) *(.fini) }
}
