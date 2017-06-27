/* Linker script to configure memory regions. */
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

/*		_
flash map in in_flash_map.h
*/

MEMORY
{
  FLASH (rx) : ORIGIN = 0x17000, LENGTH = 0x19000
  RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x6000 /* 24 kB, 8 kB is taken by S110. */
}
INCLUDE "nrf51822_sdk/linker/gcc_nrf51_common.ld"
