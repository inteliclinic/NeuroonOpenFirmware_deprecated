/* Linker script to configure memory regions. */
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

/*		_
flash map in in_flash_map.h
*/

MEMORY
{
  FLASH (rx) : ORIGIN = 0x30000, LENGTH = 0x08000
  RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8 kB, 8 kB is taken by S110. */
}
INCLUDE "gcc_nrf51_common.ld"
