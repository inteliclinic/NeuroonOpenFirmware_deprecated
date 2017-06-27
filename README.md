# NeuroonOpen_deprecated
Deprecated version of NeuroonOpen

## Why?

This code is based on _old_ version of Neuroon firmware(2.0.40.4). __*NeuroonOpen*__ is vastly different, shares little functionality with his predecessor. We decided not to update it anymore and build new version from the scratch.
NeuroonOpen is based on the newest Bluetooth stack from Nordic Semiconductor and their SDK.

## Requirements

* arm-none-eabi-gcc with *nanolib* support under arm-none-eabi-gcc command
* make tool
* openocd version >= 0.9.0
* stlink or jlink software (Makefile supports stlink at the moment - will add jlink also)

## Instructions

* __make *release*__ - build code using release preset
* __make *debug*__ - build code using debug preset
* __make *flash_app*__ - program neuroon with neuroon firmware(no stack, dfu and bootloader)
* __make *flash_all*__ - program neuroon all binaries (including stack, dfu and bootloader)

## Contribution

[TODO] Give us a little time.

## Contact info

Paweł Kaźmierzewski - p.kazmierzewski@inteliclinic.com

Wojtek Węclewski - w.weclewski@inteliclinic.com
