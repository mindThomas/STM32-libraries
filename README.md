# STM32-libraries
Collection of RTOS aware low-level (HAL) and high-level libraries (drivers) for the STM32 device family

## Layer Hierarchy
From low-level to high-level (high numbers = high abstraction)
1. `Periphirals` : periphiral (HAL) libraries
2. `Drivers` : component specific driver libraries
3. `Devices` : high-level device abstraction layer, mostly containing templates used by component drivers
4. `Modules` : interconnecting modules utilizing and combining devices or drivers
5. `Misc` : platform independent algorithms and libraries