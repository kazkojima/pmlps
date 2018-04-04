STM32CUBEEXPANSION_53L1A1 = symlink-STM32CubeExpansion_53L1A1_V1.0.0

COMPONENT_SRCDIRS := . \
        $(STM32CUBEEXPANSION_53L1A1)/Drivers/BSP/Components/vl53l1x

COMPONENT_ADD_INCLUDEDIRS = \
        $(STM32CUBEEXPANSION_53L1A1)/Drivers/BSP/X-NUCLEO-53L1A1 \
        $(STM32CUBEEXPANSION_53L1A1)/Drivers/BSP/Components/vl53l1x \
        $(STM32CUBEEXPANSION_53L1A1)/Projects/Multi/Examples/VL53L1X/SimpleRangingExamples/Inc

CFLAGS += -Wno-unused-variable -Wno-maybe-uninitialized -DI2C_HandleTypeDef=int
