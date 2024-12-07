# Tạo thư viện bằng các mã nguồn bên trong thư mục Drivers
add_library(
        Lib_Drivers
        OBJECT
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
        STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
)

# Thêm các thư mục header cần thiết vào thư viện Lib_Drivers
target_include_directories(
    Lib_Drivers 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32F1xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32F1xx_HAL_Driver/Inc/Legacy
)

# target_compile_definitions(Lib_Drivers PRIVATE 
# 	USE_HAL_DRIVER 
# 	STM32F407xx
# )