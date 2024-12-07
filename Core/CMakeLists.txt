# Tạo thư viện bằng các mã nguồn bên trong thư mục Core
add_library(
        Lib_Core 
        OBJECT 
        Src/stm32f1xx_it.c
        Src/stm32f1xx_hal_msp.c
        Src/syscalls.c
        Src/sysmem.c
        Src/system_stm32f1xx.c
)

# Thêm các thư mục header cần thiết vào thư viện Lib_Core
target_include_directories(
    Lib_Core 
    PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32f1xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/STM32f1xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32f1xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# target_compile_definitions(Lib_Core PRIVATE 
# 	USE_HAL_DRIVER 
# 	STM32f107xx
# )