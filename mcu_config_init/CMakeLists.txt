
# Tạo thư viện bằng các mã nguồn bên trong thư mục mcu_config_init
add_library(
        Lib_mcu_config_init
        OBJECT
        startup_stm32f103xb.s
)

target_link_libraries(Lib_mcu_config_init PRIVATE Lib_Core)

# target_include_directories(
#     Lib_mcu_config_init 
#     PRIVATE
#     ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
#     ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
#     ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
#     ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
#     ${CMAKE_SOURCE_DIR}/Core/Inc
# )

# target_compile_definitions(Lib_mcu_config_init PRIVATE 
# 	USE_HAL_DRIVER 
# 	STM32F407xx
# )
