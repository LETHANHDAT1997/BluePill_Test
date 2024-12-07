# Thiết lập hệ thống và bộ vi xử lý mục tiêu
set(CMAKE_SYSTEM_NAME               Generic)           
set(CMAKE_SYSTEM_PROCESSOR          arm)                



# Cấu hình trình biên dịch và các thông số liên quan
set(CMAKE_C_COMPILER_FORCED     TRUE)                  
set(CMAKE_CXX_COMPILER_FORCED   TRUE)                   
set(CMAKE_C_COMPILER_ID         GNU)                    
set(CMAKE_CXX_COMPILER_ID       GNU)                    



# Xuất lệnh biên dịch để dễ dàng lập chỉ mục với các công cụ như clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)



# Chỉ định đường dẫn đến file Linker
set(PATH_LINKER_FILE "${CMAKE_SOURCE_DIR}/mcu_config_init/STM32F103C8Tx_FLASH.ld")



# Đặt các trình biên dịch cho C, C++, và các công cụ liên quan đến ARM
set(CMAKE_C_COMPILER                "/home/ledat/Public/Arm_GNU_Cross_Compiler/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER              "/home/ledat/Public/Arm_GNU_Cross_Compiler/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER              "/home/ledat/Public/Arm_GNU_Cross_Compiler/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-g++")
set(CMAKE_LINKER                    "/home/ledat/Public/Arm_GNU_Cross_Compiler/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-g++")
set(CMAKE_OBJCOPY                   "/home/ledat/Public/Arm_GNU_Cross_Compiler/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-objcopy")
set(CMAKE_OBJDUMP                   "/home/ledat/Public/Arm_GNU_Cross_Compiler/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-objdump")
set(CMAKE_SIZE                      "/home/ledat/Public/Arm_GNU_Cross_Compiler/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-size")
set(CMAKE_READELF                   "/home/ledat/Public/Arm_GNU_Cross_Compiler/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-readelf")



# Chỉ định tiêu chuẩn C là C11
set(CMAKE_C_STANDARD 11)     
# Bắt buộc sử dụng tiêu chuẩn C đã chỉ định         
set(CMAKE_C_STANDARD_REQUIRED ON)     
# Cho phép sử dụng các phần mở rộng của trình biên dịch nếu có (ví dụ GNU extensions)
set(CMAKE_C_EXTENSIONS ON)            



# Đặt tiêu chuẩn C++ thành C++17
set(CMAKE_CXX_STANDARD 17)           
# Bắt buộc sử dụng tiêu chuẩn C++ đã chỉ định  
set(CMAKE_CXX_STANDARD_REQUIRED ON)    
# Cho phép sử dụng các phần mở rộng của trình biên dịch nếu có
set(CMAKE_CXX_EXTENSIONS ON)           



# Thiết lập chế độ biên dịch Debug để dễ dàng gỡ lỗi với GDB
set(CMAKE_BUILD_TYPE Debug)
# Đảm bảo rằng cờ "-g" luôn được thêm vào khi biên dịch trong chế độ Debug
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g") # Thêm cờ debug cho C



# Kích hoạt hỗ trợ cho các ngôn ngữ ASM, C và C++
enable_language(C ASM)



# Thiết lập phần mở rộng của tệp thực thi cho từng loại ngôn ngữ
set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")  
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")  
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")  



# Đặt loại tệp thử nghiệm khi CMake kiểm tra khả năng biên dịch, thiết lập kiểu thử nghiệm là thư viện tĩnh
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)  



# ------------------------------------------------------------------------------------------ Cờ biên dịch cho C ------------------------------------------------------------------------------------------
# Cờ cho bộ vi xử lý cụ thể (MCU) - ví dụ, Cortex-M4
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m3 ")     
# Cờ cảnh báo và tối ưu hóa      
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")



# Kiểm tra nếu build type là Debug
if(CMAKE_BUILD_TYPE MATCHES Release)
    # Debug, không tối ưu, thêm thông tin debug
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g3")  
endif()



# Kiểm tra nếu build type là Release
if(CMAKE_BUILD_TYPE MATCHES Release)
    # Release, tối ưu không gian, không debug
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -g0")  
endif()



# ------------------------------------------------------------------------------------------ Cờ biên dịch cho ASM ------------------------------------------------------------------------------------------
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")         



# ------------------------------------------------------------------------------------------ Cờ biên dịch cho C++ ------------------------------------------------------------------------------------------
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")



# ------------------------------------------------------------------------------------------ Cờ liên kết cho C ------------------------------------------------------------------------------------------
set(CMAKE_C_LINK_FLAGS "-mcpu=cortex-m3 ")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T ${PATH_LINKER_FILE}")  
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")   
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")  
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")        
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")               



# ------------------------------------------------------------------------------------------ Cờ liên kết cho C++ ------------------------------------------------------------------------------------------
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")
