# Đặt phiên bản tối thiểu của CMake yêu cầu
cmake_minimum_required(VERSION 3.22)



# Include init file
include("cmake_config/cmake_init.cmake")



# Include config compiler file
include("cmake_config/gcc-arm-none-eabi.cmake")



# Định nghĩa các macro chung cho toàn bộ chương trình ngay từ lúc được thêm vào (nên thêm vào đầu chương trình),ngược lại ta có target_compile_definitions áp dụng cho target cụ thể
add_definitions(-DUSE_HAL_DRIVER -DSTM32F103xB)  



# Định nghĩa màu sắc in ra màn hình khi bắt đầu chạy CMake
print_complete_message("--------------------------------------------- CMAKE STARTED RUN ---------------------------------------------")



# Đặt tên cho project
project(MyCProject)



# Cung cấp các định nghĩa macro cho target chính của dự án 
# target_compile_definitions(MyCProject PRIVATE 
# 	USE_HAL_DRIVER 
# 	STM32F407xx
# )



# Thêm các tùy chọn biên dịch đặc biệt
# target_compile_options(MyCProject PRIVATE -Wall -Wextra -O2)



# Thêm các thư mục chứa các file header cho toàn bộ dự án, lệnh này không cần target
include_directories(Core/Inc)
include_directories(Drivers/CMSIS/Include)
include_directories(Drivers/CMSIS/Device/ST/STM32F1xx/Include)
include_directories(Drivers/STM32F1xx_HAL_Driver/Inc)
include_directories(Drivers/STM32F1xx_HAL_Driver/Inc/Legacy)



# Thêm các thư mục con và xử lí các cấu trúc CMakeList bên trong thư mục con đó, thường phải đi kèm với include_directories hoặc target_include_directories, nếu không các thư viện hoặc executable được tạo ra không để làm gì cả   
add_subdirectory(Core)
add_subdirectory(Drivers)
add_subdirectory(mcu_config_init)



# Biến chứa tên các file source C ở thư mục hiện hành
set(LIST_NAME_SOURCE_C "${CMAKE_SOURCE_DIR}/Core/Src/main.c")



# Thu thập tất cả các tệp .c trong thư mục hiện tại không bao gồm các thư mục con (nếu muốn tìm trong các thư mục con dùng GLOB_RECURSE)
# file(GLOB PATH_SOURCE_FILES "${CMAKE_SOURCE_DIR}/*.c")



# # Lặp lại qua từng tệp .c để lấy tên của các file source ở thư mục hiện tại
# foreach(PATH_SOURCE ${PATH_SOURCE_FILES})
#     # Lấy tên tệp có phần mở rộng với keyword là NAME (ví dụ: main.c hoặc foo.c)
#     get_filename_component(FILENAME ${PATH_SOURCE} NAME)
#     # Thêm tên file source vào danh sách
#     list(APPEND LIST_NAME_SOURCE_C ${FILENAME})
#     # Hiện thông tin tên file tìm thấy
#     print_infor_message("[CMakeList_In_CMAKE_GENERATE_PROJECT]: Found Sources Code ---> ${FILENAME}")
# endforeach()



# Thêm tệp nguồn vào file thực thi project
add_executable(MyCProject ${LIST_NAME_SOURCE_C})



# Liên kết thư viện với target chính chính của dự án
target_link_libraries(MyCProject Lib_Core)
target_link_libraries(MyCProject Lib_Drivers)
target_link_libraries(MyCProject Lib_mcu_config_init)



# Liên kết target chính với một thư mục header cụ thể
# target_include_directories(MyCProject PRIVATE Lib_funtion)



# Lặp lại qua từng tệp .c để tạo tệp .i tương ứng
# foreach(SOURCE_FILE ${PATH_SOURCE_FILES})
#     # Lấy tên tệp không có phần mở rộng với keyword NAME_WE (ví dụ: main hoặc foo)
#     get_filename_component(FILENAME_WE ${SOURCE_FILE} NAME_WE)
    
#     # Gọi add_custom_command cho từng tệp nguồn, add_custom_command sẽ add các lệnh custom để xử lí cho từng tệp tương ứng
#     add_custom_command(
#         OUTPUT ${CMAKE_BINARY_DIR}/${FILENAME_WE}.i
#         COMMAND ${CMAKE_CXX_COMPILER} -E ${SOURCE_FILE} -o ${CMAKE_BINARY_DIR}/${FILENAME_WE}.i 
#                 -I${CMAKE_SOURCE_DIR}/funtion_1 -I${CMAKE_SOURCE_DIR}/funtion_1/Header_Inside1 
#                 -I${CMAKE_SOURCE_DIR}/funtion_2 -I${CMAKE_SOURCE_DIR}/funtion_2/Header_Inside2
#                 -I${CMAKE_SOURCE_DIR}/funtion_3
#                 -I${CMAKE_SOURCE_DIR}/Lib_funtion
#         DEPENDS ${SOURCE_FILE}
#         COMMENT "Generating preprocessed file ${FILENAME_WE}.i"
#     )
    
#     # Tạo target tùy chỉnh cho mỗi file source, được hiểu là để xuất file .i cho từng file source
#     add_custom_target(
#         ${FILENAME_WE}_preprocess ALL
#         DEPENDS ${CMAKE_BINARY_DIR}/${FILENAME_WE}.i
#     )

#     # Đảm bảo rằng quá trình build của MyCProject sẽ phụ thuộc vào việc tạo các file .i, do đó "quá trình tiền xử lý" sẽ được thực hiện "trước" khi biên dịch
#     add_dependencies(MyCProject ${FILENAME_WE}_preprocess)
# endforeach()
