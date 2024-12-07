# Định nghĩa macro in ra thông tin hợp lệ hoặc thành công
macro(print_complete_message message)
        execute_process(COMMAND ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
                                ${CMAKE_COMMAND} -E cmake_echo_color  --green --bold "${message}")
endmacro()

# Định nghĩa macro in ra thông tin thông thường
macro(print_infor_message message)
        execute_process(COMMAND ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
                                ${CMAKE_COMMAND} -E cmake_echo_color --blue --bold "${message}")
endmacro()

# Định nghĩa macro in ra thông tin khi bị lỗi
macro(print_error_message message)
        execute_process(COMMAND ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
                                ${CMAKE_COMMAND} -E cmake_echo_color --red --bold "${message}")
endmacro()

# Định nghĩa macro in ra thông tin cảnh báo
macro(print_warning_message message)
        execute_process(COMMAND ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
                                ${CMAKE_COMMAND} -E cmake_echo_color --yellow --bold "${message}")
endmacro()