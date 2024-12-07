@echo off

:: Chạy CMake để cấu hình dự án
cmake -S . -B ./build -G "MinGW Makefiles"

:: Build chương trình bằng make hoặc cmake --build .
mingw32-make -C build

:: Chạy chương trình sau khi đã biên dịch xong
@REM .\build\MyCppProject.exe

:: Pause chương trình lại, chờ đến khi người dùng nhấn phím để thoát chương trình 
pause

:: Chạy chương trình trên terminal bằng câu lệnh .\Build_and_run.bat
