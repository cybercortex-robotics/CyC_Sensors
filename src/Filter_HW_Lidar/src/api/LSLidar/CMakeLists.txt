add_library(lslidar
    include/lslidar.hpp
    src/lslidar.cpp
    src/serial.hpp
    src/serial.cpp
    src/serial_unix.hpp
    src/serial_unix.cpp
    src/serial_win.hpp
    src/serial_win.cpp
)

target_include_directories(lslidar
    PRIVATE
        src
    PUBLIC
        include)

target_link_libraries(lslidar PUBLIC CycToolkit)
