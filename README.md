# serialcom
-----------

The serialcom library provides accessing serial ports on LINUX systems with real time RTAI compatibility. This library was think to be compatible with process with multiples threads and allows that multiples threads can access the same serial port each time. In that case, these functionalities are implemented to COM1, COM2, COM3 and COM4. Also, this library implements functions to access by semaphores, which allows the same serial port to be access by multiples threads per once.


# How to use


    cmake_minimum_required(VERSION 2.8)
    project( serial )
    # find_package( OpenCV REQUIRED )

    include(ExternalProject)
    ExternalProject_Add(serialcom
        GIT_REPOSITORY https://github.com/Gastd/serialcom
        GIT_TAG master
        # PREFIX ${CMAKE_CURRENT_BINARY_DIR}/serialcom
        SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/serialcom-src"
        BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/build"
        # CONFIGURE_COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" "${CMAKE_BINARY_DIR}/serialcom-src/"
        BUILD_COMMAND "${CMAKE_COMMAND}" --build .
        INSTALL_COMMAND ""
        TEST_COMMAND ""
    )
    # ExternalProject_Get_Property(serialcom install_dir)
    ExternalProject_Get_Property(serialcom binary_dir)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/include)
    # message(${install_dir})

    add_executable( serial serial.cpp )
    add_dependencies(serial serialcom)
    target_link_libraries( serial ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}serialcomlib.so -pthread)
