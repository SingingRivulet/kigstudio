cmake_minimum_required(VERSION 3.10)  
  
# 设置项目名称  
project(kigstudio)  

set(SDL_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True) 

add_subdirectory(dep/bgfx.cmake)

set(SDL2_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include")
set(SDL2_LIB_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/lib/x64")

# 设置包含目录  
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/dep/libtinyfiledialog 
    ${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/examples/common
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/3rdparty/
    ${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include
    )  
  
# 查找src目录中的所有.c文件  
file(GLOB_RECURSE SRC_FILES 
    "${PROJECT_SOURCE_DIR}/src/**/*.cpp"
    ${CMAKE_SOURCE_DIR}/dep/libtinyfiledialog/tinyfiledialogs.c
    )  

# 创建静态库libkigstudio.a  
add_library(kigstudio STATIC ${SRC_FILES})  

# 编译可执行文件
file(GLOB_RECURSE TEST_FILES "${PROJECT_SOURCE_DIR}/tests/*.cpp")  

foreach(TEST_FILE ${TEST_FILES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${TEST_NAME} ${TEST_FILE})

    target_link_libraries(${TEST_NAME}
        kigstudio
        bgfx
        bx
        bimg

        "${SDL2_LIB_DIR}/SDL2.lib"
        # "${SDL2_LIB_DIR}/SDL2main.lib"

        d3d11
        dxgi
        dxguid
        user32
        gdi32
        shell32
        ole32
        winmm
    )
endforeach()
