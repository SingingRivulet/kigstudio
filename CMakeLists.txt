cmake_minimum_required(VERSION 3.10)  
  
# 设置项目名称  
project(kigstudio)  
find_package(OpenMP REQUIRED)

set(SDL_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# MSVC 默认使用系统代码页解析源码，添加 /utf-8 使其按 UTF-8 处理源文件和执行字符集
if(MSVC)
    add_compile_options("/utf-8")
endif() 

add_subdirectory(dep/bgfx.cmake)

set(SDL2_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include")
set(SDL2_LIB_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/lib/x64")

# 设置包含目录  
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/dep/libtinyfiledialog 
    ${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/examples/common
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/3rdparty/
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/include/
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bx/include/
    ${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include
    ${CMAKE_SOURCE_DIR}/dep/imnodes
    )  

# imgui
add_library(imgui
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/3rdparty/dear-imgui/imgui.cpp
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/3rdparty/dear-imgui/imgui_draw.cpp
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/3rdparty/dear-imgui/imgui_widgets.cpp
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/3rdparty/dear-imgui/imgui_tables.cpp
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/examples/common/imgui/imgui.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/3rdparty/dear-imgui
)

target_link_libraries(imgui PUBLIC bgfx bx bimg)

# 查找src目录中的所有.c文件  
file(GLOB_RECURSE SRC_FILES 
    "${PROJECT_SOURCE_DIR}/src/**/*.cpp"
    ${CMAKE_SOURCE_DIR}/dep/libtinyfiledialog/tinyfiledialogs.c
    )  

# 创建静态库libkigstudio.a  
add_library(kigstudio STATIC ${SRC_FILES})  
target_link_libraries(kigstudio PUBLIC OpenMP::OpenMP_CXX)

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
        imgui
    )
endforeach()

file(GLOB_RECURSE UI_FILES
    "${CMAKE_SOURCE_DIR}/ui/*.cpp"
)

add_executable(
    kigstudio_ui 
    ${UI_FILES}
    ${CMAKE_SOURCE_DIR}/dep/imnodes/imnodes.cpp
    )
    
target_link_libraries(kigstudio_ui
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
    imgui
)