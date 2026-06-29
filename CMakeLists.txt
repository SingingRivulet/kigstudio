cmake_minimum_required(VERSION 3.10)  
  
# 设置项目名称  
project(kigstudio)  
find_package(OpenMP REQUIRED)

# Boost (required by CGAL) — header-only branch build
set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/dep/boost")
set(Boost_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/dep/boost")
set(Boost_NO_BOOST_CMAKE TRUE CACHE BOOL "" FORCE)

# CGAL (branch build, header-only)
set(CGAL_DIR "${CMAKE_SOURCE_DIR}/dep/cgal/Installation")

# CMake 3.31+ removed FindBoost from its default module path.
# Provide our own patched copy in scripts/cmake_modules so CGAL can still use it.
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/scripts/cmake_modules" ${CMAKE_MODULE_PATH})
find_package(CGAL REQUIRED)

# Eigen3 (header-only, required by CGAL Surface_mesh_skeletonization)
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/dep/eigen3")
find_package(Eigen3 3.3.7 QUIET)
if(NOT Eigen3_FOUND)
    # Manual fallback for branch-build / tarball layout
    set(EIGEN3_FOUND TRUE)
    set(Eigen3_VERSION "3.4.0")
endif()

set(SDL_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# MSVC 默认使用系统代码页解析源码，添加 /utf-8 使其按 UTF-8 处理源文件和执行字符集
if(MSVC)
    add_compile_options("/utf-8")
    add_compile_definitions(NOMINMAX)
endif() 

add_subdirectory(dep/bgfx.cmake)
add_subdirectory(dep/zlib)

set(SDL2_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include")
set(SDL2_LIB_DIR "${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/lib/x64")

# 设置包含目录  
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${CGAL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/dep/libtinyfiledialog 
    ${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/examples/common
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/3rdparty/
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bgfx/include/
    ${CMAKE_SOURCE_DIR}/dep/bgfx.cmake/bx/include/
    ${CMAKE_SOURCE_DIR}/dep/SDL2-2.30.11/include
    ${CMAKE_SOURCE_DIR}/dep/imnodes
    ${CMAKE_SOURCE_DIR}/dep/zlib
    ${CMAKE_SOURCE_DIR}/dep/cJSON
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

# 创建静态库libkigstudio_lib.a
add_library(kigstudio_lib STATIC ${SRC_FILES} ${CMAKE_SOURCE_DIR}/dep/cJSON/cJSON.c)
target_link_libraries(kigstudio_lib PUBLIC OpenMP::OpenMP_CXX CGAL::CGAL)
if(TARGET CGAL::Eigen3_support)
    target_link_libraries(kigstudio_lib PUBLIC CGAL::Eigen3_support)
else()
    target_compile_definitions(kigstudio_lib PUBLIC CGAL_EIGEN3_ENABLED)
endif()

# 编译可执行文件
file(GLOB_RECURSE TEST_FILES "${PROJECT_SOURCE_DIR}/tests/*.cpp")  

foreach(TEST_FILE ${TEST_FILES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${TEST_NAME} ${TEST_FILE})

    target_link_libraries(${TEST_NAME}
        kigstudio_lib
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
    kigstudio
    ${UI_FILES}
    ${CMAKE_SOURCE_DIR}/dep/imnodes/imnodes.cpp
    )

target_link_libraries(kigstudio
    kigstudio_lib
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
    zlib
)

# 构建时将 shader 目录复制到可执行文件输出目录旁
add_custom_command(TARGET kigstudio POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/shader
    $<TARGET_FILE_DIR:kigstudio>/shader
)