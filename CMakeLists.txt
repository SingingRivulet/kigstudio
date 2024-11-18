cmake_minimum_required(VERSION 3.10)  
  
# 设置项目名称  
project(kigstudio)  
  
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置包含目录  
include_directories(${PROJECT_SOURCE_DIR}/src)  
  
# 查找src目录中的所有.c文件  
file(GLOB_RECURSE SRC_FILES "${PROJECT_SOURCE_DIR}/src/**/*.cpp")  

# 创建静态库libkigstudio.a  
add_library(kigstudio STATIC ${SRC_FILES})  

# 编译可执行文件
file(GLOB_RECURSE TEST_FILES "${PROJECT_SOURCE_DIR}/tests/*.cpp")  
foreach(TEST_FILE ${TEST_FILES})  
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)  
    set(OUTPUT_DIR ${PROJECT_BINARY_DIR}/tests)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb -fsanitize=address -fno-omit-frame-pointer -Wall -DDEBUG -DDEBUG_DUMP -rdynamic")
    add_executable(${TEST_NAME} ${TEST_FILE})  
    set_target_properties(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
    target_link_libraries(${TEST_NAME} kigstudio)  
endforeach()
