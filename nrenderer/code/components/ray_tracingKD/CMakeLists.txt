cmake_minimum_required(VERSION 3.18)

# 设置名称， 会在"/components”文件夹下生成  名称.dll
set(MY_COMPONENT_NAME "Ray_KD_tracing")

file(GLOB_RECURSE COMP_HEADER_FILES "./include/*.h" "./include/*.hpp")
source_group("Header Files" FILES ${COMP_HEADER_FILES})
file(GLOB_RECURSE COMP_SOURCE_FILES "./src/*.cpp")
# 恢复到无 KD-Tree 的状态：从编译源中排除 KDTree.cpp
# list(FILTER COMP_SOURCE_FILES EXCLUDE REGEX ".*/accel/KDTree\\.cpp$")
add_library(${MY_COMPONENT_NAME} SHARED "${COMP_SOURCE_FILES}" "${COMP_HEADER_FILES}")
target_link_libraries(${MY_COMPONENT_NAME} NRServer)

include_directories("./include")
