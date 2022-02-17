cmake_minimum_required(VERSION 3.21)
project(Tessellation)

set(CMAKE_CXX_STANDARD 14)

include_directories(
        Vendor/glfw/include
        Vendor/glad/include
        Vendor/glm
        Source
)
include_directories(Vendor/glfw)

add_library(GLAD STATIC Vendor/glad/src/glad.c Vendor/glad/include/glad/glad.h)

find_package(OpenGL REQUIRED)

set(glm_DIR Vendor/glm/cmake/glm)
find_package(glm REQUIRED)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(Vendor/glfw)


add_executable(Tessellation
        Source/main.cpp
)

target_link_libraries(Tessellation PRIVATE glfw OpenGL::GL GLAD glm::glm)
