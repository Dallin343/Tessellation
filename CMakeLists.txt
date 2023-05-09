cmake_minimum_required(VERSION 3.21)
project(Tessellation)

set(CMAKE_CXX_STANDARD 17)

include_directories(
        Vendor/glfw/include
        Vendor/glad/include
        Vendor/hungarian
        Vendor/glm
        Vendor/libigl/include
        Vendor/eigen
        Vendor/CGAL/include
        Vendor/indicators
        Vendor/lodepng
        Vendor/cereal/include
        Source
)

set(CGAL_DIR Vendor/CGAL)
set(CGAL_DISABLE_GMP OFF)

set(Qt5_DIR /usr/lib/x86_64-linux-gnu/cmake/Qt5 CACHE PATH "" FORCE)

#find_package(CGAL REQUIRED COMPONENTS Core OPTIONAL_COMPONENTS Qt5)
find_package(CGAL REQUIRED COMPONENTS Core)

find_package(MPFI)
if(MPFI_FOUND AND NOT CGAL_DISABLE_GMP)
    message("Found MPFI")
    include(${MPFI_USE_FILE})
    include(CGAL_VersionUtils)

endif()

add_library(GLAD STATIC Vendor/glad/src/glad.c Vendor/glad/include/glad/glad.h)
add_library(Hungarian STATIC Vendor/hungarian/Hungarian.cpp Vendor/hungarian/Hungarian.h)

find_package(OpenGL REQUIRED)

set(glm_DIR Vendor/glm/cmake/glm)
find_package(glm REQUIRED)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(Vendor/glfw)


add_executable(Tessellation
        Source/main.cpp
        Source/Shader.h
        Source/Camera.h
        Source/Tessellator.cpp
        Source/Tessellator.h
        Source/IO.cpp
        Source/IO.h
        Source/Strategy.cpp
        Source/Strategy.h
        Source/Utils.cpp
        Source/Utils.h
        Source/Prepare.cpp
        Source/Prepare.h
        Source/OGLMesh.cpp
        Source/OGLMesh.h
        Source/stb.h
        Vendor/lodepng/lodepng.cpp
        Vendor/lodepng/lodepng.h
        )

if(CGAL_Qt5_FOUND)
target_link_libraries(Tessellation PRIVATE
        glfw
        OpenGL::GL
        GLAD
        glm::glm
        Hungarian
        CGAL::CGAL
        CGAL::CGAL_Core
#        CGAL::CGAL_Basic_viewer
        )
else()
    target_link_libraries(Tessellation PRIVATE
            glfw
            OpenGL::GL
            GLAD
            glm::glm
            Hungarian
            CGAL::CGAL
            CGAL::CGAL_Core
            #        CGAL::CGAL_Basic_viewer
            )
endif()