cmake_minimum_required(VERSION 3.21)
project(Tessellation)

set(CMAKE_CXX_STANDARD 17)

include_directories(
        Vendor/glfw/include
        Vendor/glad/include
        Vendor/hungarian
        Vendor/glm
        Vendor/libigl/include
#        Vendor/openmesh/src/OpenMesh/Tools
#        Vendor/openmesh/src/OpenMesh/Core
        Vendor/eigen
        Vendor/CGAL/include
        Source
)

set(CGAL_DIR Vendor/CGAL)
set(CGAL_DISABLE_GMP OFF)
#set(GMP_LIBRARIES /usr/lib/x86_64-linux-gnu/libgmp.so)
#set(MPFR_LIBRARIES /usr/lib/x86_64-linux-gnu/libmpfr.so)
#set(GMPXX_LIBRARIES /usr/lib/x86_64-linux-gnu/libgmpxx.so)
#set(MPFI_LIBRARIES /usr/lib/x86_64-linux-gnu/libmpfi.so)

set(Qt5_DIR /usr/lib/x86_64-linux-gnu/cmake/Qt5 CACHE PATH "" FORCE)

find_package(CGAL REQUIRED COMPONENTS Core OPTIONAL_COMPONENTS Qt5)

find_package(MPFI)
if(MPFI_FOUND AND NOT CGAL_DISABLE_GMP)
    message("Found MPFI")
    include(${MPFI_USE_FILE})
    include(CGAL_VersionUtils)

endif()
#add_subdirectory(Vendor/CGAL)

#set(BUILD_APPS OFF CACHE BOOL "" FORCE)
#add_subdirectory(Vendor/openmesh)

#set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
#add_subdirectory(Vendor/assimp)

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
#        Source/Mesh.cpp
#        Source/Mesh.h
#        Source/Model.cpp
#        Source/Model.h
        Source/Shader.h
        Source/Camera.h
#        Source/MeshProcessor.cpp
#        Source/MeshProcessor.h
#        Source/Primitives.h
#        Source/Primitives.cpp
#        Source/OMesh.cpp
#        Source/OMesh.h
        Source/Tessellator.cpp
        Source/Tessellator.h
        Source/GlobalParameterization.cpp
        Source/GlobalParameterization.h Source/IO.cpp Source/IO.h Source/Strategy.cpp Source/Strategy.h)

if(CGAL_Qt5_FOUND)
target_link_libraries(Tessellation PRIVATE
        glfw
        OpenGL::GL
        GLAD
        glm::glm
#        OpenMeshCore
#        OpenMeshTools
        Hungarian
        CGAL::CGAL
        CGAL::CGAL_Core
        CGAL::CGAL_Basic_viewer
        )
else()

    message(FATAL_ERROR ${CGAL_Qt5_MISSING_DEPS})
endif()