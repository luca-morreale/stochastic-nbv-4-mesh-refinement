
project(mesh_cam_generator)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(CGAL REQUIRED)

find_library(GMP_LIBRARY gmp /usr/lib)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${LIBSSH_PUBLIC_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIBSSH_LIBRARY ssh)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ----------------------- INCLUDES ----------------------------

include_directories(include/)
include_directories(${CGAL_INCLUDE_DIRS})

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

file(GLOB SOURCE src/*.cpp)

add_executable(eval main/main.cpp ${SOURCE} )

target_link_libraries(eval ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )

add_executable(testssh main/test_ssh.cpp ${SOURCE} )

target_link_libraries(testssh ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )

add_executable(result main/main_closest.cpp ${SOURCE} )

target_link_libraries(result ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )

# ------------------------ INSTALLATION ------------------------



# --------------------------------------------------------------
