
project(mesh_cam_generator)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Boost COMPONENTS system filesystem REQUIRED)

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
include_directories("/usr/local/include/chilkat-9.5.0/")

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

add_executable(autoeval src/main.cpp src/Evaluator.cpp src/KDTree.cpp src/OpenMvgJsonHandler.cpp src/SshHandler.cpp src/utilities.cpp )

target_link_libraries(autoeval ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${PCL_LIBRARIES}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                )

add_executable(evalnocopy src/mainnocopy.cpp src/EvaluatorNoCopy.cpp src/OpenMvgJsonHandler.cpp src/SshHandler.cpp src/utilities.cpp )

target_link_libraries(evalnocopy ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                )

add_executable(testssh src/test_ssh.cpp src/EvaluatorNoCopy.cpp src/OpenMvgJsonHandler.cpp src/SshHandler.cpp src/utilities.cpp )

target_link_libraries(testssh ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                )


# ------------------------ INSTALLATION ------------------------



# --------------------------------------------------------------
