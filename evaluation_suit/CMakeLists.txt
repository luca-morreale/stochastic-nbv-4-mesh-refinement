
project(mesh_cam_generator)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(CGAL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

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

include_directories(${PCL_INCLUDE_DIRS} 
                ${EIGEN3_INCLUDE_DIR} 
                ${OpenCV_INCLUDE_DIRS} 
                ${LIBSSH_PUBLIC_INCLUDE_DIRS} 
                )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

file(GLOB eval_SOURCE src/*.cpp)

add_library(eval SHARED ${eval_SOURCE})

target_link_libraries(eval ${OpenCV_LIBS}  ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
				${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                            )

add_executable(dbeval main/main_db.cpp )

target_link_libraries(dbeval eval
                                ${OpenCV_LIBS}  
                                ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )

add_executable(closest main/main_closest.cpp )

target_link_libraries(closest eval
                                ${OpenCV_LIBS}  
                                ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )

add_executable(transform main/main_povray.cpp )

target_link_libraries(transform eval
                                ${OpenCV_LIBS}  
                                ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )

add_executable(system main/system_evaluation.cpp )

target_link_libraries(system eval
                                ${OpenCV_LIBS}  
                                ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )


add_executable(intersection main/intersection.cpp )

target_link_libraries(intersection eval
                                ${OpenCV_LIBS}  
                                ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )


add_executable(generate_basic main/generate_basic.cpp )

target_link_libraries(generate_basic eval
                                ${OpenCV_LIBS}  
                                ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )

add_executable(comparator main/comparator.cpp )

target_link_libraries(comparator eval
                                ${OpenCV_LIBS}  
                                ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${CGAL_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${LIBSSH_LIBRARY}
                                ${PCL_LIBRARIES}
                                )


add_executable(coverage main/coverage.cpp )

target_link_libraries(coverage eval
                                ${OpenCV_LIBS}  
                                ${GMPXX_LIBRARIES}
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
