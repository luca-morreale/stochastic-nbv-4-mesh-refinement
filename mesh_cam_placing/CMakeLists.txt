
project(mesh_cam_generator)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(GMP_LIBRARY gmp /usr/lib)

set(MANIFOLD_LIB manifoldReconstructor)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ----------------------- INCLUDES ----------------------------

include_directories(${EIGEN3_INCLUDE_DIR}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
                      )

include_directories(include/)

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

add_executable(sequencegen src/main_sequence.cpp src/OpenMvgParser.cpp src/CamReader.cpp src/visualization_utils.cpp src/sphere_handler.cpp)
add_executable(scoregen src/main_best_score.cpp src/OpenMvgParser.cpp src/CamReader.cpp src/visualization_utils.cpp src/sphere_handler.cpp)
add_executable(finalgen src/main_final.cpp src/OpenMvgParser.cpp src/CamReader.cpp src/visualization_utils.cpp src/sphere_handler.cpp)

target_link_libraries(sequencegen ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${OPTIMIZER_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            )

target_link_libraries(scoregen ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${OPTIMIZER_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            )

target_link_libraries(finalgen ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${OPTIMIZER_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            )

# ------------------------ INSTALLATION ------------------------



# --------------------------------------------------------------
