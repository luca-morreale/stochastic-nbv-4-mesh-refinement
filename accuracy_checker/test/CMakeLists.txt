cmake_minimum_required(VERSION 2.8.3)

project (testAccuracy)
 
# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(GMP_LIBRARY gmp /usr/lib)

set(MANIFOLD_LIB realtimeManifoldReconstructor)
set(MESH_ACCURACY_LIB meshac)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ----------------------- INCLUDES ----------------------------

include_directories(  ${EIGEN3_INCLUDE_DIR} 
                      ${GMPXX_INCLUDE_DIR} 
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS} 
                      )

include_directories(include/)


# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE main_SOURCE )

add_executable(pointColorer src/Main.cpp src/OpenMvgParser.cpp)

add_executable(faceColorer src/main_face.cpp src/OpenMvgParser.cpp)

target_link_libraries(pointColorer ${OpenCV_LIBS} 
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB} 
                                            ${MESH_ACCURACY_LIB}
                                            ${GMP_LIBRARY} )

target_link_libraries(faceColorer ${OpenCV_LIBS} 
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB} 
                                            ${MESH_ACCURACY_LIB}
                                            ${GMP_LIBRARY} )
