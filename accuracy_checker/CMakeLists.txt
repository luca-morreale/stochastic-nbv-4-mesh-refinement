
project(MeshAccuracyChecker)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_library(GMP_LIBRARY gmp /usr/lib)

#set(MANIFOLD_LIB manifoldReconstructor)
set(MANIFOLD_LIB realtimeManifoldReconstructor)

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
                      ${PCL_INCLUDE_DIRS}
                      )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include/)

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

file(GLOB meshac_SOURCE src/*.cpp)

add_library(meshac SHARED ${meshac_SOURCE})

target_link_libraries(meshac ${OpenCV_LIBS} 
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB} 
                                            ${MESH_ACCURACY_LIB}
                                            ${GMP_LIBRARY}
                                            ${PCL_LIBRARIES}
                                            )

# ------------------------ INSTALLATION ------------------------

install(TARGETS meshac DESTINATION lib)
install(DIRECTORY include/meshac/ DESTINATION include/meshac)

# --------------------------------------------------------------
