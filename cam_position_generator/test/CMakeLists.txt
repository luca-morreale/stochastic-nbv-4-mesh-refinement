
project(OptimalViewEstimator)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Cplex REQUIRED)

find_library(GMP_LIBRARY gmp /usr/lib)

set(MANIFOLD_LIB realtimeManifoldReconstructor)
set(OPTIMAL_VIEW_LIB optimalView)
set(MAXFLOW_LIB external-library-maxflow-shared)
set(AD3_LIB external-library-ad3-shared)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ----------------------- INCLUDES ----------------------------

if(CPLEX_FOUND)
    set(OPTIMIZER_INCLUDE_DIRS ${CPLEX_INCLUDE_DIRS})
    set(OPTIMIZER_LIBRARIES ${CPLEX_LIBRARIES})
endif()

include_directories(${EIGEN3_INCLUDE_DIR}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
                      ${OPTIMIZER_INCLUDE_DIRS}
                      )

include_directories(include/)

# CPLEX switch to be compatible with STL
ADD_DEFINITIONS(-DIL_STD)

# print out the dependencies
message(STATUS "  Optimizer: ${OPTIMIZER_INCLUDE_DIR}")

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

add_executable(view src/main.cpp src/OpenMvgParser.cpp)

add_executable(gview src/main_graph.cpp src/OpenMvgParser.cpp)

target_link_libraries(view ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${OPTIMIZER_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB} )

target_link_libraries(gview ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${OPTIMIZER_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB} )

# ------------------------ INSTALLATION ------------------------



# --------------------------------------------------------------
