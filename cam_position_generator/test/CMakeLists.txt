
project(OptimalViewEstimator)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GSL REQUIRED)

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


include_directories(${EIGEN3_INCLUDE_DIR}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
                      ${GSL_INCLUDE_DIRS}
                      ${GSLCBLAS_INCLUDE_DIRS}
                      )

include_directories(include/)


# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

add_executable(gview src/main_graph.cpp src/OpenMvgParser.cpp )
add_executable(multi src/main_multigraph.cpp src/OpenMvgParser.cpp )
add_executable(automulti src/main_automulti.cpp src/OpenMvgParser.cpp )
add_executable(mcview src/main_mcmc.cpp src/OpenMvgParser.cpp )
add_executable(automcview src/main_automc.cpp src/OpenMvgParser.cpp )
add_executable(psoview src/main_pso.cpp src/OpenMvgParser.cpp )
add_executable(autopsoview src/main_autopso.cpp src/OpenMvgParser.cpp )

add_executable(scorer src/main_score.cpp src/OpenMvgParser.cpp )
add_executable(db src/db_scorer.cpp src/OpenMvgParser.cpp )


target_link_libraries(gview ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(multi ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(automulti ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(mcview ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(automcview ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )
target_link_libraries(psoview ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )
target_link_libraries(autopsoview ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(scorer ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(db ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${MANIFOLD_LIB}
                                            ${GMP_LIBRARY}
                                            ${MAXFLOW_LIB}
                                            ${AD3_LIB}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

# ------------------------ INSTALLATION ------------------------



# --------------------------------------------------------------
