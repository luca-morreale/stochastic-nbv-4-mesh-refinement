
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


set(MANIFOLD_LIB manifoldReconstructor)
set(OPTIMAL_VIEW_LIB optimalView)

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
add_executable(gm src/main_graph.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(mc src/main_mcmc.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(gpso src/main_global_pso.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(lpso src/main_local_pso.cpp src/OpenMvgParser.cpp src/utilities.cpp )


add_executable(bu_gm src/bu_gm.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(bu_mc src/bu_mc.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(bu_gpso src/bu_gpso.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(bu_lpso src/bu_lpso.cpp src/OpenMvgParser.cpp src/utilities.cpp )

add_executable(for_gm src/for_gm.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(for_mc src/for_mc.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(for_gpso src/for_gpso.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(for_lpso src/for_lpso.cpp src/OpenMvgParser.cpp src/utilities.cpp )

add_executable(car_gm src/car_gm.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(car_mc src/car_mc.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(car_gpso src/car_gpso.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(car_lpso src/car_lpso.cpp src/OpenMvgParser.cpp src/utilities.cpp )

add_executable(city_gm src/city_gm.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(city_mc src/city_mc.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(city_gpso src/city_gpso.cpp src/OpenMvgParser.cpp src/utilities.cpp )
add_executable(city_lpso src/city_lpso.cpp src/OpenMvgParser.cpp src/utilities.cpp )

add_executable(middlebury_scorer src/middlebury_scorer.cpp src/OpenMvgParser.cpp src/utilities.cpp )

target_link_libraries(gm  ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )
                                            
target_link_libraries(mc ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(gpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )
target_link_libraries(lpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )


target_link_libraries(bu_gm ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(bu_mc ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${MANIFOLD_LIB} 
                                            ${AD3_LIB}
                                            ${MAXFLOW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(bu_gpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(bu_lpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(for_gm ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(for_mc ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(for_gpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(for_lpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(car_gm ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}

                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(car_mc ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(car_gpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(car_lpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(city_gm ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(city_mc ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(city_gpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(city_lpso ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )

target_link_libraries(middlebury_scorer ${OpenCV_LIBS}
                                            ${OPENCV_BUILD_3RDPARTY_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${GMP_LIBRARY}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}
                                            ${OPTIMAL_VIEW_LIB}
                                            ${GSL_LIBRARIES}
                                            ${GSLCBLAS_LIBRARIES}
                                            )
