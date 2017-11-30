cmake_minimum_required(VERSION 2.8.3)

project (testAccuracy)
 
# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_library(GMP_LIBRARY gmp /usr/lib)

set(MANIFOLD_LIB manifoldReconstructor)
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
                      ${PCL_INCLUDE_DIRS}
                      )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include/)


# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

file(GLOB main_SOURCE src/OpenMvgParser.cpp src/ReportGenerator.cpp src/filter_points.cpp src/convertSfM.cpp )

add_executable(residualColorer src/main_residual.cpp ${main_SOURCE} )
add_executable(inverseColorer src/main_inverted.cpp ${main_SOURCE} )
add_executable(photoColorer src/main_photo.cpp ${main_SOURCE} )
add_executable(pointColorer src/main_point.cpp ${main_SOURCE} )
add_executable(ssdColorer src/main_ssd.cpp ${main_SOURCE} )
add_executable(nccColorer src/main_ncc.cpp ${main_SOURCE} )

add_executable(residualReport src/fast_residual.cpp ${main_SOURCE} )
add_executable(inverseReport src/fast_inverted.cpp ${main_SOURCE} )
add_executable(photoReport src/fast_photo.cpp ${main_SOURCE} )
add_executable(pointReport src/fast_point.cpp ${main_SOURCE} )
add_executable(ssdReport src/fast_ssd.cpp ${main_SOURCE} )
add_executable(nccReport src/fast_ncc.cpp ${main_SOURCE} )


target_link_libraries(residualColorer ${OpenCV_LIBS} 
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
target_link_libraries(inverseColorer ${OpenCV_LIBS} 
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
target_link_libraries(photoColorer ${OpenCV_LIBS} 
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
                                            ${GMP_LIBRARY} 
                                            ${PCL_LIBRARIES}
                                            )
target_link_libraries(ssdColorer ${OpenCV_LIBS} 
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
target_link_libraries(nccColorer ${OpenCV_LIBS} 
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

target_link_libraries(residualReport ${OpenCV_LIBS} 
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
target_link_libraries(inverseReport ${OpenCV_LIBS} 
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
target_link_libraries(photoReport ${OpenCV_LIBS} 
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
target_link_libraries(pointReport ${OpenCV_LIBS} 
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
target_link_libraries(ssdReport ${OpenCV_LIBS} 
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
target_link_libraries(nccReport ${OpenCV_LIBS} 
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
