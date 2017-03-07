
project(MeshGenerator)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(GMP_LIBRARY gmp /usr/lib)

set(MANIFOLD_LIB manifoldReconstructor)

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

file(GLOB main_SOURCE src/*.cpp)

add_executable(accuracyDetector ${main_SOURCE})
target_link_libraries(accuracyDetector ${OpenCV_LIBS} 
                                  ${OpenCV2_LIBS} 
                                  ${GMPXX_LIBRARIES}
                                  ${GMP_LIBRARIES}
                                  ${LAPACK_LIBRARIES}
                                  ${CGAL_LIBRARY}
                                  ${Boost_LIBRARIES}
                                  ${ASSIMP_LIBRARIES}
                                  ${MANIFOLD_LIB}
                                  ${GMP_LIBRARY} )

# --------------------------------------------------------------
