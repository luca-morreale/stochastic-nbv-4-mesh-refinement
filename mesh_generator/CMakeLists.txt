cmake_minimum_required(VERSION 2.8.3)

project(manifoldReconstructor)  
   
#----------------------- CMAKE MODULES ------------------------ 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)  
  
# ----------------------- LIBRARIES ---------------------------
find_library(GMP_LIBRARY gmp /usr/lib)
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)

set(MANIFOLD_LIBS manifoldReconstructor)

# ----------------------- INCLUDES ----------------------------
include_directories(  ${EIGEN3_INCLUDE_DIR}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
                      external/
                      include/
                      ) 
# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ MESSAGES ----------------------------

# ------------------------ BUILD -------------------------------


file(GLOB_RECURSE reconstructor_SOURCE  src/Main.cpp src/OpenMvgParser.cpp )

add_executable(meshReconstructor ${reconstructor_SOURCE})

target_link_libraries(meshReconstructor ${OpenCV_LIBS} 
                                    ${GMPXX_LIBRARIES}
                                    ${GMP_LIBRARIES}
                                    ${LAPACK_LIBRARIES}
                                    ${CGAL_LIBRARY}
                                    ${Boost_LIBRARIES}
                                    ${ASSIMP_LIBRARIES}
                                    ${MANIFOLD_LIBS} 
                                    ${GMP_LIBRARY} )


