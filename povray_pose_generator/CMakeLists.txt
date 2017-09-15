project(pose_povray)
cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Boost COMPONENTS system filesystem REQUIRED)

find_library(GMP_LIBRARY gmp /usr/lib)


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ----------------------- INCLUDES ----------------------------

include_directories(include/)

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ BUILD -------------------------------

add_executable(povpose src/main.cpp )

target_link_libraries(povpose ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                )

add_executable(mappose src/mapping.cpp )

target_link_libraries(povpose ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                )

add_executable(genrot src/pose_generator.cpp )

target_link_libraries(genrot ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                )

add_executable(gentrans src/translation_gen.cpp )

target_link_libraries(gentrans ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                )

add_executable(complextrans src/complex_translation_gen.cpp )

target_link_libraries(complextrans ${GMPXX_LIBRARIES}
                                ${GMP_LIBRARIES}
                                ${GMP_LIBRARY}
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                )

# ------------------------ INSTALLATION ------------------------



# --------------------------------------------------------------
