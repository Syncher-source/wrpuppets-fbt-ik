# wrpuppets-fbt-ik: full-body IK for VR trackers on any humanoid skeleton -- a static library, no dependencies.
#   add_subdirectory(wrpuppets-fbt-ik)
#   target_link_libraries(your_target PRIVATE wrpuppets_fbt_ik)      # then #include "Fbt/FbtIk.hpp"
cmake_minimum_required(VERSION 3.20)
project(wrpuppets_fbt_ik LANGUAGES CXX)

add_library(wrpuppets_fbt_ik STATIC
    src/Fbt/FbtSkeleton.cpp
    src/Fbt/FbtIk.cpp
    src/Fbt/FbtCalibration.cpp
    src/Fbt/FbtFilter.cpp)
target_include_directories(wrpuppets_fbt_ik PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(wrpuppets_fbt_ik PUBLIC cxx_std_17)
