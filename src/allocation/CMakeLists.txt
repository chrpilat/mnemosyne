CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(
${CMAKE_SOURCE_DIR}/src/accelerator
${CMAKE_SOURCE_DIR}/src/plm_controller
${CMAKE_SOURCE_DIR}/src/rtl_architecture
${CMAKE_SOURCE_DIR}/src/utils
${CMAKE_SOURCE_DIR}/src/utils/yaml-cpp/include
${LibXML++_INCLUDE_DIRS}
${Boost_INCLUDE_DIR}
)

ADD_LIBRARY(
lib-allocation
AreaHeuristic.cpp
Functor.cpp
Allocation.cpp
)
