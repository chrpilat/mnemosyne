CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(
${CMAKE_SOURCE_DIR}/src/utils
${CMAKE_SOURCE_DIR}/src/utils/yaml-cpp/include
${CMAKE_SOURCE_DIR}/src/plm_controller
${LibXML++_INCLUDE_DIRS}
${Boost_INCLUDE_DIR}
)

ADD_LIBRARY(
lib-accelerator
Component.cpp
UGraph.cpp
Array.cpp
Scenario.cpp
)
