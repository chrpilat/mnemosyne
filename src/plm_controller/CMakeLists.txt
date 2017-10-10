CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(
${CMAKE_SOURCE_DIR}/src/utils/yaml-cpp/include
${CMAKE_SOURCE_DIR}/src/utils
${CMAKE_SOURCE_DIR}/src/accelerator
${LibXML++_INCLUDE_DIRS}
${Boost_INCLUDE_DIR}
)

ADD_LIBRARY(
lib-plm_controller
Architecture.cpp
Tag.cpp
PBlock.cpp
Partition.cpp
Memory.cpp
)
