CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(
${CMAKE_SOURCE_DIR}/src/accelerator
${CMAKE_SOURCE_DIR}/src/allocation
${CMAKE_SOURCE_DIR}/src/plm_controller
${CMAKE_SOURCE_DIR}/src/rtl_architecture
${CMAKE_SOURCE_DIR}/src/utils
${CMAKE_SOURCE_DIR}/src/utils/yaml-cpp/include
${Boost_INCLUDE_DIR}
${LibXML++_INCLUDE_DIRS}
)

ADD_LIBRARY(
lib-backend
Writer.cpp
ExportWriter.cpp
VerilogWriter.cpp
IpxactWriter.cpp
CtosipWriter.cpp
StratusipWriter.cpp
)
