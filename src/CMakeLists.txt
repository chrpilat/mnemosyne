CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

ADD_SUBDIRECTORY(accelerator)
ADD_SUBDIRECTORY(allocation)
ADD_SUBDIRECTORY(plm_controller)
ADD_SUBDIRECTORY(backend)
ADD_SUBDIRECTORY(utils)

FIND_PACKAGE(
Boost 1.40
COMPONENTS filesystem system program_options graph
REQUIRED
)

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/src/revision_hash.hpp
    WORKING_DIR ${CMAKE_BINARY_DIR}/src
    COMMAND git rev-parse HEAD > ./revision_hash
    COMMAND xxd -i revision_hash > ./revision_hash.hpp
)

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/src/branch_name.hpp
    WORKING_DIR ${CMAKE_BINARY_DIR}/src
    COMMAND git rev-parse --abbrev-ref HEAD > ./branch_name
    COMMAND xxd -i branch_name > ./branch_name.hpp
)

ADD_CUSTOM_TARGET(revision ALL DEPENDS ${CMAKE_BINARY_DIR}/src/revision_hash.hpp)
ADD_CUSTOM_TARGET(branch-name ALL DEPENDS ${CMAKE_BINARY_DIR}/src/branch_name.hpp)

INCLUDE_DIRECTORIES(
${CMAKE_SOURCE_DIR}/src/allocation
${CMAKE_SOURCE_DIR}/src/plm_controller
${CMAKE_SOURCE_DIR}/src/accelerator
${CMAKE_SOURCE_DIR}/src/backend
${CMAKE_SOURCE_DIR}/src/utils
${CMAKE_SOURCE_DIR}/src/utils/yaml-cpp/include
${CMAKE_BINARY_DIR}/src
${Boost_INCLUDE_DIRS}
${LibXML++_INCLUDE_DIRS}
)

ADD_EXECUTABLE(
mnemosyne
mnemosyne.cpp
)
TARGET_LINK_LIBRARIES(
mnemosyne
lib-allocation
lib-accelerator
lib-plm_controller
lib-backend
yaml-cpp
${Boost_LIBRARIES}
${LibXML++_LIBRARIES}
)
ADD_DEPENDENCIES(mnemosyne revision branch-name)
INSTALL(
TARGETS mnemosyne
RUNTIME
DESTINATION bin
PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
