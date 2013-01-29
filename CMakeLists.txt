# Project Description
cmake_minimum_required(VERSION 2.6)
project(HuboApplication CXX C)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/find/")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include the OpenCV library
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

# Include the Eigen library
#set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Include the project files
include_directories(include)

#Include the Hubo_Tech components
include_directories(Hubo_Tech/include)
  
# Collect the source, script and test files
file(GLOB lib_source "src/libraries/*.cpp")
LIST(SORT lib_source)
file(GLOB tests_source "src/tests/*.cpp")
LIST(SORT tests_source)
file(GLOB scripts_source "src/exe/*.cpp")
LIST(SORT scripts_source)
file(GLOB resource_source "resources/*")
LIST(SORT resource_source)
file(COPY ${resource_source} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
message(${resource_source})

# Build library
#add_library(${PROJECT_NAME} SHARED ${lib_source})
#target_link_libraries(${PROJECT_NAME} freenect) 

# Build tests
message(STATUS "\n-- TESTS: ")
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
foreach(test_src_file ${tests_source})
    get_filename_component(test_base ${test_src_file} NAME_WE)
    message(STATUS "Adding test ${test_src_file} with base name ${test_base}" )
    add_executable(${test_base} ${test_src_file})
    target_link_libraries(${test_base} ${PROJECT_NAME})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check ${test_base})
endforeach(test_src_file)

# Build scripts
message(STATUS "\n-- EXECUTABLES: ")
foreach(script_src_file ${scripts_source})
    get_filename_component(script_base ${script_src_file} NAME_WE)
    message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
    add_executable(${script_base} ${script_src_file})
    target_link_libraries(${script_base} )
    add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)

