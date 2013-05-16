# Project Description
cmake_minimum_required(VERSION 2.6)
project(hubo_vision CXX C)
set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS roscpp message_generation genmsg std_msgs sensor_msgs geometry_msgs)

file(GLOB service_source "srv/*")
LIST(SORT service_source)
foreach(service_src_file ${service_source})
    get_filename_component(service_base ${service_src_file} NAME)
    message(STATUS "Adding service ${service_base}" )
    add_service_files(DIRECTORY srv  FILES ${service_base}) 
endforeach(service_src_file)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs cv_bridge image_transport tf eigen_conversions tabletop_object_detector
    DEPENDS ach )

link_libraries(${catkin_LIBRARIES})
link_libraries(tf)

#link_libraries(roscpp roscpp_serialization rosconsole rostime roslib tf message_filters cv_bridge image_transport eigen_conversions)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/find/")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include Threading
find_package (Threads)
link_libraries("${CMAKE_THREAD_LIBS_INIT}")

# Include Boost
find_package(Boost REQUIRED)# COMPONENTS system filesystem thread)
include_directories(${BOOST_INCLUDE_DIRS})
link_libraries(${BOOST_LIBRARIES})
link_libraries(boost_system boost_thread boost_signals) #???

# Include the OpenCV library
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

# Include the PCL library
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES})

# Include the Eigen library
#set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# ACH
link_libraries(ach)

# hubo-motion-rt
link_libraries(huboctrl daemonizer rt huboparams)

# Include CppUnit
find_package(CppUnit)
include_directories(${CPPUNIT_INCLUDE_DIR})
link_libraries(${CPPUNIT_LIBRARIES})

# Include the project files
include_directories("include")
include_directories("srv_gen/cpp/include")

# Include the Hubo_Tech components
#include_directories(modules/hubo-motion-rt/include)
#include_directories(modules/hubo-ach/include)

  
# Collect the source, script and test files
file(GLOB lib_source "src/libraries/*.cpp")
LIST(SORT lib_source)
file(GLOB tests_source "src/tests/*.cpp")
LIST(SORT tests_source)
file(GLOB unit_tests_source "src/tests/unit/*.cpp")
LIST(SORT unit_tests_source)
file(GLOB system_tests_source "src/tests/system/*.cpp")
LIST(SORT system_tests_source)
file(GLOB scripts_source "src/exe/*.cpp")
LIST(SORT scripts_source)
file(GLOB resource_source "resources/*")
LIST(SORT resource_source)
file(COPY ${resource_source} DESTINATION ${EXECUTABLE_OUTPUT_PATH})

# Build library
add_library(${PROJECT_NAME} SHARED ${lib_source})

enable_testing()

# Build all tests
message(STATUS "\n-- FULL TESTS: ")
foreach(test_src_file ${tests_source})
    get_filename_component(test_base ${test_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}" )
    add_executable(${test_base} ${test_src_file} ${unit_tests_source})
    target_link_libraries(${test_base} ${catkin_LIBRARIES} ${PROJECT_NAME})
    add_dependencies(${test_base} ${PROJECT_NAME}_gencpp)
endforeach(test_src_file)

# Build unit tests
message(STATUS "\n-- UNIT TESTS: ")
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
foreach(utest_src_file ${unit_tests_source})
    get_filename_component(test_base ${utest_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}" )
    add_executable(${test_base} "src/tests/test_units.cpp" ${utest_src_file})
    target_link_libraries(${test_base} ${catkin_LIBRARIES} ${PROJECT_NAME})
    add_dependencies(${test_base} ${PROJECT_NAME}_gencpp)
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check ${test_base})
endforeach(utest_src_file)

# Build system tests
message(STATUS "\n-- SYSTEM TESTS: ")
#add_custom_target(ucheck COMMAND ${CMAKE_CTEST_COMMAND})
foreach(test_src_file ${system_tests_source})
    get_filename_component(test_base ${test_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}" )
    add_executable(${test_base} ${test_src_file})
    target_link_libraries(${test_base} ${catkin_LIBRARIES} ${PROJECT_NAME})
    add_dependencies(${test_base} ${PROJECT_NAME}_gencpp)
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
endforeach(test_src_file)

# Build executables
message(STATUS "\n-- EXECUTABLES: ")
foreach(script_src_file ${scripts_source})
    get_filename_component(script_base ${script_src_file} NAME_WE)
    message(STATUS "Adding executable ${script_base}" )
    add_executable(${script_base} ${script_src_file})
    target_link_libraries(${script_base} ${catkin_LIBRARIES} ${PROJECT_NAME})
    add_dependencies(${script_base} ${PROJECT_NAME}_gencpp)
    add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)

