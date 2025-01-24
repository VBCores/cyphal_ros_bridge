include(CMakePrintHelpers)

cmake_minimum_required(VERSION 3.0.2)
project(cyphal_ros)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

get_filename_component(CYPHAL_DIR "libs/libcxxcanard" ABSOLUTE)
message(STATUS "${CMAKE_CURRENT_LIST_DIR} | Using <${CYPHAL_DIR}> as directory of libcxxcanard")
add_subdirectory(${CYPHAL_DIR} ${PROJECT_BINARY_DIR}/build/libcxxcanard)

file(GLOB_RECURSE SOURCES "src/*.cpp")
cmake_print_variables(SOURCES)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(FILES
    HMILed.msg
    HMIBeeper.msg
    PowerButtons.msg
)
 generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  include
  src
  libs/types
  libs/libvoltbro
  libs/nlohmann
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${CYPHAL_DIR}
  ${COMMON_LIBS_DIR}
  ${Boost_INCLUDE_DIRS}
)
link_libraries(
  libcxxcanard
)
set(COMPILE_FEATS c_std_99 cxx_std_17)

add_executable(${PROJECT_NAME}_node ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} libcxxcanard)
target_compile_features(${PROJECT_NAME}_node PUBLIC ${COMPILE_FEATS})

install(TARGETS libcxxcanard
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
