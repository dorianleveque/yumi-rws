cmake_minimum_required(VERSION 2.8.3)
project(yumi_rws)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp abb_librws)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp abb_librws
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#################################
# Build the basic level samples #
#################################

add_executable(${PROJECT_NAME}_app src/main.cpp)
set_target_properties(${PROJECT_NAME}_app 
  PROPERTIES 
  OUTPUT_NAME ${PROJECT_NAME}_app
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
add_dependencies(${PROJECT_NAME}_app ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_app ${catkin_LIBRARIES})
