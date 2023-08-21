cmake_minimum_required(VERSION 3.5)
project(sonar_pcl  LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED COMPONENT common io visualization filters segmentation)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )


file(GLOB PRO_SOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB PRO_INCLUDE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h *.hpp)

include_directories(
	include/
	${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}
)

add_executable( ${PROJECT_NAME} bridgesonar.cpp ${PRO_SOURCE_DIR} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} )