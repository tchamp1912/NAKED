cmake_minimum_required(VERSION 3.0.2)

project(Camera)

set(CMAKE_CXX_STANDARD 11)

FILE(GLOB CPP_FILES
  "*.cpp"
  )

######################
#### Internal API ####
######################

include_directories(.)

foreach (lib ${LIBS}) 
  add_subdirectory(${lib})
endforeach()

add_library(${PROJECT_NAME} STATIC ${CPP_FILES})

foreach (lib ${LIBS}) 
  target_link_libraries(${PROJECT_NAME} ${lib})
endforeach()

######################
#### External API ####
######################

#### OPENCV ####

find_package(OpenCV REQUIRED)

#### OPENNI2 ####

find_package( OpenNI2 REQUIRED )


if( OpenNI2_FOUND AND OpenCV_FOUND )
  # Additional Include Directories
  include_directories( ${OpenNI2_INCLUDE_DIR} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  # Additional Dependencies
  target_link_libraries( Color ${OpenNI2_LIBRARY} )
  target_link_libraries( Color ${OpenCV_LIBS} )
endif()
