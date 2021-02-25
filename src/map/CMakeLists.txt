cmake_minimum_required(VERSION 3.0.2)

project(Map)

set(CMAKE_CXX_STANDARD 11)

FILE(GLOB CPP_FILES
  "*.cpp"
  )

######################
#### Internal API ####
######################

include_directories(.)

add_library(${PROJECT_NAME} STATIC ${CPP_FILES})

foreach (lib ${LIBS})
  target_link_libraries(${PROJECT_NAME} ${lib})
endforeach()
