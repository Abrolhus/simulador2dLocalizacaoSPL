#include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES "")
add_executable(run main.cpp Field.cpp World.cpp Robot.cpp ${SOURCES})
target_include_directories(run PUBLIC ../include)
target_link_libraries(run ${OpenCV_LIBS} )
target_include_directories(run INTERFACE ${EIGEN3_INCLUDE_DIRS})

