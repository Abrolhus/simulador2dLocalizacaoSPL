#include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(run main.cpp Field.cpp World.cpp Robot.cpp Object.cpp Renderer.cpp)
target_include_directories(run PUBLIC ../include)
target_link_libraries(run ${OpenCV_LIBS} )
target_include_directories(run INTERFACE ${EIGEN3_INCLUDE_DIRS})

