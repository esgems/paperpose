cmake_minimum_required(VERSION 3.0.2)

project(paperpose)

find_package(OpenCV 4.0 REQUIRED QUIET PATHS /home/rustam/programs/opencv/release)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS}
)

SET(LIBRARIES
	${OpenCV_LIBRARIES} 
)

SET(APP_SRC
  ./src/PaperFinder.cpp
  ./src/PositionEstimator.cpp
  ./src/Visualizer.cpp
)

add_executable(Main ./Main.cpp ${APP_SRC})
target_link_libraries(Main ${LIBRARIES})
target_include_directories(Main PUBLIC ${INCLUDE_DIRS})

set_target_properties(Main PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON)