project(capture_3d)

add_executable(${PROJECT_NAME}
    main.cpp
)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${zed_open_capture_SOURCE_DIR}/include  ${zed_open_capture_SOURCE_DIR}/examples/include)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} zed_open_capture)
add_dependencies(${PROJECT_NAME} zed_open_capture )

add_definitions(-DVIDEO_MOD_AVAILABLE)