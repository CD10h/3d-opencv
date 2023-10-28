message(STATUS "Fetching zed-open-capture...")

set(BUILD_VIDEO      ON)
set(BUILD_SENSORS    OFF)
set(BUILD_EXAMPLES   OFF)
set(DEBUG_CAM_REG    OFF)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
FetchContent_MakeAvailable(zed-open-capture)
