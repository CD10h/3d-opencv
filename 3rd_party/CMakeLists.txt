include(FetchContent)

FetchContent_Declare(
  zed-open-capture
  GIT_REPOSITORY "https://github.com/stereolabs/zed-open-capture.git"
  GIT_TAG "v0.5.0"
  PATCH_COMMAND patch -N -r- -i  "${CMAKE_CURRENT_LIST_DIR}/zed-open-capture/cmake_version.patch" || true
)
add_subdirectory(zed-open-capture)