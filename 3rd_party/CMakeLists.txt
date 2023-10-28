include(FetchContent)

FetchContent_Declare(
  zed-open-capture
  URL "https://github.com/stereolabs/zed-open-capture/archive/refs/tags/v0.5.0.zip"
  URL_HASH SHA256=1c2344855c6c805df2cd2c2b54e323bcd5d9bc065fd30bf80a0e68178b81fb06
  PATCH_COMMAND patch -N -r- -i  "${CMAKE_CURRENT_LIST_DIR}/zed-open-capture/cmake_version.patch" || true
)
add_subdirectory(zed-open-capture)