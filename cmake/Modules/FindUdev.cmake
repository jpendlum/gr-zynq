INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_UDEV udev)

FIND_PATH(
    UDEV_INCLUDE_DIRS
    NAMES libudev.h
    PATHS /usr/local/include/
          /usr/include/
)

FIND_LIBRARY(
    UDEV_LIBRARIES
    NAMES libudev.so
    PATHS /usr/local/lib
          /usr/lib
          /usr/lib/arm-linux-gnueabihf/
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(UDEV DEFAULT_MSG UDEV_LIBRARIES UDEV_INCLUDE_DIRS)
MARK_AS_ADVANCED(UDEV_LIBRARIES UDEV_INCLUDE_DIRS)