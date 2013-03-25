if(NOT LIBUSB_FOUND)
  pkg_check_modules (LIBUSB_PKG libusb)
  find_path(LIBUSB_INCLUDE_DIR NAMES usb.h
    PATHS
    ${LIBUSB_PKG_INCLUDE_DIRS}
    /usr/include/libusb
    /usr/include
    /usr/local/include
  )

  find_library(LIBUSB_LIBRARIES NAMES usb
    PATHS
    ${LIBUSB_PKG_LIBRARY_DIRS}
    /usr/lib
    /usr/local/lib
  )

if(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)
  set(LIBUSB_FOUND TRUE CACHE INTERNAL "libusb found")
  message(STATUS "Found libusb: ${LIBUSB_INCLUDE_DIR}, ${LIBUSB_LIBRARIES}")
else(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)
  set(LIBUSB_FOUND FALSE CACHE INTERNAL "libusb found")
  message(STATUS "libusb not found.")
endif(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)

mark_as_advanced(LIBUSB_INCLUDE_DIR LIBUSB_LIBRARIES)

endif(NOT LIBUSB_FOUND)
