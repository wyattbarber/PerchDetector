add_library(LibCamera STATIC IMPORTED)
string(CONCAT _LIBCAMERA_LIB "/usr/local/lib/" ${CMAKE_SYSTEM_PROCESSOR} "-linux-gnu/libcamera.so")
set_target_properties(LibCamera PROPERTIES
    IMPORTED_LOCATION ${_LIBCAMERA_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include/libcamera"
)
