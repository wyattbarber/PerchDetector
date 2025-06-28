    add_library(LibCamera STATIC IMPORTED)
    set_target_properties(LibCamera PROPERTIES
        IMPORTED_LOCATION "/usr/local/lib/aarch64-linux-gnu/libcamera.so"
        INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include/libcamera"
    )
