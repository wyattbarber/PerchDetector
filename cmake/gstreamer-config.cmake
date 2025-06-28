    add_library(Gstreamer STATIC IMPORTED)
    set_target_properties(Gstreamer PROPERTIES
        IMPORTED_LOCATION "/usr/lib/aarch64-linux-gnu/libgstreamer-1.0.so"
        INTERFACE_INCLUDE_DIRECTORIES "/usr/include/gstreamer-1.0/"
    )
