    add_library(LibCamera STATIC IMPORTED)
    set_target_properties(LibCamera PROPERTIES
        IMPORTED_LOCATION "/path/to/LibCamera.a"
        INTERFACE_INCLUDE_DIRECTORIES "/path/to/LibCamera/include"
    )
