set(MODULE_ENCODER_ENABLED TRUE)

if(MODULE_ENCODER_ENABLED)
    add_library(usermod_encoder INTERFACE)

    target_sources(usermod_encoder INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.c
    )

    target_include_directories(usermod_encoder INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_compile_definitions(usermod_encoder INTERFACE
        -DMODULE_ENCODER_ENABLED=1
    )
endif()
