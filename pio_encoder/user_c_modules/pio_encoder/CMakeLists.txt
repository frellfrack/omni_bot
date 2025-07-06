set(MODULE_PIO_ENCODER_ENABLED TRUE)

if(MODULE_PIO_ENCODER_ENABLED)
    add_library(usermod_pio_encoder INTERFACE)

    target_sources(usermod_pio_encoder INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.c
    )

    target_include_directories(usermod_pio_encoder INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_compile_definitions(usermod_pio_encoder INTERFACE
        -DMODULE_PIO_ENCODER_ENABLED=1
    )
endif()
