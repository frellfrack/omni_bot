/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "py/obj.h"
#include "py/runtime.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"

typedef struct _mp_obj_encoder_t {
    mp_obj_base_t base;
    PIO pio;
    uint sm;
    uint pin_ab;
} mp_obj_encoder_t;

// MicroPython constructor: Encoder(pin_a)
STATIC mp_obj_t encoder_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, 1, false);
    mp_obj_encoder_t *self = m_new_obj(mp_obj_encoder_t);
    self->base.type = type;
    self->pio = pio0;
    self->sm = 0;
    self->pin_ab = mp_obj_get_int(args[0]);

    // Load and init PIO program
    uint offset = pio_add_program(self->pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(self->pio, self->sm, self->pin_ab, offset);

    return MP_OBJ_FROM_PTR(self);
}

// MicroPython method: read()
STATIC mp_obj_t encoder_read(mp_obj_t self_in) {
    mp_obj_encoder_t *self = MP_OBJ_TO_PTR(self_in);
    int32_t count = quadrature_encoder_get_count(self->pio, self->sm);
    return mp_obj_new_int(count);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(encoder_read_obj, encoder_read);

// Method table
STATIC const mp_rom_map_elem_t encoder_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&encoder_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(encoder_locals_dict, encoder_locals_dict_table);

// Type definition
STATIC const mp_obj_type_t encoder_type = {
    { &mp_type_type },
    .name = MP_QSTR_Encoder,
    .make_new = encoder_make_new,
    .locals_dict = (mp_obj_dict_t*)&encoder_locals_dict,
};

// Module globals
STATIC const mp_rom_map_elem_t encoder_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_encoder) },
    { MP_ROM_QSTR(MP_QSTR_Encoder), MP_ROM_PTR(&encoder_type) },
};
STATIC MP_DEFINE_CONST_DICT(encoder_module_globals, encoder_module_globals_table);

const mp_obj_module_t encoder_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&encoder_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_encoder, encoder_user_cmodule, MODULE_ENCODER_ENABLED);
#include "py/obj.h"
#include "py/runtime.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"

typedef struct _mp_obj_encoder_t {
    mp_obj_base_t base;
    PIO pio;
    uint sm;
    uint pin_ab;
} mp_obj_encoder_t;

// MicroPython constructor: Encoder(pin_a)
STATIC mp_obj_t encoder_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, 1, false);
    mp_obj_encoder_t *self = m_new_obj(mp_obj_encoder_t);
    self->base.type = type;
    self->pio = pio0;
    self->sm = 0;
    self->pin_ab = mp_obj_get_int(args[0]);

    // Load and init PIO program
    uint offset = pio_add_program(self->pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(self->pio, self->sm, self->pin_ab, offset);

    return MP_OBJ_FROM_PTR(self);
}

// MicroPython method: read()
STATIC mp_obj_t encoder_read(mp_obj_t self_in) {
    mp_obj_encoder_t *self = MP_OBJ_TO_PTR(self_in);
    int32_t count = quadrature_encoder_get_count(self->pio, self->sm);
    return mp_obj_new_int(count);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(encoder_read_obj, encoder_read);

// Method table
STATIC const mp_rom_map_elem_t encoder_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&encoder_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(encoder_locals_dict, encoder_locals_dict_table);

// Type definition
STATIC const mp_obj_type_t encoder_type = {
    { &mp_type_type },
    .name = MP_QSTR_Encoder,
    .make_new = encoder_make_new,
    .locals_dict = (mp_obj_dict_t*)&encoder_locals_dict,
};

// Module globals
STATIC const mp_rom_map_elem_t encoder_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_encoder) },
    { MP_ROM_QSTR(MP_QSTR_Encoder), MP_ROM_PTR(&encoder_type) },
};
STATIC MP_DEFINE_CONST_DICT(encoder_module_globals, encoder_module_globals_table);

const mp_obj_module_t encoder_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&encoder_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_encoder, encoder_user_cmodule, MODULE_ENCODER_ENABLED);

