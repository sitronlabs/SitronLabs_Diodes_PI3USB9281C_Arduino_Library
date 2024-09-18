#ifndef PI3USB9281C_H
#define PI3USB9281C_H

/* Arduino libraries */
#include <Arduino.h>
#include <Wire.h>

/* C/C++ libraries */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

/**
 *
 */
enum pi3usb9281c_register {
    PI3USB9281C_REGISTER_ID = 0x01,
    PI3USB9281C_REGISTER_CONTROL = 0x02,
    PI3USB9281C_REGISTER_INTERRUPT = 0x03,
    PI3USB9281C_REGISTER_INTERRUPT_MASK = 0x05,
    PI3USB9281C_REGISTER_DEVICE_TYPE = 0x0A,
    PI3USB9281C_REGISTER_VBUS_DETECT = 0x0B,
    PI3USB9281C_REGISTER_CHARGER_STATUS = 0x0E,
    PI3USB9281C_REGISTER_MANUAL_SWITCH = 0x13,
    PI3USB9281C_REGISTER_RESET = 0x1B,
    PI3USB9281C_REGISTER_VBUS = 0x1D,
};

/**
 * List of different switch states for the D+ and D- signals
 */
enum pi3usb9281c_switch_state {
    PI3USB9281C_SWITCH_STATE_AUTO,
    PI3USB9281C_SWITCH_STATE_MANUAL_OPEN,
    PI3USB9281C_SWITCH_STATE_MANUAL_CLOSED,
};

/**
 * List of different USB devices the IC can detect
 */
enum pi3usb9281c_device_type {
    PI3USB9281C_DEVICE_TYPE_UNKNOWN,
    PI3USB9281C_DEVICE_TYPE_USB_SDP,       //!< Standard Downstream Port, up to 500mA
    PI3USB9281C_DEVICE_TYPE_USB_CDP,       //!< Charging Downstream Port, up to 1.5A
    PI3USB9281C_DEVICE_TYPE_USB_DCP,       //!< Dedicated Charging Port, beyond 1.5A
    PI3USB9281C_DEVICE_TYPE_CHARGER_1A,    //!< 1.0 Amps charger
    PI3USB9281C_DEVICE_TYPE_CHARGER_2A,    //!< 2.0 Amps charger
    PI3USB9281C_DEVICE_TYPE_CHARGER_2_4A,  //!< 2.4 Amps charger
    PI3USB9281C_DEVICE_TYPE_CARKIT1,       //!<
    PI3USB9281C_DEVICE_TYPE_CARKIT2,       //!<
};

/**
 *
 */
class pi3usb9281c {
   public:
    int setup(TwoWire &i2c_library, const uint8_t i2c_address, const int pin_res);
    bool detect(void);
    int reset(void);
    int device_attach_wait(const uint32_t timeout_ms);
    int device_type_get(enum pi3usb9281c_device_type *const type);
    int switch_state_set(const enum pi3usb9281c_switch_state state);
    int register_read(const enum pi3usb9281c_register reg_address, uint8_t *const reg_content);
    int register_write(const enum pi3usb9281c_register reg_address, const uint8_t reg_content);

   protected:
    TwoWire *m_i2c_library = NULL;
    uint8_t m_i2c_address = 0;
};

#endif
