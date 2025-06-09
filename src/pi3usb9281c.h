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
 * @brief Device register addresses
 *
 * List of all accessible registers in the PI3USB9281C device.
 * These registers control device operation and provide status information.
 */
enum pi3usb9281c_register {
    PI3USB9281C_REGISTER_ID = 0x01,              //!< Device ID register
    PI3USB9281C_REGISTER_CONTROL = 0x02,         //!< Control register
    PI3USB9281C_REGISTER_INTERRUPT = 0x03,       //!< Interrupt status register
    PI3USB9281C_REGISTER_INTERRUPT_MASK = 0x05,  //!< Interrupt mask register
    PI3USB9281C_REGISTER_DEVICE_TYPE = 0x0A,     //!< Connected device type register
    PI3USB9281C_REGISTER_VBUS_DETECT = 0x0B,     //!< VBUS detection register
    PI3USB9281C_REGISTER_CHARGER_STATUS = 0x0E,  //!< Charger status register
    PI3USB9281C_REGISTER_MANUAL_SWITCH = 0x13,   //!< Manual D+/D- switch control register
    PI3USB9281C_REGISTER_RESET = 0x1B,           //!< Software reset register
    PI3USB9281C_REGISTER_VBUS = 0x1D,            //!< VBUS status register
};

/**
 * @brief D+/D- Switch control states
 *
 * Controls how the internal D+/D- switch operates:
 * - Auto: Switch controlled by device based on detected USB/charger type
 * - Manual Open: Force switch to disconnected state
 * - Manual Closed: Force switch to connected state
 */
enum pi3usb9281c_switch_state {
    PI3USB9281C_SWITCH_STATE_AUTO,
    PI3USB9281C_SWITCH_STATE_MANUAL_OPEN,
    PI3USB9281C_SWITCH_STATE_MANUAL_CLOSED,
};

/**
 * @brief Types of USB devices and chargers that can be detected
 *
 * Represents the different types of USB ports and charging devices that
 * the PI3USB9281C can identify, including standard USB ports (SDP/CDP/DCP)
 * and various charging capabilities.
 */
enum pi3usb9281c_device_type {
    PI3USB9281C_DEVICE_TYPE_UNKNOWN,
    PI3USB9281C_DEVICE_TYPE_USB_SDP,       //!< Standard Downstream Port, up to 500mA
    PI3USB9281C_DEVICE_TYPE_USB_CDP,       //!< Charging Downstream Port, up to 1.5A
    PI3USB9281C_DEVICE_TYPE_USB_DCP,       //!< Dedicated Charging Port, beyond 1.5A
    PI3USB9281C_DEVICE_TYPE_CHARGER_1A,    //!< 1.0 Amps charger
    PI3USB9281C_DEVICE_TYPE_CHARGER_2A,    //!< 2.0 Amps charger
    PI3USB9281C_DEVICE_TYPE_CHARGER_2_4A,  //!< 2.4 Amps charger
    PI3USB9281C_DEVICE_TYPE_CARKIT1,       //!< CarKit type 1 accessory
    PI3USB9281C_DEVICE_TYPE_CARKIT2,       //!< CarKit type 2 accessory
};

/**
 * @brief PI3USB9281C USB charger detection IC driver class
 *
 * This class provides an interface to the PI3USB9281C IC, which can detect various
 * types of USB ports and charging devices. The IC communicates over I2C and provides
 * functionality to:
 * - Detect USB port types (SDP/CDP/DCP)
 * - Identify charging capabilities
 * - Control D+/D- switch routing
 * - Monitor device attachment
 */
class pi3usb9281c {
   public:
    //!@{
    //! Initialization and setup
    int setup(TwoWire &i2c_library, const uint8_t i2c_address, const int pin_res);
    bool detect(void);
    int reset(void);
    //!@}

    //!@{
    //! Device detection and monitoring
    int device_attach_wait(const uint32_t timeout_ms);
    int device_type_get(enum pi3usb9281c_device_type *const type);
    //!@}

    //!@{
    //! Switch control
    int switch_state_set(const enum pi3usb9281c_switch_state state);
    //!@}

    //!@{
    //! Register access
    int register_read(const enum pi3usb9281c_register reg_address, uint8_t *const reg_content);
    int register_write(const enum pi3usb9281c_register reg_address, const uint8_t reg_content);
    //!@}

   protected:
    TwoWire *m_i2c_library = NULL;  //!< Pointer to I2C library instance
    uint8_t m_i2c_address = 0;      //!< Device I2C address
};

#endif
