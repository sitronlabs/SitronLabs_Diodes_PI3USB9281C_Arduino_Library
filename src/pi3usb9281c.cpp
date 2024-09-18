/* Self header */
#include "pi3usb9281c.h"

/* Config */
#ifndef CONFIG_PI3USB9281C_LOG_FUNCTION
#define CONFIG_PI3USB9281C_LOG_FUNCTION(...) (void)0  //!< Replace by { Serial.printf(__VA_ARGS__); Serial.println(); } to output on serial port
#endif

/**
 *
 * @param[in] i2c_library
 * @param[in] i2c_address
 * @param[in] pin_enb
 */
int pi3usb9281c::setup(TwoWire& i2c_library, const uint8_t i2c_address, const int pin_enb) {

    /* Ensure i2c address is valid */
    if (i2c_address != 0x25) {
        return -EINVAL;
    }

    /* Save parameters */
    m_i2c_library = &i2c_library;
    m_i2c_address = i2c_address;

    /* Enable */
    pinMode(pin_enb, OUTPUT);
    digitalWrite(pin_enb, LOW);

    /* Return success */
    return 0;
}

/**
 *
 */
bool pi3usb9281c::detect(void) {
    uint8_t reg_id;
    if (register_read(PI3USB9281C_REGISTER_ID, &reg_id) < 0 || reg_id != 0b00011000) {
        return false;
    } else {
        return true;
    }
}

/**
 *
 */
int pi3usb9281c::reset(void) {

    /* */
    uint8_t reg_reset = 0x01;
    if (register_write(PI3USB9281C_REGISTER_RESET, reg_reset) < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @param[in] timeout_ms
 * @return
 */
int pi3usb9281c::device_attach_wait(const uint32_t timeout_ms) {

    /* While timeout has not expired */
    for (uint32_t time_start = millis();;) {

        /* Watch for timeout */
        if (millis() - time_start >= timeout_ms) {
            return -ETIMEDOUT;
        }

        /* Read interrupt flags */
        uint8_t reg_interrupt = 0;
        if (register_read(PI3USB9281C_REGISTER_INTERRUPT, &reg_interrupt) < 0) {
            return -EIO;
        }

        /* Check accessory attached interrupt flag */
        if (reg_interrupt & 0b1) {

            /* Clear interrupt flag */
            reg_interrupt = 0b1;
            if (register_write(PI3USB9281C_REGISTER_INTERRUPT, reg_interrupt) < 0) {
                return -EIO;
            }

            /* Return success */
            return 0;
        }
    }
}

/**
 *
 */
int pi3usb9281c::device_type_get(enum pi3usb9281c_device_type* const type) {

    /* Read registers */
    uint8_t reg_device_type = 0;
    uint8_t reg_charger_type = 0;
    if ((register_read(PI3USB9281C_REGISTER_DEVICE_TYPE, &reg_device_type) < 0) ||
        (register_read(PI3USB9281C_REGISTER_CHARGER_STATUS, &reg_charger_type) < 0)) {
        CONFIG_PI3USB9281C_LOG_FUNCTION("Failed to read device type!");
        return -EIO;
    }

    /*  */
    if (reg_charger_type & (1 << 4)) {
        *type = PI3USB9281C_DEVICE_TYPE_CHARGER_2_4A;
    } else if (reg_charger_type & (1 << 3)) {
        *type = PI3USB9281C_DEVICE_TYPE_CHARGER_2A;
    } else if (reg_charger_type & (1 << 2)) {
        *type = PI3USB9281C_DEVICE_TYPE_CHARGER_1A;
    } else if ((reg_charger_type & 0b11) == 0b11) {
        *type = PI3USB9281C_DEVICE_TYPE_CARKIT2;
    } else if ((reg_charger_type & 0b11) == 0b10) {
        *type = PI3USB9281C_DEVICE_TYPE_CARKIT1;
    } else if (reg_device_type & (1 << 6)) {
        *type = PI3USB9281C_DEVICE_TYPE_USB_DCP;
    } else if (reg_device_type & (1 << 5)) {
        *type = PI3USB9281C_DEVICE_TYPE_USB_CDP;
    } else if (reg_device_type & (1 << 2)) {
        *type = PI3USB9281C_DEVICE_TYPE_USB_SDP;
    } else {
        *type = PI3USB9281C_DEVICE_TYPE_UNKNOWN;
    }

    /* Return success */
    return 0;
}

/**
 *
 */
int pi3usb9281c::switch_state_set(const enum pi3usb9281c_switch_state state) {

    /* Ensure state is valid */
    if (state != PI3USB9281C_SWITCH_STATE_AUTO &&
        state != PI3USB9281C_SWITCH_STATE_MANUAL_OPEN &&
        state != PI3USB9281C_SWITCH_STATE_MANUAL_CLOSED) {
        return -EINVAL;
    }

    /* Write registers */
    uint8_t reg_control = 0b00011011 | (state == PI3USB9281C_SWITCH_STATE_AUTO ? 1 << 2 : 0);
    uint8_t reg_manual_switch = 0b00000000 | (state == PI3USB9281C_SWITCH_STATE_MANUAL_CLOSED ? 0b00100111 : 0);
    if (register_write(PI3USB9281C_REGISTER_CONTROL, reg_control) < 0 ||
        register_write(PI3USB9281C_REGISTER_MANUAL_SWITCH, reg_manual_switch) < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * Reads the contents of the given register.
 * @param[in] reg_address The address of the register.
 * @param[out] reg_content A pointer to a variable that will be updated with the contents of the register.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int pi3usb9281c::register_read(const enum pi3usb9281c_register reg_address, uint8_t* const reg_content) {
    int res;

    /* Ensure library has been configured */
    if (m_i2c_library == NULL) {
        return -EINVAL;
    }

    /* Send register address */
    m_i2c_library->beginTransmission(m_i2c_address);
    m_i2c_library->write(reg_address);
    res = m_i2c_library->endTransmission(false);
    if (res != 0) {
        return -EIO;
    }

    /* Read data */
    m_i2c_library->requestFrom(m_i2c_address, (uint8_t)1, (uint8_t) true);
    res = m_i2c_library->available();
    if (res == 0) {
        return -EIO;
    }
    *reg_content = m_i2c_library->read();

    /* Return success */
    return 0;
}

/**
 * Updates the content of the given register.
 * @param[in] reg_address The address of the register.
 * @param[in] reg_content The new content of the register.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int pi3usb9281c::register_write(const enum pi3usb9281c_register reg_address, const uint8_t reg_content) {
    int res;

    /* Ensure library has been configured */
    if (m_i2c_library == NULL) {
        return -EINVAL;
    }

    /* Send register address and data */
    m_i2c_library->beginTransmission(m_i2c_address);
    m_i2c_library->write(reg_address);
    m_i2c_library->write(reg_content);
    res = m_i2c_library->endTransmission(true);
    if (res != 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}
