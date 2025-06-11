/* Self header */
#include "pi3usb9281c.h"

/* Config */
#ifndef CONFIG_PI3USB9281C_LOG_FUNCTION
#define CONFIG_PI3USB9281C_LOG_FUNCTION(...) (void)0  //!< Replace by { Serial.printf(__VA_ARGS__); Serial.println(); } to output on serial port
#endif

/**
 * @brief Initializes the PI3USB9281C device
 *
 * This function sets up the I2C communication with the device and configures the enable pin.
 * The device only supports I2C address 0x25.
 *
 * @param[in] i2c_library Reference to the TwoWire I2C library instance to use
 * @param[in] i2c_address I2C address of the device (must be 0x25)
 * @param[in] pin_enb GPIO pin number connected to the device's enable pin
 * @return 0 on success, -EINVAL if invalid I2C address
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
 * @brief Detects if a PI3USB9281C device is present
 *
 * Reads the device ID register and verifies it matches the expected value (0x18).
 *
 * @return true if device is detected, false otherwise
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
 * @brief Performs a software reset of the device
 *
 * Writes to the reset register to trigger a device reset.
 *
 * @return 0 on success, -EIO on I2C communication error
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
 * @brief Waits for a device attachment event
 *
 * Polls the interrupt register for the device attachment flag.
 * When detected, clears the interrupt flag.
 *
 * @param[in] timeout_ms Maximum time to wait in milliseconds
 * @return 0 on device attached, -ETIMEDOUT on timeout, -EIO on I2C error
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
 * @brief Checks for device attachment (non-blocking)
 *
 * Reads the interrupt register once to check for device attachment.
 * If a device is attached, clears the interrupt flag.
 *
 * @return 1 if device attached, 0 if no device attached, -EIO on I2C error
 */
int pi3usb9281c::device_attach_get(void) {
	
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

        /* Return device attached */
        return 1;
    }

    /* Return no device attached */
    return 0;
}

/**
 * @brief Gets the type of attached USB device or charger
 *
 * Reads the device type and charger status registers to determine what kind of
 * device is attached. Can detect various USB ports (SDP, CDP, DCP) and charger types.
 *
 * @param[out] type Pointer to store the detected device type
 * @return 0 on success, -EIO on I2C communication error
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
 * @brief Controls the state of the D+/D- switch
 *
 * Sets the switch to either automatic control, manually open, or manually closed.
 * In automatic mode, the device controls the switch based on detected device type.
 * Manual modes allow forcing the switch state regardless of device type.
 *
 * @param[in] state Desired switch state (auto, manual open, or manual closed)
 * @return 0 on success, -EINVAL for invalid state, -EIO on I2C error
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
 * @brief Reads a device register
 *
 * Performs an I2C read transaction to get the contents of the specified register.
 *
 * @param[in] reg_address Register address to read
 * @param[out] reg_content Pointer to store the register value
 * @return 0 on success, -EINVAL if not initialized, -EIO on I2C error
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
    m_i2c_library->requestFrom(m_i2c_address, (uint8_t)1, (uint8_t)true);
    res = m_i2c_library->available();
    if (res == 0) {
        return -EIO;
    }
    *reg_content = m_i2c_library->read();

    /* Return success */
    return 0;
}

/**
 * @brief Writes to a device register
 *
 * Performs an I2C write transaction to update the specified register.
 *
 * @param[in] reg_address Register address to write
 * @param[in] reg_content Value to write to the register
 * @return 0 on success, -EINVAL if not initialized, -EIO on I2C error
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
