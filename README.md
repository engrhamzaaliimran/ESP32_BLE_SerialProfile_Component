# Component BLE SPP (Serial Profile)

For BLE systems, an adopted SPP profile over BLE is not defined, thus emulation of a serial port must be implemented as a vendor-specific custom profile. This components implements a serial profile using GATT (Generic Attribute Profile). It creates a GATT service by adding attributes one by one as defined by Bluedroid. It receives data from the connected devices and forward that to UART.

## API functions details
The details of all API functions are given below

1. void ble_spp_init()

Initialize the Bluedroid stack, UART drivers and starts the BLE advertisement and BLE communication.

2. void set_UUID(uint8_t UUID[])

This functions sets the UUID of the BLE devices. This is an optional function. If not called the default UUID that will be set is **0x03, 0x03, 0xF0, 0xAB**. This function call should be made before call the init function. The parameter *UUID[]*  is a 16 bit array that will contain custom UUID.

3. void set_device_name(char device_name[])

This function sets the name of the devices and that updated name will appear in the advertisement of the BLE device. This function call should be made before call the init function. The parameter *device_name* This is the char array of 13 elements.

4. void set_generic_access_name(char name[])

This function will overwrite the name of the devices in details of Generic Access Profile. It is an optional function call. If it is made called than the default value set will be **EmumbaSPPPort**. This function call should be made before call the init function. The parameter *name* is a character array. 

5. char *get_received_buffer_data()

This function returns a character array pointer which will be pointing to the latest value presented in the receive buffer.

