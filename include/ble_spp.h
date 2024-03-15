/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_gap_ble_api.h"



/*
 * DEFINES
 ****************************************************************************************
 */

#define spp_sprintf(s, ...) sprintf((char *)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN (512)
#define SPP_CMD_MAX_LEN (20)
#define SPP_STATUS_MAX_LEN (20)
#define SPP_DATA_BUFF_MAX_LEN (2 * 1024)
/// Attributes State Machine
enum
{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

    SPP_IDX_NB,
};

/**
 * @brief Dose all the initializations and started the BLE communication process.
 * 
 * Initialize the Bluedroid stack, UART drivers and starts the BLE advertisement and BLE communication.
 */
void ble_spp_init();

/**
 * @brief Set UUID of the device
 * 
 * @param UUID This is an 16 bit UUID. This will be set as UUID of your BLE device
 * 
 * This functions sets the UUID of the BLE devices. This is an optional function.
 * If not called the default UUID that will be set is 0x03, 0x03, 0xF0, 0xAB .
 * This function call should be made before call the init function. 
 */
void set_UUID(uint8_t UUID[]);             

/**
 * @brief Set name of device in advertisement.
 * 
 * @param device_name This is the char array of 13 elements.
 * 
 * This function sets the name of the devices and that updated name will appear
 * in the advertisement of the BLE device. 
 * This function call should be made before call the init function. 
 */
void set_device_name(char device_name[]); 

/**
 * @brief Set the generic access name.
 * 
 * @param name It is a character array. 
 * 
 * This function will overwrite the name of the devices in details of Generic Access Profile.
 * It is an optional function call. If it is made called than the default value set will be EmumbaSPPPort.
 * This function call should be made before call the init function. 
 */
void set_generic_access_name(char name[]);

/**
 * @brief Get the received buffer data. 
 * 
 * @return It return a char* which will be pointing to the latest value presented in the receive buffer.
 */
char *get_received_buffer_data();
