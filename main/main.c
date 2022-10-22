#include <stdio.h>
#include "driver/i2c.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


// Definitions for i2c
#define I2C_MASTER_SCL_IO   4
#define I2C_MASTER_SDA_IO   5
#define I2C_MASTER_NUM  I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_FREQ_HZ  10000 //previous version 100000 (no higher than 1MHz for now)

#define EEPROM_WRITE_ADDR   0x00
#define EEPROM_READ_ADDR    0x01
#define DEVICEADDRESS_READ  0X50
#define EXAMPLE_TAG             "I2C"
uint8_t deviceadress = 49;

#define EEPROM_ADDRESS 0X50
#define START_EEPROM_ADDRESS  20

#define NUM_PAR_SAVED 24  //note : chnage according to the number of data
#define EEPROM_SIZE 65536

typedef struct
{
uint8_t hv_soc;
uint8_t hv_current;
uint8_t hv_batt_air;
uint8_t hv_block_v[14];
uint8_t hv_power;
uint8_t hv_fan_spd;
uint8_t hv_batt_t[3];
uint8_t hv_num_blocks;
uint8_t vehicle_spd;
}vehicle_data;

vehicle_data data_recorded;

void write_vehicle_data(vehicle_data *data , bool reset);

void init_i2c_master() {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;//====!=====//
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;//====!=====//
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;//====!=====//
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

}


/*
**EEPROM single byte write**
*/

esp_err_t eeprom_write_byte(uint8_t deviceaddress, uint16_t eeaddress, uint8_t data) {
    
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_WRITE_ADDR, 1);//last 1 is acknoledment bool (1 or 0)
    i2c_master_write_byte(cmd, eeaddress>>8, 1);
    i2c_master_write_byte(cmd, eeaddress&0xFF, 1);
    i2c_master_write_byte(cmd, data, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
**EEPROM single page write**
*/

esp_err_t eeprom_write_page(uint8_t deviceaddress, uint16_t eeaddress, uint8_t data[]) {
    
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_WRITE_ADDR, 1);//last 1 is acknoledment bool (1 or 0)
    i2c_master_write_byte(cmd, eeaddress>>8, 1);
    i2c_master_write_byte(cmd, eeaddress&0xFF, 1);

    i2c_master_write(cmd, data, NUM_PAR_SAVED , 1);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/*
**EEPROM single byte Read**
*/

uint8_t eeprom_read_byte(uint8_t deviceaddress, uint16_t eeaddress) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_WRITE_ADDR, 1);
    i2c_master_write_byte(cmd, eeaddress<<8, 1);
    i2c_master_write_byte(cmd, eeaddress&0xFF, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_READ_ADDR, 1);

    uint8_t data;
    i2c_master_read_byte(cmd, &data, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}


void app_main() {
    init_i2c_master();
    ESP_LOGI(EXAMPLE_TAG, "Start");


    data_recorded.hv_soc = 25;
    data_recorded.hv_current = 3;
    data_recorded.hv_batt_air = 12;
    for(uint8_t i =0;i<14; i++)
    {
        data_recorded.hv_block_v[i] = i;
    }
    data_recorded.hv_power = 30;
    data_recorded.hv_fan_spd = 7;
    data_recorded.hv_batt_t[0] = 9;
    data_recorded.hv_batt_t[1] = 10;
    data_recorded.hv_batt_t[2] = 11;
    data_recorded.hv_num_blocks =14;
    data_recorded.vehicle_spd = 120;

    /*
    write_vehicle_data(&data_recorded , 0);
    write_vehicle_data(&data_recorded , 0);
    write_vehicle_data(&data_recorded , 0);
    write_vehicle_data(&data_recorded , 0);
    write_vehicle_data(&data_recorded , 0);

*/


//eeprom_write_byte(EEPROM_ADDRESS, 0, 0);
//eeprom_write_byte(EEPROM_ADDRESS, 1, 0);

    for (uint8_t i = 0; i < NUM_PAR_SAVED * 10; i++) {
        uint8_t value = eeprom_read_byte(EEPROM_ADDRESS, START_EEPROM_ADDRESS + i);
        ESP_LOGI(EXAMPLE_TAG, "data[%d]= %d",i, value);   
    }
    ESP_LOGI(EXAMPLE_TAG, "Reading is complete");  


  //Serial.println("end");
}


void write_vehicle_data(vehicle_data *data , bool reset)
{
    //Note : change size of this array according too the maxximum number of data tobe saved.
    uint8_t array_of_data[NUM_PAR_SAVED];

    memcpy(array_of_data , data , NUM_PAR_SAVED );

    uint16_t pre_address_left = eeprom_read_byte(EEPROM_ADDRESS, 0);
    uint16_t pre_address_right = eeprom_read_byte(EEPROM_ADDRESS, 1);
    uint16_t current_address = ((pre_address_left<<8) | pre_address_right) + NUM_PAR_SAVED  ; 

    if(reset | (current_address + NUM_PAR_SAVED > EEPROM_SIZE) | ((pre_address_left ==0) & (pre_address_right== 0)))
    {
        current_address = START_EEPROM_ADDRESS;
    }

    eeprom_write_page(EEPROM_ADDRESS , current_address , array_of_data );
    
    eeprom_write_byte(EEPROM_ADDRESS , 0 , (current_address>>8) & 0XFF );
    eeprom_write_byte(EEPROM_ADDRESS , 1 , current_address& 0XFF );

}



