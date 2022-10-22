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
#define DEVICEADDRESS_READ ((uint8_t)(0b01010000))
#define EXAMPLE_TAG             "I2C"
uint8_t deviceadress = 49;


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

esp_err_t eeprom_write_byte(uint8_t deviceaddress, uint16_t eeaddress, uint8_t byte) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_WRITE_ADDR, 1);//last 1 is acknoledment bool (1 or 0)
    i2c_master_write_byte(cmd, eeaddress>>8, 1);
    i2c_master_write_byte(cmd, eeaddress&0xFF, 1);
    i2c_master_write_byte(cmd, byte, 1);
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
    deviceadress = DEVICEADDRESS_READ;
    uint8_t write_value = 55;

    for (uint8_t i = deviceadress; i < (deviceadress + 8); i++) {
        ESP_LOGI(EXAMPLE_TAG, "device - id = %d", i);
        write_value = i;
        ESP_LOGI(EXAMPLE_TAG, " write= %d",write_value);
        eeprom_write_byte(0x50, i, write_value);
        ESP_LOGI(EXAMPLE_TAG, "Writting is complete.");
        //vTaskDelay(10000 / portTICK_PERIOD_MS);
        vTaskDelay(100);
        uint8_t value = eeprom_read_byte(0x50,i);
        ESP_LOGI(EXAMPLE_TAG, "re-read= %d", value);
        ESP_LOGI(EXAMPLE_TAG, "Reading is complete");
  }
  //Serial.println("end");
}