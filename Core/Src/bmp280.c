/*
This is where the code for interfacing with the BMP280 barometer.
Functions contained:
    - BMP280_READ_PRESSURE()
    - BMP280_READ_TEMPERATURE()
    - BMP280_READ(uint8_t read_addr)
    - BMP280_WRITE(uint8_t write_addr, uint8_t write_val)
    - BMP280_Config(uint8_t config_byte)

Also contains functions to parse the pressure and temp data
    - BMP280_PARSE_TEMPERATURE(uint8_t raw_temperature)
    - BMP280_PARSE_PRESSURE(uint8_t raw_pressure)

Most of the interfacing with this module is either config or reading from registers.

Note: The HAL SPI functions have bus guards in place already, so we don't need to implement them ourselves
*/
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "bmp280.h"


/*
This will configure the BMP280 registers over SPI.
Configure the ctrl_meas(addr 0xF4) and config(addr 0xF5) with the appropriate settings.
ctrl_meas - 0xF4 params:
    osrs_t[2:0] -> Oversampling setting for temperature
    osrs_p[2:0] -> Oversampling setting for pressure
    mode[1:0] -> Power setting
config - 0xF5 params:
    t_sb[2:0] -> Standby time setting. This will control the time between pressure readings.
    filter[2:0] -> IIR filter settings
    spi3w_en -> bit that enables 3 wire spi mode
Codes for different function outputs:

*/
uint16_t BMP280_CONFIG(struct bmp280_config bmp280_config_params, SPI_HandleTypeDef spi_handle, int* spi_done_flag, uint8_t* BMP280_TX_Buffer, uint8_t* BMP280_RX_Buffer, int BMP280_BUFFER_SIZE)
{
    /*
    Set up the bits in the ctrl_meas register
    */
   uint8_t ctrl_meas = 0b00000000;

   switch (bmp280_config_params.pressure_oversampling)
   {
    case SKIP_MEASURMENT:
        ctrl_meas |= (0b000 << 2);
        break;
    case ULTRA_LOW_POWER:
        ctrl_meas |= (0b001 << 2);
        break;
    case LOW_POWER:
        ctrl_meas |= (0b010 << 2);
        break;
    case STANDARD_RESOLUTION:
        ctrl_meas |= (0b011 << 2);
        break;
    case HIGH_RESOLUTION:
        ctrl_meas |= (0b100 << 2);
        break;
    case ULTRA_HIGH_RESOLUTION:
        ctrl_meas |= (0b101 << 2);
        break;
    default:
        break;
   }

   switch (bmp280_config_params.temperature_oversampling)
   {
    case SKIP_MEASURMENT:
            ctrl_meas |= (0b000 << 5);
            break;
        case ULTRA_LOW_POWER:
            ctrl_meas |= (0b001 << 5);
            break;
        case LOW_POWER:
            ctrl_meas |= (0b010 << 5);
            break;
        case STANDARD_RESOLUTION:
            ctrl_meas |= (0b011 << 5);
            break;
        case HIGH_RESOLUTION:
            ctrl_meas |= (0b100 << 5);
            break;
        case ULTRA_HIGH_RESOLUTION:
            ctrl_meas |= (0b101 << 5);
            break;
        default:
            break;
   }

   switch (bmp280_config_params.power_setting)
   {
    case SLEEP:
        ctrl_meas |= (0b00 << 0);
        break;
    case FORCED:
        ctrl_meas |= (0b01 << 0);
        break;
    case NORMAL:
        ctrl_meas |= (0b11 << 0);
        break;
    default:
        break;
   }

    /*
    Set up the bits in the config register
    */
    uint8_t config = 0b00000000;

    switch (bmp280_config_params.t_standby)
    {
        case _0_5_ms:
            config |= (0b000 << 5);
            break;
        case _62_5_ms:
            config |= (0b001 << 5);
            break;
        case _125_ms:
            config |= (0b010 << 5);
            break;
        case _250_ms:
            config |= (0b011 << 5);
            break;
        case _500_ms:
            config |= (0b100 << 5);
            break;
        case _1000_ms:
            config |= (0b101 << 5);
            break;
        case _2000_ms:
            config |= (0b110 << 5);
            break;
        case _4000_ms:
            config |= (0b111 << 5);
            break;
        default:
            break;
    }

    switch (bmp280_config_params.spi3w_enable)
    {
    case yes:
        config |= (0b1 << 0);
        break;
    case no:
        config |= (0b0 << 0);
        break;
    default:
        break;
    }

    switch (bmp280_config_params.filter_coeff)
    {
    case FILTER_OFF:
        config |= (0b000 << 2);
        break;
    default:
        break;
    }

    BMP280_TX_Buffer[0] = 0x74;
    BMP280_TX_Buffer[1] = ctrl_meas;
    BMP280_TX_Buffer[2] = 0x75;
    BMP280_TX_Buffer[3] = config;

    HAL_StatusTypeDef spi_status;
    //Attempt to send the command, and check the return on the HAL_SPI_DMA function
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive_DMA(&spi_handle, BMP280_TX_Buffer, BMP280_RX_Buffer, BMP280_BUFFER_SIZE);

    switch (spi_status)
    {
    case HAL_OK:
        /* HAL function sent the SPI transaction sucsessfully - now being handled by DMA */
        *spi_done_flag = 0;
        return 0x0001;
    case HAL_BUSY:
        /* SPI bus in use currently */
        return 0x0002;
    case HAL_ERROR:
        /* HAL funnction returned some error */
        return 0x0003;
    case HAL_TIMEOUT:
        /* HAL function returned a timeout error */
        return 0x0004;
    default:
        return 0x0006; //This is the impossible case that should not happen at all, but for pedantics sake we can put it here
    }
}

/*
This has the spi_completed guard and the t_standby_elapsed guard built in, so main.c code stays organized
will return an 8-bit hex value depening on what kind of error there is (if there is any)
Output Codes:
    0x0001 -> HAL transaction sucsessfull,
    0x0002 -> SPI Bus is busy
    0x0003 -> SPI bus returned some error
    0x0004 -> HAL_SPI_TransmitReceive_DMA returned a timeout
    0x0005 -> standby time not elapsed yet
    0x0006 -> HAl function returned nothing at all
*/
uint16_t BMP280_READ(SPI_HandleTypeDef spi_handle, int* spi_done_flag, uint8_t* BMP280_TX_Buffer, uint8_t* BMP280_RX_Buffer, int BMP280_BUFFER_SIZE)
{
    HAL_StatusTypeDef spi_status;
    //Attempt to send the command, and check the return on the HAL_SPI_DMA function
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive_DMA(&spi_handle, BMP280_TX_Buffer, BMP280_RX_Buffer, BMP280_BUFFER_SIZE);

    switch (spi_status)
    {
    case HAL_OK:
        /* HAL function sent the SPI transaction sucsessfully - now being handled by DMA */
        *spi_done_flag = 0;
        return 0x0001;
    case HAL_BUSY:
        /* SPI bus in use currently */
        return 0x0002;
    case HAL_ERROR:
        /* HAL funnction returned some error */
        return 0x0003;
    case HAL_TIMEOUT:
        /* HAL function returned a timeout error */
        return 0x0004;
    default:
        return 0x0006; //This is the impossible case that should not happen at all, but for pedantics sake we can put it here
    }
}

