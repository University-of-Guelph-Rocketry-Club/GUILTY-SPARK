#ifndef __BMP280_H
#define __BMP280_H

typedef enum{
    SKIP_MEASURMENT,
    ULTRA_LOW_POWER,
    LOW_POWER,
    STANDARD_RESOLUTION,
    HIGH_RESOLUTION,
    ULTRA_HIGH_RESOLUTION
} oversampling_setting;

typedef enum{
    SLEEP,
    FORCED,
    NORMAL
}power_mode;

typedef enum{ //Module not taking new measurments during this time, and as such we should not ask it for data here
    _0_5_ms,
    _62_5_ms,
    _125_ms,
    _250_ms,
    _500_ms,
    _1000_ms,
    _2000_ms,
    _4000_ms
}t_standby_options;

typedef enum{
    yes,
    no
}spi3w_en;

typedef enum{
    FILTER_OFF
    /*Aside from this setting I'm not too sure about how this part works*/
}filter_setting;

struct bmp280_config{
    oversampling_setting pressure_oversampling;
    oversampling_setting temperature_oversampling;
    power_mode power_setting;
    t_standby_options t_standby;
    filter_setting filter_coeff;
    spi3w_en spi3w_enable;
};

/* --------------------------------------------------------------- Function Prototypes --------------------------------------------------------------- */
uint16_t BMP280_CONFIG(struct bmp280_config bmp280_config_params, SPI_HandleTypeDef spi_handle, int* spi_done_flag, uint8_t* BMP280_TX_Buffer, uint8_t* BMP280_RX_Buffer, int BMP280_BUFFER_SIZE);

uint16_t BMP280_READ(SPI_HandleTypeDef spi_handle, int* spi_done_flag, uint8_t* BMP280_TX_Buffer, uint8_t* BMP280_RX_Buffer, int BMP280_BUFFER_SIZE);

#endif /* __BMP280_H */
