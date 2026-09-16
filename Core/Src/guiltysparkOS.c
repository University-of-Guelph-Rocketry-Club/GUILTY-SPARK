#include "guiltysparkOS.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*
This is where code pertaining to scheduling tasks goes
*/

typedef enum{
    SAMPLE_PRESSURE_DATA,
    SAMPLE_ACCELEROMETER_DATA,
    SAMPLE_MAGNETOMETER_DATA
} TASK_LIST;

struct task_setup{
    int cnt_sample_pressure_data;
    int cnt_sample_accelerometer_data;
    int cnt_sample_magnetometer_data;
};

/*
This function will organize tasks into their respective blocks
*/
uint16_t organize_task(struct task_setup state_task_params, int N)
{

}
