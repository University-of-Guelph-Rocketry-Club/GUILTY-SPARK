

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
