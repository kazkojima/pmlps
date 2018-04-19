/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#if CONFIG_VL53L1X_ENABLE

#include "vl53l1_api.h"

VL53L1_Dev_t dev;
VL53L1_DEV Dev = &dev;

extern bool range_enable;
extern int range_milli;

/* Autonomous ranging loop*/
static void
AutonomousLowPowerRangingTest(void)
{
    static VL53L1_RangingMeasurementData_t RangingData;
    printf("Autonomous Ranging Test\n");
    int status = VL53L1_WaitDeviceBooted(Dev);
    status = VL53L1_DataInit(Dev);
    status = VL53L1_StaticInit(Dev);
    status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 100000);
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 1000);
    status = VL53L1_StartMeasurement(Dev);

    if(status) {
        printf("VL53L1_StartMeasurement failed \n");
        vTaskDelete(NULL);
    }	

    float val;
    if (0/*isInterrupt*/) {
    } else {
        do // polling mode
            {
                status = VL53L1_WaitMeasurementDataReady(Dev);
                if(!status) {
                    status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
                    if(status==0) {
                        range_milli = RangingData.RangeMilliMeter;
                        //printf("%4d\n", range_milli);
                    }
                    if (!range_enable) {
                        VL53L1_StopMeasurement(Dev);
                        while(!range_enable)
                           vTaskDelay(200 / portTICK_PERIOD_MS);
                    }
                    status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
                }
            }
        while(1);
    }
}

static int i2c_handle = CONFIG_I2C_NUM;

void
rn_task(void *pvParameters)
{
    vTaskDelay(200 / portTICK_PERIOD_MS);

    Dev->I2cHandle = &i2c_handle;
    Dev->I2cDevAddr = 0x52;

    uint8_t byteData;
    uint16_t wordData;

    VL53L1_RdByte(Dev, 0x010F, &byteData);
    printf("VL53L1X Model_ID: %02X\n\r", byteData);
    VL53L1_RdByte(Dev, 0x0110, &byteData);
    printf("VL53L1X Module_Type: %02X\n\r", byteData);
    VL53L1_RdWord(Dev, 0x010F, &wordData);
    printf("VL53L1X: %02X\n\r", wordData);

    AutonomousLowPowerRangingTest();

    while(1) {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
#endif // CONFIG_VL53L1X_ENABLE
