/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-06-10     RT-Thread    first version
 */

#include <rtthread.h>


#include <stdlib.h>
#include "board.h"
#include "sensor.h"
#include "sensor_dallas_ds18b20.h"


#include "mb.h"
#include "mb_m.h"



extern int onenet_mqtt_init(void);
extern rt_err_t onenet_mqtt_upload_digit(const char *ds_name, const double digit);

extern ADC_HandleTypeDef hadc1;
extern void MX_ADC1_Init(void);


#define DBG_TAG "main"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define ADC_CH_NMBR     1       //ADC CHANNEL NUMBR

//---------------global variables------------------
struct rt_sensor_data sensor_data;                    //ds18b20 data
uint32_t  adc_conv_result;                            //adc conversion result
extern USHORT   usMRegHoldBuf[16][100];               //modbusRTU 传感器
//---------------private function define-----------
static void read_ds18b20_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;

    rt_size_t res;

    dev = rt_device_find(parameter);
    if (dev == RT_NULL)
    {
        LOG_D("Can't find device:%s\n", parameter);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_D("open device failed!\n");
        return;
    }
    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)100);

    while (1)
    {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1)
        {
            LOG_D("read data failed!size is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else
        {
            if (sensor_data.data.temp >= 0)
            {
                LOG_D("temp:%3d.%dC, timestamp:%5d\n",
                           sensor_data.data.temp / 10,
                           sensor_data.data.temp % 10,
                           sensor_data.timestamp);
            }
            else
            {
                LOG_D("temp:-%2d.%dC, timestamp:%5d\n",
                           abs(sensor_data.data.temp / 10),
                           abs(sensor_data.data.temp % 10),
                           sensor_data.timestamp);
            }
        }
        rt_thread_mdelay(1000);
    }
}



/* upload random value to temperature*/
static void adc1_poolForConv_entry(void *parameter)
{
    //------------------------ADC  convert-----------
    MX_ADC1_Init();

    while(1){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1,100);
        adc_conv_result = HAL_ADC_GetValue(&hadc1);

//        LOG_I("adc1 result = %d", adc_conv_result[0]);

//
//        if (onenet_mqtt_upload_digit("lightStr", adc_conv_result[0]) < 0)
//        {
//            LOG_E("upload has an error, stop uploading");
//            break;
//        }


        rt_thread_mdelay(1000);


    }

}

//-----------------modbus rtu-----------------------
#define SLAVE_ADDR      MB_SAMPLE_TEST_SLAVE_ADDR
#define PORT_NUM        MB_MASTER_USING_PORT_NUM
#define PORT_BAUDRATE   MB_MASTER_USING_PORT_BAUDRATE

#define PORT_PARITY     MB_PAR_EVEN

#define MB_POLL_THREAD_PRIORITY  16
#define MB_SEND_THREAD_PRIORITY  RT_THREAD_PRIORITY_MAX - 1

#define MB_SEND_REG_START  0
#define MB_SEND_REG_NUM    2

#define MB_POLL_CYCLE_MS   500


static void rs485_master_poll(void *parameter)
{
    eMBMasterInit(MB_RTU, PORT_NUM, PORT_BAUDRATE, PORT_PARITY);
    eMBMasterEnable();

    while (1)
    {
        eMBMasterPoll();
        rt_thread_mdelay(MB_POLL_CYCLE_MS);
    }
}




static void rs485_send_thread_entry(void *parameter)
{
    eMBMasterReqErrCode error_code = MB_MRE_NO_ERR;
    rt_uint16_t error_count = 0;

    while (1)
    {
        /* Test Modbus Master */
        rt_thread_mdelay(2000);
        error_code = eMBMasterReqReadHoldingRegister( SLAVE_ADDR, 0, 2, 200);
        rt_kprintf("humi=%d, temp=%d\r\n", usMRegHoldBuf[0][0],usMRegHoldBuf[0][1]);

//        rt_thread_mdelay(2000);
//        error_code = eMBMasterReqReadHoldingRegister( 2, 0, 2, 200);
//        rt_kprintf("humi=%d, temp=%d\r\n", usMRegHoldBuf[2-1][0],usMRegHoldBuf[2-1][1]);
//
//        rt_thread_mdelay(2000);
//        error_code = eMBMasterReqReadHoldingRegister( 3, 0, 2, 200);
//        rt_kprintf("humi=%d, temp=%d\r\n", usMRegHoldBuf[3-1][0],usMRegHoldBuf[3-1][1]);

//
//        if (onenet_mqtt_upload_digit("air_temp", usMRegHoldBuf[0][1]) < 0)
//        {
//            LOG_E("upload has an error, stop uploading");
//            break;
//        }
//        rt_thread_mdelay(1000);
//        if (onenet_mqtt_upload_digit("air_humi", usMRegHoldBuf[0][0]) < 0)
//        {
//            LOG_E("upload has an error, stop uploading");
//            break;
//        }



        /* Record the number of errors */
        if (error_code != MB_MRE_NO_ERR)
        {
            error_count++;
        }
    }
}


static int mb_master_start(void)
{
    static rt_uint8_t is_init = 0;
    rt_thread_t tid1 = RT_NULL, tid2 = RT_NULL;

    if (is_init > 0)
    {
        rt_kprintf("sample is running\n");
        return -RT_ERROR;
    }
    tid1 = rt_thread_create("rs485Poll", rs485_master_poll, RT_NULL, 1024, MB_POLL_THREAD_PRIORITY, 10);
    if (tid1 != RT_NULL)
    {
        rt_thread_startup(tid1);
    }


    tid2 = rt_thread_create("rs485Send", rs485_send_thread_entry, RT_NULL, 512, MB_POLL_THREAD_PRIORITY+1, 10);
    if (tid2 != RT_NULL)
    {
        rt_thread_startup(tid2);
    }


    return RT_EOK;

}


/* upload random value to temperature*/
static void onenet_sensordata_upload_entry(void *parameter)
{
    float value;

    while (1)
    {
        //--------upload ds18b20 sensor data---------
        value = sensor_data.data.temp / 10 + sensor_data.data.temp % 10;         //ds18b20温度
        if (onenet_mqtt_upload_digit("temperature", value) < 0)
        {
            LOG_E("upload has an error, stop uploading");
            break;
        }
        else
        {
            LOG_D("buffer : {\"temperature\":%f}", value);
        }
        rt_thread_mdelay((2 * 1000));
        //--------upload adc1 sensor data---------
        value = adc_conv_result/0.01;//*(4095/3.3);                                     //ADC1 voltage
        if (onenet_mqtt_upload_digit("adc1Vol", value) < 0)
        {
            LOG_E("upload has an error, stop uploading");
            break;
        }
        else
        {
            LOG_D("buffer : {\"adc1Vol\":%f}", value);
        }

        rt_thread_mdelay((2 * 1000));
        //--------modbus 485 sensor data upload---------
        value = usMRegHoldBuf[0][1]*0.1;                                     //air temp
        if (onenet_mqtt_upload_digit("air_temp", value) < 0)
        {
            LOG_E("upload has an error, stop uploading");
            break;
        }
        else
        {
            LOG_D("buffer : {\"air_temp\":%f}", value);
        }
        rt_thread_mdelay((2 * 1000));

        value = usMRegHoldBuf[0][0]*0.1;                                     //air humi
        if (onenet_mqtt_upload_digit("air_humi", value) < 0)
        {
            LOG_E("upload has an error, stop uploading");
            break;
        }
        else
        {
            LOG_D("buffer : {\"air_humi\":%f}", value);
        }






        rt_thread_mdelay((5 * 1000));
    }
}


//----------------------------------------------------------------
int main(void)
{



    //--------------------------DS18B20 sensor read thread------------------------------------

    rt_thread_t ds18b20_thread;

    ds18b20_thread = rt_thread_create("18b20TempRead",
                                      read_ds18b20_entry,
                                      "temp_ds18b20",
                                      1024,
                                      RT_THREAD_PRIORITY_MAX / 2,
                                      20);
    if (ds18b20_thread != RT_NULL)
    {
        rt_thread_startup(ds18b20_thread);
    }
    //--------------------------------------------------------------


    //-------------onenet--------------

    onenet_mqtt_init();
    rt_thread_mdelay(3000);

    rt_thread_t cloud_tid;
    cloud_tid = rt_thread_create("onenet_senddata",
                                 onenet_sensordata_upload_entry,
                                 RT_NULL,
                                 2 * 1024,
                                 RT_THREAD_PRIORITY_MAX / 3 - 1,
                                 5);
    if (cloud_tid)
    {
        rt_thread_startup(cloud_tid);
    }

   //-------------

    rt_thread_t adc_conv_tid;

    adc_conv_tid = rt_thread_create("adc_conv",
                                     adc1_poolForConv_entry,
                                     RT_NULL,
                                     2 * 1024,
                                     RT_THREAD_PRIORITY_MAX / 2+1,
                                     20);
    if (adc_conv_tid != RT_NULL)
    {
        rt_thread_startup(adc_conv_tid);
    }




    //--------------start modbus polling thread-----------------

//    rt_thread_mdelay(3000);
    mb_master_start();




    return RT_EOK;
}
