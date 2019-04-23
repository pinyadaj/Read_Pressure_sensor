/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "Dps310.h"

#include "nrf_delay.h"

#include "math.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

 /* Number of possible TWI addresses. */
// #define TWI_ADDRESSES      127
#define PRESSURE_ADDRESS 			0x77

#define i2c_command_example
//#define i2c_background_example

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//Prototype functions.
uint32_t i2c_write_byte(uint8_t reg_addr, uint8_t data);
uint32_t i2c_read_byte(uint8_t reg_addr, uint8_t *data);
uint32_t i2c_read_block(uint8_t reg_addr, uint8_t len, uint8_t *buf_data);
void delay_ms(uint32_t ms_time);

dps310_i2c_init_t dps310_init_func = {
	.write_func = i2c_write_byte,
	.read_func = i2c_read_byte,
	.read_block_func = i2c_read_block,
	.delay_ms_func = delay_ms
};


/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void delay_ms(uint32_t ms_time)
{
	nrf_delay_ms(ms_time);
}

uint32_t i2c_write_byte(uint8_t reg_addr, uint8_t data) 
{
    uint8_t tx_data[2] = { reg_addr, data };

    return nrf_drv_twi_tx(&m_twi, PRESSURE_ADDRESS, tx_data, 2, false);
}


uint32_t i2c_read_byte(uint8_t reg_addr, uint8_t *data) 
{
    ret_code_t err_code;
	
    err_code = nrf_drv_twi_tx(&m_twi, PRESSURE_ADDRESS, &reg_addr, 1, true);
    if (err_code != NRF_SUCCESS) return err_code;
    
    err_code = nrf_drv_twi_rx(&m_twi, PRESSURE_ADDRESS, data, 1);
    
    return err_code;
}

uint32_t i2c_read_block(uint8_t reg_addr, uint8_t len, uint8_t *buf_data) 
{
    ret_code_t err_code;
	  if(buf_data == NULL){
			return NRF_ERROR_NULL;
		}

    err_code = nrf_drv_twi_tx(&m_twi, PRESSURE_ADDRESS, &reg_addr, 1, true);
    if (err_code != NRF_SUCCESS) return err_code;
	
	
	  for(uint8_t count = 0; count < len; count++){
			err_code = nrf_drv_twi_rx(&m_twi, PRESSURE_ADDRESS, &buf_data[count], 1);
		}
    
    return err_code;
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
//    ret_code_t err_code;
//    uint8_t address;
//    uint8_t sample_data;
//    bool detected_device = false;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Pressure application started.");
    NRF_LOG_FLUSH();
    twi_init();

#if 0	
	//Test read product ID
	uint8_t id = 0;
  err_code = i2c_read_byte(0x0D, &id);
	NRF_LOG_INFO("err_code:: %d", err_code);
	if(err_code == NRF_SUCCESS){
			NRF_LOG_INFO("Product REV_ID:: %d, PROD_ID:: %d", id >> 4, id & 0x0F);
			NRF_LOG_FLUSH();
	}
#endif

#if 0	
	 //Test read coeffs
	 uint8_t buf_data[20] = {0};
	 err_code = i2c_read_block(0x10, 18, buf_data);
	 
	 if(err_code == NRF_SUCCESS){
		 for(uint8_t i = 0; i < 18; i++){
			 NRF_LOG_INFO("0x%0X", buf_data[i]);
			 NRF_LOG_FLUSH();
		 }
	 }
#endif
	
#if 0
    for (address = 0; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address); 
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.");
        NRF_LOG_FLUSH();
    }
#endif


    #ifdef i2c_command_example
		int32_t temperature;
		int32_t pressure;
		double _pressure;
//		int32_t last_pressure;
//		int32_t diff;
//    double mbar;
    double altitude;
		double last_altitude = 0;
		static uint8_t flag_altitude = 0;
		int16_t oversampling = 7;
		int16_t ret;
		
		
		dps310_init(dps310_init_func);
    #endif
		
		#ifdef i2c_background_example
		
		uint8_t pressureCount = 20;
		int32_t pressure[pressureCount];
		uint8_t temperatureCount = 20;
		int32_t temperature[temperatureCount];
		
		//Call begin to initialize Dps310PressureSensor
		//The parameter 0x76 is the bus address. The default address is 0x77 and does not need to be given.
		//Dps310PressureSensor.begin(Wire, 0x76);
		//Use the commented line below instead to use the default I2C address.
		dps310_init(dps310_init_func);


		//temperature measure rate (value from 0 to 7)
		//2^temp_mr temperature measurement results per second
		int16_t temp_mr = 2;
		//temperature oversampling rate (value from 0 to 7)
		//2^temp_osr internal temperature measurements per result
		//A higher value increases precision
		int16_t temp_osr = 2;
		//pressure measure rate (value from 0 to 7)
		//2^prs_mr pressure measurement results per second
		int16_t prs_mr = 2;
		//pressure oversampling rate (value from 0 to 7)
		//2^prs_osr internal pressure measurements per result
		//A higher value increases precision
		int16_t prs_osr = 2;
		//startMeasureBothCont enables background mode
		//temperature and pressure ar measured automatically
		//High precision and hgh measure rates at the same time are not available.
		//Consult Datasheet (or trial and error) for more information
		int16_t ret = dps310_startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
		//Use one of the commented lines below instead to measure only temperature or pressure
		//int16_t ret = Dps310PressureSensor.startMeasureTempCont(temp_mr, temp_osr);
		//int16_t ret = Dps310PressureSensor.startMeasurePressureCont(prs_mr, prs_osr);
		
		if (ret != 0)
		{
			NRF_LOG_INFO("Init FAILED! ret = %d", ret);
		}
		else
		{
			NRF_LOG_INFO("Init complete!");
		}
		NRF_LOG_FLUSH();
		#endif
    

     
		
		//NRF_LOG_INFO("Product REV_ID:: %d, PROD_ID:: %d", dps310_getRevisionId(), dps310_getProductId());
		//NRF_LOG_FLUSH();

    while (true)
    {
        /* Empty loop. */
			
		#ifdef i2c_command_example
			//lets the Dps310 perform a Single temperature measurement with the last (or standard) configuration
			//The result will be written to the paramerter temperature
			//ret = Dps310PressureSensor.measureTempOnce(temperature);
			//the commented line below does exactly the same as the one above, but you can also config the precision
			//oversampling can be a value from 0 to 7
			//the Dps 310 will perform 2^oversampling internal temperature measurements and combine them to one result with higher precision
			//measurements with higher precision take more time, consult datasheet for more information
			ret = dps310_measureTempOnce(&temperature, oversampling);
			
			if (ret != 0)
			{
				//Something went wrong.
				//Look at the library code for more information about return codes
				NRF_LOG_INFO("FAIL! ret = %d", ret);
			}
			else
			{
				//NRF_LOG_INFO("Temperature: %d degrees of Celsius", temperature);
			}
			
			//Pressure measurement behaves like temperature measurement
			//ret = Dps310PressureSensor.measurePressureOnce(pressure);
			//ret = dps310_measurePressureOnce(&pressure, oversampling);
			ret = dps310_decimal_measurePressureOnce(&_pressure, oversampling);
			if (ret != 0)
			{
				//Something went wrong.
				//Look at the library code for more information about return codes
				NRF_LOG_INFO("FAIL! ret = %d", ret);
			}
			else
			{
				//NRF_LOG_INFO("Pressure: %ld Pascal", pressure);
				
				//mbar = (double)pressure / 100;
				//mbar = _pressure;
//				NRF_LOG_ERROR( "Altitude " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(_pressure));
				altitude = 44330 * (1.0 - pow( (_pressure / 101325) ,0.1903));
			
//				NRF_LOG_ERROR( "Altitude " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(altitude));
				NRF_LOG_INFO("%d %d %d",temperature, (uint32_t)(_pressure*100), (uint32_t)(altitude*100));
				
//				if(flag_altitude == 0){
//					last_altitude = altitude;
//					flag_altitude = 1;
//				}else{
//					NRF_LOG_INFO("Higth: %ld ", abs((int32_t)(last_altitude*100) - (int32_t)(altitude*100))  );
//				}
//				diff = last_pressure - pressure;
//				last_pressure = pressure;
//				//NRF_LOG_ERROR( "Float " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(diff));
//				NRF_LOG_INFO("Pressure: %ld ", diff);
//				if(h > 0){
//					NRF_LOG_INFO("h: %ld cm", h);
//				}
			}
			
			NRF_LOG_FLUSH();
			nrf_delay_ms(100);
			
			
		 #endif
		 
		 #ifdef i2c_background_example
			//This function writes the results of continuous measurements to the arrays given as parameters
			//The parameters temperatureCount and pressureCount should hold the sizes of the arrays temperature and pressure when the function is called
			//After the end of the function, temperatureCount and pressureCount hold the numbers of values written to the arrays
			//Note: The Dps310 cannot save more than 32 results. When its result buffer is full, it won't save any new measurement results
			int16_t ret = dps310_getContResults(temperature, &temperatureCount, pressure, &pressureCount);

			if (ret != 0)
			{
				NRF_LOG_INFO("\n\nFAIL! ret = %d", ret);
			}
			else
			{
				NRF_LOG_INFO("\n\n%d", temperatureCount);

				NRF_LOG_INFO(" temperature values found: ");
				for (int16_t i = 0; i < temperatureCount; i++)
				{
					NRF_LOG_INFO("%d", temperature[i]);
					NRF_LOG_INFO(" degrees of Celsius");
				}

				
				NRF_LOG_INFO("\n\n%d", pressureCount);
				
			
				NRF_LOG_INFO(" pressure values found: ");
				for (int16_t i = 0; i < pressureCount; i++)
				{
					NRF_LOG_INFO("%d", pressure[i]);
					NRF_LOG_INFO(" Pascal");
				}
			}
     NRF_LOG_FLUSH();
		 nrf_delay_ms(10000);
			
		 #endif
			
			
			
			
			
    }
}

/** @} */
