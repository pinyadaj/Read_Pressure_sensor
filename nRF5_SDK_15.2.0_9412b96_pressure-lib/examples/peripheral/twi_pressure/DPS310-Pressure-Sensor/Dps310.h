/**
 * Arduino library to control Dps310
 *
 * "Dps310" represents Infineon's high-sensetive pressure and temperature sensor. 
 * It measures in ranges of 300 - 1200 hPa and -40 and 85 °C. 
 * The sensor can be connected via SPI or I2C. 
 * It is able to perform single measurements
 * or to perform continuous measurements of temperature and pressure at the same time, 
 * and stores the results in a FIFO to reduce bus communication. 
 *
 * Have a look at the datasheet for more information. 
 */

#ifndef DPS310_H__
#define DPS310_H__

#include <stdint.h>
#include <string.h>
#include "util/dps310_consts.h"


#ifdef __cplusplus
extern "C" {
#endif

enum Mode
{
	IDLE 								= 0x00,
	CMD_PRS 						= 0x01,
	CMD_TEMP 						= 0x02,
	INVAL_OP_CMD_BOTH 	= 0x03,		//invalid
	INVAL_OP_CONT_NONE 	= 0x04, 	//invalid
	CONT_PRS						= 0x05,
	CONT_TMP						=	0x06,
	CONT_BOTH						= 0x07
};

typedef uint32_t (*i2c_write_byte_call_back)(uint8_t reg_addr, uint8_t data);
typedef uint32_t (*i2c_read_byte_call_back)(uint8_t reg_addr, uint8_t *data);
typedef uint32_t (*i2c_read_block_call_back)(uint8_t reg_addr, uint8_t len, uint8_t *buf_data);
typedef void (*delay_ms_call_back)(uint32_t ms_time);

typedef struct{
	i2c_write_byte_call_back 	write_func;
	i2c_read_byte_call_back  	read_func;
	i2c_read_block_call_back  read_block_func;
	delay_ms_call_back				delay_ms_func;
}dps310_i2c_init_t;

		//general
		void dps310_init(dps310_i2c_init_t init_func); //Init by call back functions.
		uint8_t dps310_getProductId(void);
		uint8_t dps310_getRevisionId(void);
    int16_t dps310_readcoeffs(void);

		//measurement
    int16_t dps310_setInterruptPolarity(uint8_t polarity);
		int16_t dps310_setInterruptSources(uint8_t fifoFull, uint8_t tempReady, uint8_t prsReady);

    int16_t dps310_startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate);
    int16_t dps310_startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate);
		int16_t dps310_startMeasureBothCont(uint8_t tempMr,
										 uint8_t tempOsr,
										 uint8_t prsMr,
										 uint8_t prsOsr);

		int16_t dps310_getContResults(int32_t *tempBuffer,
								   uint8_t *tempCount,
								   int32_t *prsBuffer,
								   uint8_t *prsCount);

		int16_t dps310_measureTempOnce(int32_t *result, uint8_t oversamplingRate);
		int16_t dps310_measurePressureOnce(int32_t *result, uint8_t oversamplingRate);
		int16_t dps310_decimal_measurePressureOnce(double *result, uint8_t oversamplingRate);

    int16_t dps310_getTemp(int32_t *result);
		int16_t dps310_getPressure(int32_t *result);
		int16_t dps310_getIntStatusFifoFull(void);
		int16_t dps310_getIntStatusTempReady(void);
		int16_t dps310_getIntStatusPrsReady(void);


#ifdef __cplusplus
}
#endif

#endif	//DPS310_H__
