/**
 * @file	vl53l0x.h
 * @author	Arnaud C.
 * @date	23.04.2022
 * @brief
 *
 */

/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef INC_VL53L0X_H_
#define INC_VL53L0X_H_

/* Includes -----------------------------------------------------------------*/
#include <stdint.h>
#include "gpio.h"

#include "vl53l0x_api.h"

/* Exported types -----------------------------------------------------------*/
typedef struct VL53L0X_xshut{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;

} xshut;

typedef struct vl53l0x_Cfg{
	VL53L0X_DEV dev;
	xshut pin_xshut;

} vl53l0x_Cfg;

/* End of exported types ----------------------------------------------------*/

/* Exported macros ----------------------------------------------------------*/
#define VL53L0X_MAX_NB	4
#define VL53L0X_DEFAULT_ADDRESS (0x29 << 1)

#define RANGE_PROFILE_SELECTED 	HIGH_ACCURACY
#define LONG_RANGE 				0
#define HIGH_ACCURACY			1
#define HIGH_SPEED				2

/* Prototypes ---------------------------------------------------------------*/
VL53L0X_Error vl53l0x_begin(I2C_HandleTypeDef *i2c, xshut *xshut, int nb_sensors);
VL53L0X_Error vl53l0x_init();
uint16_t vl53l0x_PerformRangingMeasurement(uint8_t tof_id);

#endif /* INC_VL53L0X_H_ */
