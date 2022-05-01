/**
 * @file	vl53l0x.c
 * @author	Arnaud C.
 * @date	23.04.2022
 * @brief
 */

#include "vl53l0x.h"

/* Variables ----------------------------------------------------------------*/
VL53L0X_Dev_t dev[VL53L0X_MAX_NB];
vl53l0x_Cfg cfg[VL53L0X_MAX_NB]; 	// library sensor configurations

VL53L0X_Error status = 0;
uint8_t nb_tof = 0;

/* Prototypes ---------------------------------------------------------------*/
VL53L0X_Error _vl53l0x_Device_Initialization(uint8_t tof_id);
VL53L0X_Error _vl53l0x_Calibration_Data_Load(uint8_t tof_id);
VL53L0X_Error _vl53l0x_System_Settings(uint8_t tof_id);
VL53L0X_Error _vl53l0x_Range_Profiles(uint8_t tof_id);

/* Functions ----------------------------------------------------------------*/

/**
 * @brief Initialization of vl53l0x_Cfg structures
 *
 * @param i2c			i2c handler
 * @param xshut			xshut pin info
 * @param nb_sensors	number of sensors
 * @return VL53L0X_Error
 */
VL53L0X_Error vl53l0x_begin(I2C_HandleTypeDef *i2c, xshut *xshut,
		int nb_sensors)
{
	status = VL53L0X_ERROR_NONE;
	nb_tof = nb_sensors;

	// Structure clearing
	for(int i=0; i<VL53L0X_MAX_NB; i++){
		cfg[i].pin_xshut.GPIOx = NULL;
		cfg[i].pin_xshut.GPIO_Pin = 0;
	}

	// Structure filling
	for(int i=0; i<nb_sensors; i++){
		cfg[i].dev = &dev[i];
		cfg[i].dev->I2cHandle = i2c;
		cfg[i].dev->I2cDevAddr = 0x52;
		cfg[i].dev->comms_speed_khz = 400;
		cfg[i].dev->comms_type = 1;

		cfg[i].pin_xshut = xshut[i];
		HAL_GPIO_WritePin(cfg[i].pin_xshut.GPIOx, cfg[i].pin_xshut.GPIO_Pin,
				GPIO_PIN_RESET);
	}

	return status;
}

/**
 * @brief Initialization of sensors
 *
 * @return VL53L0X_Error
 */
VL53L0X_Error vl53l0x_init()
{
	printf("Initialization started for %d vl53l0x sensor(s)\r\n\n", nb_tof);
	HAL_Delay(250);

	for(int j=0; j<nb_tof; j++)
	{
		// Xshut pin drive to high state
		HAL_GPIO_WritePin(cfg[j].pin_xshut.GPIOx, cfg[j].pin_xshut.GPIO_Pin,
				GPIO_PIN_SET);
		HAL_Delay(250);

		// Device initialization
		if((status = _vl53l0x_Device_Initialization(j)) != VL53L0X_ERROR_NONE){
			printf("Error vl53l0x_Device_Initialization() : %d\r\n", status);
			return status;
		}
		printf("Device initialization done...\r\n");

		// Calibration
		if((status = _vl53l0x_Calibration_Data_Load(j)) != VL53L0X_ERROR_NONE){
			printf("Error _vl53l0x_Calibration_Data_Load() : %d\r\n", status);
			return status;
		}
		printf("Calibration data loaded...\r\n");

		// System settings
		if((status = _vl53l0x_System_Settings(j)) != VL53L0X_ERROR_NONE){
			printf("Error _vl53l0x_System_Settings() : %d\r\n", status);
			return status;
		}
		printf("System settings applied...\r\n");

		// Set new i2c address
		if((status = VL53L0X_SetDeviceAddress(cfg[j].dev, 0x52 + (j+1) * 2))
				!= VL53L0X_ERROR_NONE){
			printf("Error VL53L0X_SetDeviceAddress() : %d\r\n", status);
			return status;
		}
		cfg[j].dev->I2cDevAddr = 0x52 + (j+1) * 2;
		printf("New i2c addr: 0x%2X\r\n", cfg[j].dev->I2cDevAddr);

		HAL_Delay(250);
	}

	// Start measurement
	for(int j=0; j<nb_tof; j++){
		VL53L0X_StartMeasurement(cfg[j].dev);
	}

	return status;
}

/**
 * @brief Device initialization
 *
 * @param tof_id Sensor index
 * @return VL53L0X_Error
 */
VL53L0X_Error _vl53l0x_Device_Initialization(uint8_t tof_id)
{
	// Data init
	if((status = VL53L0X_DataInit(cfg[tof_id].dev)) != VL53L0X_ERROR_NONE){
		printf("Error DataInit() : %d\r\n",status);
		return status;
	}

	// Static init
	if((status = VL53L0X_StaticInit(cfg[tof_id].dev)) != VL53L0X_ERROR_NONE){
		printf("Error StaticInit() : %d\r\n",status);
		return status;
	}

	return status;
}

/**
 * @brief Calibration
 *
 * @param tof_id Sensor index
 * @return VL53L0X_Error
 */
VL53L0X_Error _vl53l0x_Calibration_Data_Load(uint8_t tof_id)
{
	uint32_t count; uint8_t isApertureSpads;
	if((status = VL53L0X_PerformRefSpadManagement(cfg[tof_id].dev, &count,
			&isApertureSpads)) != VL53L0X_ERROR_NONE){
		printf("Error VL53L0X_PerformRefSpadManagement() : %d\r\n",status);
		return status;
	}

	uint8_t VhvSettings, PhaseCal;
	if((status = VL53L0X_PerformRefCalibration(cfg[tof_id].dev, &VhvSettings,
			&PhaseCal)) != VL53L0X_ERROR_NONE){
		printf("Error VL53L0X_PerformRefCalibration() : %d\r\n",status);
		return status;
	}

	return status;
}

/**
 * @brief System settings definiton
 *
 * @param tof_id Sensor index
 * @return VL53L0X_Error
 */
VL53L0X_Error _vl53l0x_System_Settings(uint8_t tof_id)
{

	// Device Mode
	if((status = VL53L0X_SetDeviceMode(cfg[tof_id].dev,
			VL53L0X_DEVICEMODE_CONTINUOUS_RANGING)) != VL53L0X_ERROR_NONE){
		printf("Error VL53L0X_SetDeviceMode() : %d\r\n",status);
		return status;
	}

	// API range profiles
	if((status = _vl53l0x_Range_Profiles(tof_id)) != VL53L0X_ERROR_NONE){
		printf("Error _vl53l0x_Range_Profiles() : %d\r\n",status);
		return status;
	}

	// Polling or interrupt mode
	if((status = VL53L0X_SetGpioConfig(cfg[tof_id].dev, 0,
			VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
			VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
			VL53L0X_INTERRUPTPOLARITY_HIGH)) != VL53L0X_ERROR_NONE){
		printf("Error VL53L0X_SetGpioConfig() : %d\r\n",status);
		return status;
	}
	else VL53L0X_ClearInterruptMask(cfg[tof_id].dev, 0);

	return status;
}

/**
 * @brief Default range profiles
 *
 * @param tof_id Sensor index
 * @return VL53L0X_Error
 */
VL53L0X_Error _vl53l0x_Range_Profiles(uint8_t tof_id)
{
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

	switch(RANGE_PROFILE_SELECTED)
	{
	case LONG_RANGE:
		signalLimit = (FixPoint1616_t)(0.1*65536);
		sigmaLimit = (FixPoint1616_t)(60*65536);
		timingBudget = 33000;
		preRangeVcselPeriod = 18;
		finalRangeVcselPeriod = 14;
		break;
	case HIGH_ACCURACY:
		signalLimit = (FixPoint1616_t)(0.25*65536);
		sigmaLimit = (FixPoint1616_t)(18*65536);
		timingBudget = 200000;
		preRangeVcselPeriod = 14;
		finalRangeVcselPeriod = 10;
		break;
	case HIGH_SPEED:
		signalLimit = (FixPoint1616_t)(0.25*65536);
		sigmaLimit = (FixPoint1616_t)(32*65536);
		timingBudget = 20000;
		preRangeVcselPeriod = 14;
		finalRangeVcselPeriod = 10;
		break;
	default:
		printf("Not Supported");
	}

	VL53L0X_SetLimitCheckValue(cfg[tof_id].dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
	VL53L0X_SetLimitCheckValue(cfg[tof_id].dev,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(cfg[tof_id].dev,
			timingBudget);
	VL53L0X_SetVcselPulsePeriod(cfg[tof_id].dev,
			VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
	VL53L0X_SetVcselPulsePeriod(cfg[tof_id].dev,
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);

	return status;
}

/**
 * @brief Perform ranging measurement
 *
 * @param tof_id Sensor index
 * @return range
 */
uint16_t vl53l0x_PerformRangingMeasurement(uint8_t tof_id){
	VL53L0X_RangingMeasurementData_t VL53L0X_RangingMeasurementData;

	//VL53L0X_WaitDeviceReadyForNewMeasurement(cfg[tof_id].dev, 100);
	VL53L0X_GetRangingMeasurementData(cfg[tof_id].dev,
			&VL53L0X_RangingMeasurementData);
	VL53L0X_ClearInterruptMask(cfg[tof_id].dev, 0);

	return VL53L0X_RangingMeasurementData.RangeMilliMeter;
}
