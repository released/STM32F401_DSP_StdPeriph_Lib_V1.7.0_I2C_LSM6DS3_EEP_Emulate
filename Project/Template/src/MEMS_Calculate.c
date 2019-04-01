/* Includes ------------------------------------------------------------------*/
#include "MEMS_Calculate.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*angle variable*/
#define PI (float)3.14159265f
float RollAng = 0.0f, PitchAng = 0.0f;

#define FILTER_COUNT  	(16)
#define FILTER_FACTOR  	(4)

#define	GYRO_SCALE 	(1)	//(70)

int16_t ax_buf[FILTER_COUNT], ay_buf[FILTER_COUNT],az_buf[FILTER_COUNT];
int16_t gx_buf[FILTER_COUNT], gy_buf[FILTER_COUNT],gz_buf[FILTER_COUNT];
int16_t gx, gy, gz, ax ,ay, az;
static float angle, angle_dot, f_angle, f_angle_dot;

/*MEMS calibration*/
uint8_t Flag_Calibrate = 0;

/*EEPROM variable*/
/*	EEPROM address pre-define
	check NB_OF_VAR in eeprom.h
*/
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x0000, 0x0001, 0x0002, 0x0003, 
										0x0004,	0x0005, 0x0006, 0x0007, 
										0x0008, 0x0009, 0x000A, 0x000B,
										0x000C, 0x000D, 0x000E, 0x000F,
										0x0010, 0x0011, 0x0012, 0x0013, 
										0x0014,	0x0015, 0x0016, 0x0017, 
										0x0018, 0x0019, 0x001A, 0x001B,
										0x001C, 0x001D, 0x001E, 0x001F};

#define	GYRO_INT_X  		(VirtAddVarTab[0x00])
#define	GYRO_AXIS_X 		(VirtAddVarTab[0x01])
#define	GYRO_INT_Y			(VirtAddVarTab[0x02])	
#define	GYRO_AXIS_Y 		(VirtAddVarTab[0x03])
#define	GYRO_INT_Z			(VirtAddVarTab[0x04])	
#define	GYRO_AXIS_Z 		(VirtAddVarTab[0x05])

#define	ACC_INT_X 			(VirtAddVarTab[0x06])	
#define	ACC_AXIS_X 			(VirtAddVarTab[0x07])	
#define	ACC_INT_Y 			(VirtAddVarTab[0x08])		
#define	ACC_AXIS_Y 			(VirtAddVarTab[0x09])	
#define	ACC_INT_Z 			(VirtAddVarTab[0x0A])		
#define	ACC_AXIS_Z 			(VirtAddVarTab[0x0B])	

/*misc marco*/
#define	MEMSABS(X)			((X) >= 0 ? (X) : -(X))
#define	MEMSCONVERTA(x,y)	(y = (x>=0)?(1):(0))
#define	MEMSCONVERTB(x,y)	(x = (y == 1)?(-x):(x))

extern uint8_t Flag_Button;

/* Private functions ---------------------------------------------------------*/

void Gyroscope_Calibration(void)
{
	int32_t gyroX = 0;
	int32_t gyroY = 0;
	int32_t gyroZ = 0;		

	uint16_t integerX = 0;
	uint16_t integerY = 0;	
	uint16_t integerZ = 0;	
	
	if (Flag_Calibrate)
	{
		gyroX = appLSM6DS3_GetGyroData(AXIS_X);
		if (MEMSABS(gyroX)>0)
		{
			appLSM6DS3_SetGyroCalData(AXIS_X,-gyroX);

			#if defined (ENABLE_EEP_CALIBRATE)
			EE_WriteVariable(GYRO_AXIS_X,MEMSABS(gyroX));	
			EE_WriteVariable(GYRO_INT_X,MEMSCONVERTA(gyroX,integerX));
			#endif			
		}
		gyroY = appLSM6DS3_GetGyroData(AXIS_Y);
		if (MEMSABS(gyroY)>0)
		{
			appLSM6DS3_SetGyroCalData(AXIS_Y,-gyroY);

			#if defined (ENABLE_EEP_CALIBRATE)
			EE_WriteVariable(GYRO_AXIS_Y,MEMSABS(gyroY));	
			EE_WriteVariable(GYRO_INT_Y,MEMSCONVERTA(gyroY,integerY));
			#endif			
		}
		gyroZ = appLSM6DS3_GetGyroData(AXIS_Z);
		if (MEMSABS(gyroZ)>0)
		{
			appLSM6DS3_SetGyroCalData(AXIS_Z,-gyroZ);

			#if defined (ENABLE_EEP_CALIBRATE)
			EE_WriteVariable(GYRO_AXIS_Z,MEMSABS(gyroZ));	
			EE_WriteVariable(GYRO_INT_Z,MEMSCONVERTA(gyroZ,integerZ));
			#endif			
		}
		
//		printf("%s : %4d,%4d,%4d\r\n",__FUNCTION__,gyroX,gyroY,gyroZ);
	}
}

void Accelerator_Calibration(void)
{
	int32_t accX = 0;
	int32_t accY = 0;
	int32_t accZ = 0;		

	uint16_t integerX = 0;
	uint16_t integerY = 0;	
	uint16_t integerZ = 0;		
	
	if (Flag_Calibrate)
	{
		appLSM6DS3_SetAccCalData(AXIS_X,0);	//reset calibration data
		accX = appLSM6DS3_GetAccData(AXIS_X);
		if (MEMSABS(accX)>0)
		{
			appLSM6DS3_SetAccCalData(AXIS_X,-accX);

			#if defined (ENABLE_EEP_CALIBRATE)
			EE_WriteVariable(ACC_AXIS_X,MEMSABS(accX));	
			EE_WriteVariable(ACC_INT_X,MEMSCONVERTA(accX,integerX));
			#endif			
		}

		appLSM6DS3_SetAccCalData(AXIS_Y,0);		//reset calibration data
		accY = appLSM6DS3_GetAccData(AXIS_Y);		
		if (MEMSABS(accY)>0)
		{
			appLSM6DS3_SetAccCalData(AXIS_Y,-accY);

			#if defined (ENABLE_EEP_CALIBRATE)
			EE_WriteVariable(ACC_AXIS_Y,MEMSABS(accY));	
			EE_WriteVariable(ACC_INT_Y,MEMSCONVERTA(accY,integerY));
			#endif			
		}

		appLSM6DS3_SetAccCalData(AXIS_Z,0);		//reset calibration data
		accZ = appLSM6DS3_GetAccData(AXIS_Z);		
		if ((MEMSABS(accZ)>1000)||(MEMSABS(accZ)<=999))
		{
			appLSM6DS3_SetAccCalData(AXIS_Z,-accZ+1000);

			#if defined (ENABLE_EEP_CALIBRATE)
			EE_WriteVariable(ACC_AXIS_Z,MEMSABS(accZ));	
			EE_WriteVariable(ACC_INT_Z,MEMSCONVERTA(accZ,integerZ));
			#endif			
		}
		
//		printf("%s : %4d,%4d,%4d\r\n",__FUNCTION__,accX,accY,accZ);
	}
}

void MEMS_Calibration(void)
{	
	uint8_t i = 0;	
	uint16_t resX = 0;
	uint16_t resY = 0;
	uint16_t resZ = 0;	
	uint16_t integerX = 0;
	uint16_t integerY = 0;	
	uint16_t integerZ = 0;		
	static uint8_t flag = 1;

	if (Flag_Calibrate)
	{
		FLASH_Unlock();

		#if 1	//erase	
		for (i=0;i<NB_OF_VAR;i++)
		{
			EE_WriteVariable(VirtAddVarTab[i] , 0xFFFF);
		}
//		#else	//write default
		EE_WriteVariable(GYRO_AXIS_X,0);	
		EE_WriteVariable(GYRO_INT_X,1);
		EE_WriteVariable(GYRO_AXIS_Y,0);	
		EE_WriteVariable(GYRO_INT_Y,1);	
		EE_WriteVariable(GYRO_AXIS_Z,0);	
		EE_WriteVariable(GYRO_INT_Z,1);

		EE_WriteVariable(ACC_AXIS_X,0);	
		EE_WriteVariable(ACC_INT_X,1);
		EE_WriteVariable(ACC_AXIS_Y,0);	
		EE_WriteVariable(ACC_INT_Y,1);	
		EE_WriteVariable(ACC_AXIS_Z,1000);	
		EE_WriteVariable(ACC_INT_Z,1);

		#endif

		Gyroscope_Calibration();
		Accelerator_Calibration();
	
		FLASH_Lock();

		Flag_Calibrate = 0;	
		flag = 1;
	}

	if (flag)
	{
		/*gyro*/
		EE_ReadVariable(GYRO_INT_X,&integerX);
		EE_ReadVariable(GYRO_AXIS_X,&resX);
		EE_ReadVariable(GYRO_INT_Y,&integerY);
		EE_ReadVariable(GYRO_AXIS_Y,&resY);
		EE_ReadVariable(GYRO_INT_Z,&integerZ);
		EE_ReadVariable(GYRO_AXIS_Z,&resZ);

		#if defined (ENABLE_EEP_CALIBRATE)
		appLSM6DS3_SetGyroCalData(AXIS_X,MEMSCONVERTB(resX,integerX));
		appLSM6DS3_SetGyroCalData(AXIS_Y,MEMSCONVERTB(resY,integerY));
		appLSM6DS3_SetGyroCalData(AXIS_Z,MEMSCONVERTB(resZ,integerZ));		
		#endif

		#if 0	//debug
		printf("%s : %5d,%5d,%5d,%5d,%5d,%5d\r\n",__FUNCTION__,integerX,MEMSCONVERTB(resX,integerX),
											integerY,MEMSCONVERTB(resY,integerY),
											integerZ,MEMSCONVERTB(resZ,integerZ));
		#endif

		/*acc*/
		EE_ReadVariable(ACC_INT_X,&integerX);
		EE_ReadVariable(ACC_AXIS_X,&resX);
		EE_ReadVariable(ACC_INT_Y,&integerY);
		EE_ReadVariable(ACC_AXIS_Y,&resY);
		EE_ReadVariable(ACC_INT_Z,&integerZ);
		EE_ReadVariable(ACC_AXIS_Z,&resZ);

		#if defined (ENABLE_EEP_CALIBRATE)
		appLSM6DS3_SetAccCalData(AXIS_X,MEMSCONVERTB(resX,integerX));	
		appLSM6DS3_SetAccCalData(AXIS_Y,MEMSCONVERTB(resY,integerY));
		appLSM6DS3_SetAccCalData(AXIS_Z,MEMSCONVERTB(resZ,integerZ)+1000);
		#endif

		#if 0	//debug
		printf("%s : %5d,%5d,%5d,%5d,%5d,%5d\r\n",__FUNCTION__,integerX,MEMSCONVERTB(resX,integerX),
											integerY,MEMSCONVERTB(resY,integerY),
											integerZ,MEMSCONVERTB(resZ,integerZ)+1000);
		#endif
		
		flag = 0;
	}
	
}

void EmulateEEP_Test(void)
{
	static uint16_t Cnt = 0;
	uint16_t i = 0;	
	uint16_t temp[NB_OF_VAR];

	FLASH_Unlock();

	#if 1	//erase	
	for (i=0;i<NB_OF_VAR;i++)
	{
		EE_WriteVariable(VirtAddVarTab[i] , 0xFFFF);
	}	
	#endif

	#if 1	//write
	for (i=0;i<NB_OF_VAR;i++)
	{
		temp[i] = i+(Cnt+0x10);
		EE_WriteVariable(VirtAddVarTab[i] , temp[i]);	
		_DELAY(10);
	}
	#endif
		
	FLASH_Lock();
	Cnt++;

	#if 1	//read
	MEMSET(temp,0x00,NB_OF_VAR);
	for (i=0;i<NB_OF_VAR;i++)
	{
		EE_ReadVariable(VirtAddVarTab[i] , &temp[i]);	
	}
	#endif
	
	#if 1	//debug
	printf("\r\n\r\n\r\n");
	for(i=0;i<NB_OF_VAR;i++)
	{
		printf("0x%2x, ", temp[i]);
		if ((i+1)%8 ==0)
		{
			printf("\r\n");
		}
	}	
	printf("\r\n\r\n\r\n");	
	#endif
}

void EmulateEEP_Init(void)
{
	/*
		use 2 pages (0x8004000 , 0x8008000) for emulate eeprom 
		check EEPROM_START_ADDRESS  in eeprom.h
	*/

	FLASH_Unlock();
	EE_Init();
//	FLASH_Lock();

	printf("%s\r\n",__FUNCTION__);
}

void Accelerator_filter(void)
{
	#if defined (ENABLE_AVERAGE_FILTER)

	uint8_t i;
	int32_t ax_sum = 0, ay_sum = 0, az_sum = 0; 

	for(i = 1 ; i < FILTER_COUNT; i++)
	{
		ax_buf[i - 1] = ax_buf[i];
		ay_buf[i - 1] = ay_buf[i];
		az_buf[i - 1] = az_buf[i];
	}

	ax_buf[FILTER_COUNT - 1] = appLSM6DS3_GetAccData(AXIS_X);
	ay_buf[FILTER_COUNT - 1] = appLSM6DS3_GetAccData(AXIS_Y);
	az_buf[FILTER_COUNT - 1] = appLSM6DS3_GetAccData(AXIS_Z);

	for(i = 0 ; i < FILTER_COUNT; i++)
	{
		ax_sum += ax_buf[i];
		ay_sum += ay_buf[i];
		az_sum += az_buf[i];
	}

	ax = (int16_t)(ax_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);
	ay = (int16_t)(ay_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);
	az = (int16_t)(az_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);

	#else
	ax = appLSM6DS3_GetAccData(AXIS_X);
	ay = appLSM6DS3_GetAccData(AXIS_Y);
	az = appLSM6DS3_GetAccData(AXIS_Z);

	#endif
	
}

void Gyroscope_filter(void)
{
	#if defined (ENABLE_AVERAGE_FILTER)	
	uint8_t i;
	int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0; 

	for(i = 1 ; i < FILTER_COUNT; i++)
	{
		gx_buf[i - 1] = gx_buf[i];
		gy_buf[i - 1] = gy_buf[i];
		gz_buf[i - 1] = gz_buf[i];
	}

	gx_buf[FILTER_COUNT - 1] = appLSM6DS3_GetGyroData(AXIS_X);
	gy_buf[FILTER_COUNT - 1] = appLSM6DS3_GetGyroData(AXIS_Y);
	gz_buf[FILTER_COUNT - 1] = appLSM6DS3_GetGyroData(AXIS_Z);

	for(i = 0 ; i < FILTER_COUNT; i++)
	{
		gx_sum += gx_buf[i];
		gy_sum += gy_buf[i];
		gz_sum += gz_buf[i];
	}

	gx = (int16_t)(gx_sum>>FILTER_FACTOR);// / FILTER_COUNT);
	gy = (int16_t)(gy_sum>>FILTER_FACTOR);// / FILTER_COUNT);
	gz = (int16_t)(gz_sum>>FILTER_FACTOR);// / FILTER_COUNT);

	#else
	gx = appLSM6DS3_GetGyroData(AXIS_X);
	gy = appLSM6DS3_GetGyroData(AXIS_Y);
	gz = appLSM6DS3_GetGyroData(AXIS_Z);	

	#endif
	
}

void Tilt_Angle_Calculate(void)
{  
	float s1 = 0;
	float s2 = 0;	

	s1 = sqrt((float)((ay *ay )+(az *az )));
	s2 = sqrt((float)((ax *ax )+(az *az )));

	PitchAng = atan(ax /s1)*57.295779;	//*180/PI;
	RollAng = atan(ay /s2)*57.295779;	//*180/PI;

	#if defined (ENABLE_KALMAN_FILTER)
	angle_dot = gx*GYRO_SCALE;	
	kalman_filter(RollAng, angle_dot, &f_angle, &f_angle_dot);
	#endif

	#if 1	//debug

	if (Flag_Button)
	{
		printf("Pitch:%8.3lf,",PitchAng);
		printf("Roll:%8.3lf,",RollAng);
		
		printf("Acc:%5d,%5d,%5d,",ax ,ay ,az );
		printf("Gyro:%5d,%5d,%5d,",gx ,gy ,gz );	
		printf("\r\n");
	}
	else
	{		
		printf("Pitch:%8.3lf,",PitchAng);
		printf("Roll:%8.3lf,",RollAng);

		#if defined (ENABLE_KALMAN_FILTER)
		printf("Angle:%8.3lf,",f_angle);
		#endif
		
		printf("Acc:%5d,%5d,%5d,",ax ,ay ,az );
		printf("Gyro:%5d,%5d,%5d,",gx ,gy ,gz );	
		printf("\r\n");
	}
	
	#endif
}

