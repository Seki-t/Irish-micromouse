#include"IMU.h"
#include"Debug.h"
#include"xprintf.h"
#include"GlobalParameter.h"

extern SPI_HandleTypeDef hspi1;

/* private method */
uint8_t read_spi(uint8_t);
uint8_t write_spi(uint8_t,uint8_t);
void incrementalReadSPI(uint8_t);
void updateOmega();
void IMUCalibration();

typedef union {
	int16_t data;
	uint8_t reg8[2];
}gyro_data;

gyro_data outX,outY,outZ;
IMUState imu_state = IMU_STATE_CALIBRATION;
GyroState gyro_state;

int8_t imu_is_read = 0;

uint8_t write_spi(uint8_t address,uint8_t wdata){

	uint8_t tx_data[2], rx_data[2];

	//Read operation
	tx_data[0] = 0x00 | address;	//Read
	tx_data[1] = wdata;	//Read

	rx_data[0] = 0x00;
	rx_data[1] = 0x00;

  //CSPIn -> 0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//atode CS PIN no bangou siraberu
	__HAL_SPI_ENABLE(&hspi1);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);	//atode CS PIN no bangou siraberu

	HAL_SPI_TransmitReceive(&hspi1,tx_data,rx_data,2,100);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//atode CS PIN no bangou siraberu

    //xprintf("tx,%x,%x\r\n",rx_data[0],rx_data[1]);

	return rx_data[0];
}

uint8_t read_spi(uint8_t address){

	uint8_t tx_data[4], rx_data[4];
	uint8_t buf_size = 2;

	//Read operation
	tx_data[0] = 0x80 | address;	//Read
	tx_data[1] = 0xff;	//Read

	rx_data[0] = 0x00;
	rx_data[1] = 0x00;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//atode CS PIN no bangou siraberu
	__HAL_SPI_ENABLE(&hspi1);

  //CSPIn -> 0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);	//atode CS PIN no bangou siraberu

	HAL_SPI_TransmitReceive(&hspi1,tx_data,rx_data,buf_size,100);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//atode CS PIN no bangou siraberu

    //xprintf("rx,%d\r\n",(int16_t)rx_data[1]);
	return rx_data[0];
}

void incrementalReadSPI(uint8_t address){

	int8_t tx_data[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	int8_t rx_data[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	//Read operation
	tx_data[0] = 0xc0 | address;	//Read

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//atode CS PIN no bangou siraberu
	//__HAL_SPI_ENABLE(&hspi1);

  //CSPIn -> 0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);	//atode CS PIN no bangou siraberu

	HAL_SPI_TransmitReceive(&hspi1,tx_data,rx_data,7,100);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//atode CS PIN no bangou siraberu

	outX.reg8[0] = rx_data[1];
	outX.reg8[1] = rx_data[2];

	outY.reg8[0] = rx_data[3];
	outY.reg8[1] = rx_data[4];
	
	outZ.reg8[0] = rx_data[5];
	outZ.reg8[1] = rx_data[6];

	//outX.data = - outX.data;
	//outY.data = - outY.data;
	outZ.data = - outZ.data;	// Match gyro direction with omega from encoder
    //xprintf("x,%d,%d,%d\r\n",(int16_t)rx_data[1],(int16_t)rx_data[2],outX.data);
    //xprintf("y,%d,%d,%d\r\n",(int16_t)rx_data[3],(int16_t)rx_data[4],outY.data);
    //xprintf("z,%d,%d,%d\r\n",(int16_t)rx_data[5],(int16_t)rx_data[6],outZ.data);
}

void IMUUpdate(){

	incrementalReadSPI(0x28);

	switch(imu_state){

		case IMU_STATE_CALIBRATION:
			IMUCalibration();
			break;
		
		case IMU_STATE_NORMAL:
			updateOmega();
			imu_is_read = 1;
			break;

		default:
			break;
	}

}

void IMUCalibration(){
	//static int16_t ox_sum = 0;
	//static int16_t oy_sum = 0;
	static int16_t oz_sum = 0;
	static int16_t calibration_count = 0;
	const int16_t calib_max_num = 20;

	//ox_sum += outX.data;
	//oy_sum += outY.data;
	oz_sum += outZ.data;
	calibration_count++;
	if (calibration_count >= calib_max_num)
	{
		//gyro_state.ox_offset = ox_sum / calib_max_num;
		//gyro_state.oy_offset = oy_sum / calib_max_num;
		gyro_state.oz_offset = ((float)oz_sum) / calib_max_num;

		imu_state = IMU_STATE_NORMAL;
		//ox_sum = 0;
		//oy_sum = 0;
		oz_sum = 0;
		calibration_count = 0;
	}
}

void updateOmega(){
	//int16_t omega_x = outX.data - gyro_state.ox_offset;
	//int16_t omega_y = outY.data - gyro_state.oy_offset;
	float omega_z = ((float)(outZ.data) - gyro_state.oz_offset) * GYRO_GAIN;

	//gyro_state.acc.x = (omega_x - gyro_state.omega.x) / GYRO_UPDATE_PERIOD;
	//gyro_state.acc.y = (omega_y - gyro_state.omega.y) / GYRO_UPDATE_PERIOD;
	gyro_state.acc.z = (omega_z - gyro_state.omega.z) / GYRO_UPDATE_PERIOD;

	//gyro_state.theta.x = gyro_state.theta.x + 0.5f * (omega_x + gyro_state.omega.x) * GYRO_UPDATE_PERIOD;
	//gyro_state.theta.y = gyro_state.theta.y + 0.5f * (omega_y + gyro_state.omega.y) * GYRO_UPDATE_PERIOD;
	gyro_state.theta.z = gyro_state.theta.z + 0.5f * (omega_z + gyro_state.omega.z)* GYRO_UPDATE_PERIOD;

	//gyro_state.omega.x = omega_x;
	//gyro_state.omega.y = omega_y;
	gyro_state.omega.z = omega_z;
    
}

void IMUInitialize(){
  read_spi(0x0f);
  read_spi(0x0f);
  write_spi(0x20, 0x0f);

  gyro_state.theta.x = 0.0f;
  gyro_state.theta.y = 0.0f;
  gyro_state.theta.z = 0.0f;
  
  gyro_state.omega.x = 0.0f;
  gyro_state.omega.y = 0.0f;
  gyro_state.omega.z = 0.0f;
  
  gyro_state.acc.x = 0.0f;
  gyro_state.acc.y = 0.0f;
  gyro_state.acc.z = 0.0f;

  gyro_state.ox_offset = 0;
  gyro_state.oy_offset = 0;
  gyro_state.oz_offset = 0;

  imu_state = IMU_STATE_CALIBRATION;
}

Attitude IMUGetAllAttitude(){ return gyro_state.theta;}
Attitude IMUGetAllOmega(){ return gyro_state.omega;}

float IMUGetAttitude(){ return gyro_state.theta.z;}
float IMUGetAngularVelocity(){ return gyro_state.omega.z;}

int8_t IMUIsReadEnable(){
	int8_t tmp = imu_is_read;
	imu_is_read = 0;
	return tmp;
}
