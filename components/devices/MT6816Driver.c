/*MT6816�����ļ�*/

#include "MT6816Driver.h"
#include "encoder_cali.h"
#include "stockpile_config.h"
#include "main.h"

//��������������λ��(Quick)(Flash)
uint16_t *Read_QuickCali_DATA = (uint16_t*)STOCKPILE_APP_CALI_ADDR;


extern SPI_HandleTypeDef hspi1;
MT6816_SPI_Signal_Typedef	mt6816_spi;
MT6816_Typedef	mt6816;

static void MT6816_SPI_CS_L(void)
{
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET);
}

static void MT6816_SPI_CS_H(void)
{
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  MT6816��ʼ��
  * @param  NULL
  * @retval NULL
**/
void REIN_MT6816_Init(void)
{
	//�ɼ�����
	mt6816_spi.sample_data = 0;
	//�������
	mt6816_spi.angle = 0;
	
	//����Ƭѡ
	MT6816_SPI_CS_H();

	//��ʼ���׶λ�ȡһ�νǶ�����(���˴�������)(��δ������λ���һ�ζ�ȡ���ݶ�ʧ��ԭ��)
	REIN_MT6816_Get_AngleData();
	
	//���У׼�������Ƿ���Ч
	mt6816.rectify_valid = 1;
	for(uint16_t i=0; i<(CALI_Encode_Res); i++){
		if(Read_QuickCali_DATA[i] == 0xFFFF)
			mt6816.rectify_valid = 0;
	}
}

/**
  * @brief  MT6816_SPI�ɼ���ȡ�Ƕ�����
  * @param  NULL
  * @retval NULL
**/

void RINE_MT6816_SPI_Get_AngleData(void)
{
	uint16_t data_t[2];
	uint16_t data_r[2];
	uint8_t h_count;
	
	data_t[0] = (0x80 | 0x03) << 8;
	data_t[1] = (0x80 | 0x04) << 8;
	
	for(uint8_t i=0; i<3; i++){
		//��ȡSPI����
		MT6816_SPI_CS_L();
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, 1000);
		MT6816_SPI_CS_H();
		MT6816_SPI_CS_L();
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1],1, 1000);
		MT6816_SPI_CS_H();
		mt6816_spi.sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
		
		//��żУ��
		h_count = 0;
		for(uint8_t j=0; j<16; j++){
			if(mt6816_spi.sample_data & (0x0001 << j))
				h_count++;
		}
		if(h_count & 0x01){
			mt6816_spi.pc_flag = 0;
		}
		else{
			mt6816_spi.pc_flag = 1;
			break;
		}
	}
	
	if(mt6816_spi.pc_flag){
		mt6816_spi.angle = mt6816_spi.sample_data >> 2;
		mt6816_spi.no_mag_flag = (uint8_t)(mt6816_spi.sample_data & (0x0001 << 1));
	}
}

/**
  * @brief  MT6816��ȡ�Ƕ�����
  * @param  NULL
  * @retval NULL
**/
void REIN_MT6816_Get_AngleData(void)
{
	RINE_MT6816_SPI_Get_AngleData();
	mt6816.angle_data = mt6816_spi.angle;
	mt6816.rectify_angle = Read_QuickCali_DATA[mt6816.angle_data];
}



