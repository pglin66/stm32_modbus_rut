/**
 * @File Name: modbus_slave.h
 * @Brief : ͨ��ģ�� (��վ��
 * @Author : JYK (yakun81131@yeah.net)
 * @Version : 1.4
 * @Creat Date : 2022-03-5
 *
 */

#ifndef __MODBUY_SLAVE_H
#define __MODBUY_SLAVE_H
#include "main.h"

// ***  485 �ӻ� �շ����� ***
#define ModbusS_RXbuffer_size 100
#define ModbusS_TXbuffer_size 1024
extern uint8_t Modbus_Slave_485_RXbuffer[ModbusS_RXbuffer_size];
extern uint8_t Modbus_Slave_485_TXbuffer[ModbusS_TXbuffer_size];

typedef struct
{
	uint8_t Slave_ID;

	UART_HandleTypeDef *pUART_HandleTypeDef_Slave; // ���ں�ָ��
	GPIO_TypeDef *DIR_GPIO_Port;				   // 485ʹ���շ�������
	uint16_t DIR_GPIO_pins;						   // 485ʹ���շ����ź�

	uint8_t *RxBuff;	   // ���ջ���
	__IO uint16_t RxCount; // ���ռ���
	__IO uint8_t RxStatus; // ����״̬

	uint8_t *TxBuff;	   // ���ͻ���
	__IO uint16_t TxCount; // ���ͼ���

	uint16_t RxBuff_Size; // ���ͻ�������
	uint16_t TxBuff_Size; // ���ջ�������

	uint8_t Send_choose; // ����ѡ��

	__IO uint8_t RspCode; // Ӧ�����

	__IO uint8_t Alert_01H;
	__IO uint8_t Alert_02H;
	__IO uint8_t Alert_03H;
	__IO uint8_t Alert_04H;
	__IO uint8_t Alert_05H;
	__IO uint8_t Alert_06H;
	__IO uint8_t Alert_07H;
	__IO uint8_t Alert_10H;
	__IO uint8_t Alert_65H;

} _Modbus_Slave;
extern _Modbus_Slave Modbus_Slave_485;

#define RS485EN_ARE 0 // ��485ʹ��
#define RS485EN_NO 1  // ��485ʹ��

// == Modbus RTU �ӻ� Ӧ����� ==
#define Slave_RSP_OK 0				// �ɹ�
#define Slave_RSP_ERR_CMD 0x01		// ��֧�ֵĹ�����
#define Slave_RSP_ERR_REG_ADDR 0x02 // �Ĵ�����ַ����
#define Slave_RSP_ERR_VALUE 0x03	// ����ֵ�����
#define Slave_RSP_ERR_WRITE 0x04	// д��ʧ��

// == ������ ==
#define Slave_FUNC_01H 0x01 // ������״̬
#define Slave_FUNC_02H 0x02 // ������
#define Slave_FUNC_03H 0x03 // ���豸
#define Slave_FUNC_04H 0x04 // ������������
#define Slave_FUNC_05H 0x05 // ���ع�����
#define Slave_FUNC_06H 0x06 // д��������
#define Slave_FUNC_07H 0x07 // ������״̬
#define Slave_FUNC_10H 0x10 // д�������
#define Slave_FUNC_65H 0x65 // У׼ģʽ

// == �����Ĵ���״̬ ==
#define Operation_Enable 0xFF00
#define Operation_Disabled 0x0000

// == �Ĵ��� ����  ==
#define Parameter_Register_Number 1024 // �����Ĵ�������
#define Data_Register_Number 10		   // ���ݼĴ�������
#define Calibration_Register_Number 10 // У׼�Ĵ�������
#define Status_Register_Number 50	   // ״̬�Ĵ�������
#define Operation_Register_Number 100  // �����Ĵ�������
#define Error_code_Register_Number 100 // �������Ĵ�������

#define Alert_OK 1	// �����뱻��־
#define Alert_ERR 0 // ������ȡ����־

extern __IO uint8_t Parameter_Register[Parameter_Register_Number];	   // �����Ĵ���
extern __IO uint8_t Data_Register[Data_Register_Number];			   // ���ݼĴ���
extern __IO uint8_t Calibration_Register[Calibration_Register_Number]; // У׼�Ĵ���
extern __IO uint8_t Status_Register[Status_Register_Number];		   // ״̬�Ĵ���
extern __IO uint8_t Operation_Register[Operation_Register_Number];	   // �����Ĵ���
extern __IO uint8_t Error_code_Register[Error_code_Register_Number];   // �������Ĵ���

// == ģ��ӿ� ==

// ������ʼ�� ***
void Modbus_Slave_init(_Modbus_Slave *pModbus_Slave, uint8_t pSlave_ID, UART_HandleTypeDef *pUART_HandleTypeDef_Slave, uint8_t *pRxBuff, uint8_t *pTxBuff, uint16_t pRxBuff_Size, uint16_t pTxBuff_Size, GPIO_TypeDef *pDIR_GPIO_Port, uint16_t pDIR_GPIO_pins, uint8_t send_choose);

// �����жϻص� ***
void Modbus_Slave_Receive_IDLE(_Modbus_Slave *pModbus_Slave);

// Modbus ��վ���ݰ����� ***
void Modbus_Slave_Poll(_Modbus_Slave *pModbus_Slave);

// �Ĵ���ת���� ����ת�Ĵ���
uint16_t Register_Conversion_Read(__IO uint8_t *Register, uint32_t pNomber);

// �Ĵ���ת��д ����ת�Ĵ���
void Register_Conversion_Write(__IO uint8_t *Register, uint32_t pNomber, uint16_t pDATA);

// �������żĴ�����ĳһ��λ
void HalfWord_Write_Bit(__IO uint8_t *pData, uint32_t pNomber, uint16_t pdataNO, uint16_t flag);

// �������Ĵ��� ����ĳһλ����������λ
void HalfWord_Write_Keep_Bit_Set_Remaining(__IO uint8_t *pData, uint32_t pNomber, uint16_t keep_data, uint16_t flag);

#endif
