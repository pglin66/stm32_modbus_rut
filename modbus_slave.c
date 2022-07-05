/**
 * @File Name: modbus_slave.c
 * @Brief : ͨ��ģ�� (��վ��
 * @Author : JYK (yakun81131@yeah.net)
 * @Version : 1.4
 * @Creat Date : 2022-03-5
 *
 */

/********************************************************************************************************************************
 * ģ��ʹ��˵��
 * 1��������������ڴӻ�
 * 2��ÿ����һ�����ڴӻ���Ҫ����ȫ�׵� (��ʼ�� �жϻص� ���շ��ͻ��� ���շ��ͻ�����������������Ϊ2������������Ϊһ���Ĵ�����Ҫ�����ֽڡ���
 * 3����ע���ڳ�ʼ����ʱ�� �粻��Ҫ485��ʹ���շ����� ��ô�뽫 ��־λ����Ϊ 1 ������������������
 * 4��Ϊ������ֲ����������е� �����ṹ�� ����Ӳ����ṹ��
 * 5����ӻ������� ��ע���շ�����Ĵ�С MCU�Ƿ������
 * 6�����´�������ʱ��Ҫ���ķ��ͺ���  ���ͷ�ʽ ����ʹ�����ŵ�
 * 7����modbus_slave.h�ļ��� ע�ͼ� *** �ĺ��������ڶ�Ӧ�ط�����
 * 8������ Slave_Maximum_read_length �����ɸ�������ȡ���� ���65535  ���ܳ������ͻ��棬����û������
 *******************************************************************************************************************************/

#include "modbus_slave.h"

_Modbus_Slave Modbus_Slave_485; // ��վ�����ṹ��

// == �Զ����շ����� ==
uint8_t Modbus_Slave_485_RXbuffer[ModbusS_RXbuffer_size] = {0};
uint8_t Modbus_Slave_485_TXbuffer[ModbusS_TXbuffer_size] = {0};

// == ���Ʋ��� ==
#define Slave_Maximum_read_length 1024 // ����ȡ����

// ===========    ���������β���Ҫ�Ķ�   =================================================================
__IO uint8_t Parameter_Register[Parameter_Register_Number] = {0};	  // �����Ĵ���
__IO uint8_t Data_Register[Data_Register_Number] = {0};				  // ���ݼĴ���
__IO uint8_t Calibration_Register[Calibration_Register_Number] = {0}; // У׼�Ĵ���
__IO uint8_t Status_Register[Status_Register_Number] = {0};			  // ״̬�Ĵ���
__IO uint8_t Operation_Register[Operation_Register_Number] = {0};	  // �����Ĵ���
__IO uint8_t Error_code_Register[Error_code_Register_Number] = {0};	  // �������Ĵ���

static uint8_t Modbus_Read_Register_value(__IO uint8_t *Register, uint16_t Register_number, uint8_t *pData, uint16_t red_addr, uint16_t red_number); // ���Ĵ�������
static uint8_t Modbus_Write_Order_H_register(__IO uint8_t *Register, uint16_t Register_number, uint16_t reg_addr, uint16_t reg_value);				 // д���Ĵ��� - ˫�ֽ�

static void Modbus_Slave_AnalyzeApp(_Modbus_Slave *pModbus_Slave); // ����Ӧ�ò�Э��

static void Modbus_S_01H(_Modbus_Slave *pModbus_Slave); // 01H���������
static void Modbus_S_02H(_Modbus_Slave *pModbus_Slave); // 02H���������
static void Modbus_S_03H(_Modbus_Slave *pModbus_Slave); // 03H���������
static void Modbus_S_04H(_Modbus_Slave *pModbus_Slave); // 04H���������
static void Modbus_S_05H(_Modbus_Slave *pModbus_Slave); // 05H���������
static void Modbus_S_06H(_Modbus_Slave *pModbus_Slave); // 06H���������
static void Modbus_S_07H(_Modbus_Slave *pModbus_Slave); // 07H���������
static void Modbus_S_10H(_Modbus_Slave *pModbus_Slave); // 10H���������
static void Modbus_S_65H(_Modbus_Slave *pModbus_Slave); // 65H���������

static void Modbus_SendAckErr(_Modbus_Slave *pModbus_Slave, uint8_t _ucErrCode);			   // ���ʹ���Ӧ��
static void Modbus_SendAckOk(_Modbus_Slave *pModbus_Slave);									   // ������ȷ��Ӧ��
static void Modbus_SendWithCRC(_Modbus_Slave *pModbus_Slave, uint8_t *_pBuf, uint16_t _ucLen); // ����һ������, �Զ�׷��2�ֽ�CRC

static uint16_t Modbus_Register_Conversion_01HRead(__IO uint8_t *Register, uint32_t pNomber);

static uint8_t Modbus_HalfWord_Read_Bit(uint16_t pdatas, uint8_t len); // ��� 16bit �����е� 1Bit
static uint8_t Modbus_Word_Read_Bit(uint32_t pdatas, uint8_t len);	   // ��� 32bit �����е� 1Bit

// ======================  ���㺯�� ================================================
#define Setbit(x, n) x |= (1 << n)	// ��x�ĵ�nλ��1
#define Clrbit(x, n) x &= ~(1 << n) // ��x�ĵ�nλ��0

static uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint32_t usDataLen, uint8_t flip);
static void InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf);
static void InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf);
static void MODBUS_Array_Zero(uint8_t *pData, uint32_t pNumber);
static uint16_t MODBUS_Merge_array_16(uint8_t dataH, uint8_t dataL);
// ======================  ���㺯�� END ============================================

/**
 * @Brief : Modbus Slave ��ʼ��
 * @param  pModbus_Slave    ����ָ��
 * @param  pSlave_ID        �ӻ� ID
 * @param  pUART_HandleTypeDef_Slave ָ������
 * @param  pRxBuff          ������ջ���
 * @param  pTxBuff          ���巢�ͻ���
 * @param  pRxBuff_Size     ������ջ��泤��
 * @param  pTxBuff_Size     ���巢�ͻ��泤��
 * @param  pDIR_GPIO_Port   ����485ʹ�ܷ���������
 * @param  pDIR_GPIO_pins   ����485ʹ�ܷ������ź�
 * @param  send_choose      �Ƿ��485���� û��485 �� 1 ������ 0  ������1ʱ�����źź������಻�������ã�
 */
void Modbus_Slave_init(_Modbus_Slave *pModbus_Slave, uint8_t pSlave_ID, UART_HandleTypeDef *pUART_HandleTypeDef_Slave, uint8_t *pRxBuff, uint8_t *pTxBuff, uint16_t pRxBuff_Size, uint16_t pTxBuff_Size, GPIO_TypeDef *pDIR_GPIO_Port, uint16_t pDIR_GPIO_pins, uint8_t send_choose)
{
	pModbus_Slave->Slave_ID = pSlave_ID;
	pModbus_Slave->pUART_HandleTypeDef_Slave = pUART_HandleTypeDef_Slave;
	pModbus_Slave->RxBuff = pRxBuff;
	pModbus_Slave->TxBuff = pTxBuff;
	pModbus_Slave->RxBuff_Size = pRxBuff_Size;
	pModbus_Slave->TxBuff_Size = pTxBuff_Size;
	pModbus_Slave->RspCode = 0;
	pModbus_Slave->RxStatus = 0;
	pModbus_Slave->TxCount = 0;
	pModbus_Slave->Send_choose = send_choose;
	pModbus_Slave->DIR_GPIO_Port = pDIR_GPIO_Port;
	pModbus_Slave->DIR_GPIO_pins = pDIR_GPIO_pins;

	__HAL_UART_ENABLE_IT(pModbus_Slave->pUART_HandleTypeDef_Slave, UART_IT_IDLE); // �����ж�

	HAL_UART_Receive_DMA(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->RxBuff, pModbus_Slave->RxBuff_Size); // DMA����

	// HAL_UART_Receive_IT(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->RxBuff, pModbus_Slave->RxBuff_Size); // �жϽ���
}

/**s
 * @Brief : ���տ����жϻص�
 *          ***  ÿ����һ���ӻ���Ҫ�������жϺ�������ӵ�ǰ�ӻ����жϻص� ***
 * @param  pModbus_Slave    ��վ����ָ��
 */
void Modbus_Slave_Receive_IDLE(_Modbus_Slave *pModbus_Slave)
{
	if ((__HAL_UART_GET_FLAG(pModbus_Slave->pUART_HandleTypeDef_Slave, UART_FLAG_IDLE) != RESET)) // �����ж�
	{

		// ���ж�
		__HAL_UART_CLEAR_IDLEFLAG(pModbus_Slave->pUART_HandleTypeDef_Slave); // ��� IDLE�ж�
		HAL_UART_DMAStop(pModbus_Slave->pUART_HandleTypeDef_Slave);			 // ��Ҫֹͣ��ε�DMA���� ������յ����ݲ��ܴ�ͷ���

		// �ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		pModbus_Slave->RxCount = pModbus_Slave->RxBuff_Size - pModbus_Slave->pUART_HandleTypeDef_Slave->RxXferCount;

		// HAL_UART_AbortReceive_IT(pModbus_Slave->pUART_HandleTypeDef_Slave);

		// ������ɱ�־λ��1
		pModbus_Slave->RxStatus = 1;

		// �����жϿ���
		__HAL_UART_ENABLE_IT(pModbus_Slave->pUART_HandleTypeDef_Slave, UART_IT_IDLE);
		// �жϽ��տ���
		// HAL_UART_Receive_IT(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->RxBuff, pModbus_Slave->RxBuff_Size);
		// DMA ���տ���
		HAL_UART_Receive_DMA(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->RxBuff, pModbus_Slave->RxBuff_Size);

		//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);									 // �����ж�
		// HAL_UART_Receive_DMA(&huart1, Modbus_Slave_485_RXbuffer, ModbusS_RXbuffer_size); // DMA����
	}

	if ((__HAL_UART_GET_FLAG(pModbus_Slave->pUART_HandleTypeDef_Slave, UART_FLAG_TC) != RESET)) // ��������ж�  // �жϷ��� DMA����
	{
		// ���ж�
		__HAL_UART_CLEAR_IDLEFLAG(pModbus_Slave->pUART_HandleTypeDef_Slave);
		HAL_GPIO_WritePin(pModbus_Slave->DIR_GPIO_Port, pModbus_Slave->DIR_GPIO_pins, GPIO_PIN_RESET); // 485 ʹ�ܽ���
	}
}

/**
 * @Brief : Modbus ��վ���ݰ�����
 * 			����ѭ����ѭ������  ÿ���ӻ�����һ��
 * @param  pModbus_Slave    ��վ����ָ��
 */
void Modbus_Slave_Poll(_Modbus_Slave *pModbus_Slave)
{
	uint16_t crc1 = 0, crc2 = 0;

	if (pModbus_Slave->RxStatus != 1) // û���յ�����
	{
		return; // �˳�����
	}
	if (pModbus_Slave->RxCount < 5) // ����С��5���ֽ�
	{
		goto exit; // ��ת�����
	}

	// �ϲ� CRC ��λ��ǰ
	crc1 = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[pModbus_Slave->RxCount - 2], pModbus_Slave->RxBuff[pModbus_Slave->RxCount - 1]);
	// ���� CRC ��λ��ǰ
	crc2 = CRC16_MODBUS(pModbus_Slave->RxBuff, (pModbus_Slave->RxCount - 2), 0);
	if (crc1 != crc2) // ���CRC
	{
		goto exit; // ��ת�����
	}

	if (pModbus_Slave->RxBuff[0] == pModbus_Slave->Slave_ID) // �ж�ID
	{
		// == ����Ӧ�ò�Э�� ==
		Modbus_Slave_AnalyzeApp(pModbus_Slave);
	}

exit:
	MODBUS_Array_Zero(pModbus_Slave->RxBuff, pModbus_Slave->RxCount); // ���ջ�������
	pModbus_Slave->RxStatus = 0;									  // ����״̬����
	pModbus_Slave->RxCount = 0;										  // ������ռ�����
}

/**
 * @brief ����һ������, �Զ�׷��2�ֽ�CRC
 *
 * @param _pBuf ����
 * @param _ucLen ���ݳ��ȣ�����CRC��
 */
static void Modbus_SendWithCRC(_Modbus_Slave *pModbus_Slave, uint8_t *_pBuf, uint16_t _ucLen)
{
	uint16_t crc = 0;

	for (uint16_t i = 0; i < _ucLen; i++) // �����ݸ����ͻ���
		pModbus_Slave->TxBuff[i] = _pBuf[i];

	crc = CRC16_MODBUS(pModbus_Slave->TxBuff, _ucLen, 0);
	pModbus_Slave->TxBuff[_ucLen++] = crc >> 8;
	pModbus_Slave->TxBuff[_ucLen++] = crc;

	if (pModbus_Slave->Send_choose == 0)
	{
		HAL_GPIO_WritePin(pModbus_Slave->DIR_GPIO_Port, pModbus_Slave->DIR_GPIO_pins, GPIO_PIN_SET); // 485 ʹ�ܷ���
		// HAL_UART_Transmit(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->TxBuff, _ucLen, 0xFFFF); // ��������
		HAL_UART_Transmit_DMA(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->TxBuff, _ucLen); // DMA ����

		// ��ʱ��DMA����ʱҪ�ȷ��������ʹ�ܽ���
		// HAL_GPIO_WritePin(pModbus_Slave->DIR_GPIO_Port, pModbus_Slave->DIR_GPIO_pins, GPIO_PIN_RESET); // 485 ʹ�ܽ���
	}
	else if (pModbus_Slave->Send_choose == 1)
	{
		// HAL_UART_Transmit(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->TxBuff, _ucLen, 0xFFFFFFFF);
		HAL_UART_Transmit_DMA(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->TxBuff, _ucLen); // DMA ����
	}
}

/**
 * @brief ����Ӧ�ò�Э��
 *        �ڲ�����
 */
static void Modbus_Slave_AnalyzeApp(_Modbus_Slave *pModbus_Slave)
{
	switch (pModbus_Slave->RxBuff[1]) // ��2���ֽ� ������
	{
	case Slave_FUNC_01H:					 // �����Ĵ���
		Modbus_S_01H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_01H = Alert_OK; // ��Ϣ
		break;
	case Slave_FUNC_02H:
		Modbus_S_02H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_02H = Alert_OK; // ��Ϣ
		break;
	case Slave_FUNC_03H:					 // ������
		Modbus_S_03H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_03H = Alert_OK; // ��Ϣ
		break;
	case Slave_FUNC_04H:					 // ������
		Modbus_S_04H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_04H = Alert_OK; // ��Ϣ
		break;
	case Slave_FUNC_05H:					 // д����
		Modbus_S_05H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_05H = Alert_OK; // ��Ϣ
		break;
	case Slave_FUNC_06H:					 // д���Ĵ���
		Modbus_S_06H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_06H = Alert_OK; // ��Ϣ
		break;
	case Slave_FUNC_07H:					 // ������
		Modbus_S_07H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_07H = Alert_OK; // ��Ϣ
		break;
	case Slave_FUNC_10H:					 // д�������
		Modbus_S_10H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_10H = Alert_OK; // ��Ϣ
		break;
	case Slave_FUNC_65H:					 // У׼ģʽ
		Modbus_S_65H(pModbus_Slave);		 // ����
		pModbus_Slave->Alert_65H = Alert_OK; // ��Ϣ
		break;

	default:
		pModbus_Slave->RspCode = Slave_RSP_ERR_CMD;				  // ��֧�ֵĹ�����
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���������������
		break;
	}
}

/**
 * @brief 01H���������
 *        �ڲ�����
 */
static void Modbus_S_01H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[Slave_Maximum_read_length];
	uint16_t tim_data = 0;
	uint16_t data_len = 0;

	pModbus_Slave->RspCode = Slave_RSP_OK; // Ĭ�ϳɹ�

	if (pModbus_Slave->RxCount < 6) // ����С��6�ֽ�
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // �Ĵ�����
	num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // �Ĵ�������

	if (num > pModbus_Slave->TxBuff_Size / 2)
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	// if (Modbus_Read_Register_value(Status_Register, Status_Register_Number, reg_value, reg, num) == 0) // �����Ĵ���ֵ����reg_value
	//{
	// pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // �Ĵ�����ַ����
	// }

	if (reg * 2 <= Status_Register_Number) // ��ַС��������
	{
		for (uint16_t i = 0; i < num; i++)
		{
			tim_data = Modbus_Register_Conversion_01HRead(Status_Register, reg + i); // ������ ָ���Ĵ���������
			reg_value[data_len++] = tim_data >> 8;									 // ���ݲ�ִ��뻺��
			reg_value[data_len++] = tim_data;
		}
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK) // ��ȷӦ��
	{
		pModbus_Slave->TxCount = 0;
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[0]; // ID
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[1]; // ������
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = num * 2;					// �����ֽ���

		for (i = 0; i < num * 2; i++)
			pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = reg_value[i];

		Modbus_SendWithCRC(pModbus_Slave, pModbus_Slave->TxBuff, pModbus_Slave->TxCount); // ������ȷӦ��
	}
	else
	{
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���ʹ���Ӧ��
	}
}

/**
 * @brief 02H���������
 *        �ڲ�����
 */
static void Modbus_S_02H(_Modbus_Slave *pModbus_Slave)
{
}

/**
 * @brief 03H���������
 *        �ڲ�����
 */
static void Modbus_S_03H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[Slave_Maximum_read_length];

	pModbus_Slave->RspCode = Slave_RSP_OK; // Ĭ�ϳɹ�

	if (pModbus_Slave->RxCount < 6) // ����С��6�ֽ�
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // �Ĵ�����
	num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // �Ĵ�������

	if (num > pModbus_Slave->TxBuff_Size / 2)
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	if (Modbus_Read_Register_value(Parameter_Register, Parameter_Register_Number, reg_value, reg, num) == 0) // �����Ĵ���ֵ����reg_value
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // �Ĵ�����ַ����
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK) // ��ȷӦ��
	{
		pModbus_Slave->TxCount = 0;
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[0]; // ID
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[1]; // ������
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = num * 2;					// �����ֽ���

		for (i = 0; i < num * 2; i++)
			pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = reg_value[i];

		Modbus_SendWithCRC(pModbus_Slave, pModbus_Slave->TxBuff, pModbus_Slave->TxCount); // ������ȷӦ��
	}
	else
	{
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���ʹ���Ӧ��
	}
}

/**
 * @brief 04H���������
 *        �ڲ�����
 */
static void Modbus_S_04H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[Slave_Maximum_read_length];

	pModbus_Slave->RspCode = Slave_RSP_OK; // Ĭ�ϳɹ�

	if (pModbus_Slave->RxCount < 6) // ����С��6�ֽ�
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // �Ĵ�����
	num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // �Ĵ�������

	if (num > pModbus_Slave->TxBuff_Size / 2)
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	if (Modbus_Read_Register_value(Data_Register, Data_Register_Number, reg_value, reg, num) == 0) // �����Ĵ���ֵ����reg_value
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // �Ĵ�����ַ����
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK) // ��ȷӦ��
	{
		pModbus_Slave->TxCount = 0;
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[0]; // ID
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[1]; // ������
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = num * 2;					// �����ֽ���

		for (i = 0; i < num * 2; i++)
			pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = reg_value[i];

		Modbus_SendWithCRC(pModbus_Slave, pModbus_Slave->TxBuff, pModbus_Slave->TxCount); // ������ȷӦ��
	}
	else
	{
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���ʹ���Ӧ��
	}
}

/**
 * @brief 05H���������
 *        �ڲ�����
 */
static void Modbus_S_05H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t value;

	pModbus_Slave->RspCode = Slave_RSP_OK; // Ĭ�ϳɹ�

	if (pModbus_Slave->RxCount < 6) // ����С��6�ֽ�
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]);   // �Ĵ�����
	value = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // �Ĵ���ֵ

	if (Modbus_Write_Order_H_register(Operation_Register, Operation_Register_Number, reg, value) != 1) // ��д���ֵ����Ĵ���
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR;											   // �Ĵ�����ַ����

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK)
		Modbus_SendAckOk(pModbus_Slave); // ��ȷӦ��
	else
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���������������
}

/**
 * @brief 06H���������
 *        �ڲ�����
 */
static void Modbus_S_06H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t value;

	pModbus_Slave->RspCode = Slave_RSP_OK; // Ĭ�ϳɹ�

	if (pModbus_Slave->RxCount < 6) // ����С��6�ֽ�
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]);   // �Ĵ�����
	value = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // �Ĵ���ֵ

	if (Modbus_Write_Order_H_register(Parameter_Register, Parameter_Register_Number, reg, value) != 1) // ��д���ֵ����Ĵ���
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR;											   // �Ĵ�����ַ����

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK)
		Modbus_SendAckOk(pModbus_Slave); // ��ȷӦ��
	else
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���������������
}

/**
 * @brief 07H���������
 *        �ڲ�����
 */
static void Modbus_S_07H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[Slave_Maximum_read_length];

	pModbus_Slave->RspCode = Slave_RSP_OK; // Ĭ�ϳɹ�

	if (pModbus_Slave->RxCount < 6) // ����С��6�ֽ�
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // �Ĵ�����
	num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // �Ĵ�������

	if (num > pModbus_Slave->TxBuff_Size / 2)
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	if (Modbus_Read_Register_value(Error_code_Register, Error_code_Register_Number, reg_value, reg, num) == 0) // �����Ĵ���ֵ����reg_value
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // �Ĵ�����ַ����
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK) // ��ȷӦ��
	{
		pModbus_Slave->TxCount = 0;
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[0]; // ID
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[1]; // ������
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = num * 2;					// �����ֽ���

		for (i = 0; i < num * 2; i++)
			pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = reg_value[i];

		Modbus_SendWithCRC(pModbus_Slave, pModbus_Slave->TxBuff, pModbus_Slave->TxCount); // ������ȷӦ��
	}
	else
	{
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���ʹ���Ӧ��
	}
}

/**
 * @brief 10H���������
 *        �ڲ�����
 */
static void Modbus_S_10H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	// uint16_t reg_num;
	uint16_t byte_num;

	pModbus_Slave->RspCode = Slave_RSP_OK; // Ĭ�ϳɹ�

	if (pModbus_Slave->RxCount < 6) // ����С��6�ֽ�
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // �Ĵ�����
	// reg_num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // �Ĵ�������
	byte_num = pModbus_Slave->RxBuff[6];

	if (byte_num * 2 <= Parameter_Register_Number && reg <= Parameter_Register_Number)
	{
		for (uint16_t i = 0; i < byte_num; i++)
			Parameter_Register[(reg * 2) + i] = pModbus_Slave->RxBuff[7 + i]; // д��
	}
	else
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // �Ĵ�����ַ����
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK)
		Modbus_SendAckOk(pModbus_Slave); // ��ȷӦ��
	else
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���������������
}

/**
 * @brief 65H���������
 *        �ڲ�����
 */
static void Modbus_S_65H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t value;

	pModbus_Slave->RspCode = Slave_RSP_OK; // Ĭ�ϳɹ�

	if (pModbus_Slave->RxCount < 6) // ����С��6�ֽ�
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // ����ֵ�����
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]);   // �Ĵ�����
	value = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // �Ĵ���ֵ

	if (Modbus_Write_Order_H_register(Calibration_Register, Calibration_Register_Number, reg, value) != 1) // ��д���ֵ����Ĵ���
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR;												   // �Ĵ�����ַ����

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK)
		Modbus_SendAckOk(pModbus_Slave); // ��ȷӦ��
	else
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // ���������������
}

/**
 * @brief ���Ĵ�������
 *
 * @param Register �Ĵ���
 * @param Register_number �Ĵ�������
 * @param pData �洢����
 * @param red_addr ����ַ
 * @param red_number ���Ĵ�������
 * @return uint8_t
 */
static uint8_t Modbus_Read_Register_value(__IO uint8_t *Register, uint16_t Register_number, uint8_t *pData, uint16_t red_addr, uint16_t red_number)
{
	uint16_t tim_data = 0;
	uint16_t data_len = 0;
	if (red_addr * 2 <= Register_number) // ��ַС��������
	{
		for (uint16_t i = 0; i < red_number; i++)
		{
			tim_data = Register_Conversion_Read(Register, red_addr + i); // ������ ָ���Ĵ���������
			pData[data_len++] = tim_data >> 8;							 // ���ݲ�ִ��뻺��
			pData[data_len++] = tim_data;
		}
	}
	else
		return 0; // �����쳣������ 0

	return 1; // д�ɹ�
}

/**
 * @brief д���Ĵ��� - ˫�ֽ�
 *
 * @param Register �Ĵ���
 * @param Register_number �Ĵ�������
 * @param reg_addr ��ַ
 * @param reg_value ����
 * @return uint8_t
 */
static uint8_t Modbus_Write_Order_H_register(__IO uint8_t *Register, uint16_t Register_number, uint16_t reg_addr, uint16_t reg_value)
{
	if (reg_addr * 2 <= Register_number) // ��ַС��������
	{
		Register_Conversion_Write(Register, reg_addr, reg_value);
	}
	else
		return 0; // �����쳣������ 0

	return 1; // д�ɹ�
}

/**
 * @brief �������Ĵ��������е�1bit
 *
 * @param pData ���Ĵ���
 * @param pNomber ���Ĵ�����
 * @param pdataNO Ҫ������λ
 * @param flag λ��״̬
 */
void HalfWord_Write_Bit(__IO uint8_t *pData, uint32_t pNomber, uint16_t pdataNO, uint16_t flag)
{
	uint16_t datas;
	if (pdataNO < 16 || (flag != 0 && flag != 1)) // ������λ��Ų��ܴ������� �� λ��״ֻ̬�� 0 �� 1
	{
		datas = Register_Conversion_Read(pData, pNomber); // �Ƚ����ݶ���

		if (flag == 0)
			Clrbit(datas, pdataNO); // ��datas�ĵ�pdatasλ��0
		else if (flag == 1)
			Setbit(datas, pdataNO); // ��datas�ĵ�pdatasλ��1

		Register_Conversion_Write(pData, pNomber, datas); // ����������д��
	}
}

/**
 * @brief : �������Ĵ��� ����ĳһλ����������λ
 * @param  pData        ���Ĵ���
 * @param  pNomber      ���Ĵ�����
 * @param  keep_data    Ҫ������λ
 * @param  flag         ����λ��״̬
 */
void HalfWord_Write_Keep_Bit_Set_Remaining(__IO uint8_t *pData, uint32_t pNomber, uint16_t keep_data, uint16_t flag)
{
	uint16_t datas;
	uint8_t _flag;

	if (keep_data < 16 || (flag != 0 && flag != 1)) // ������λ��Ų��ܴ������� �� λ��״ֻ̬�� 0 �� 1
	{
		datas = Register_Conversion_Read(pData, pNomber);	// �Ƚ����ݶ���
		_flag = Modbus_HalfWord_Read_Bit(datas, keep_data); // �Ƚ�����λȡ��

		if (flag == 0) // ����λ��״̬Ϊ 0
		{
			datas = 0x0000;

			if (_flag == 0)
				Clrbit(datas, _flag); // ��Ҫ������λ��״̬д��
			else if (_flag == 1)
				Setbit(datas, _flag); // ��Ҫ������λ��״̬д��
		}
		else // ����λ��״̬Ϊ 1
		{
			datas = 0xFFFF;

			if (_flag == 0)
				Clrbit(datas, _flag); // ��Ҫ������λ��״̬д��
			else if (_flag == 1)
				Setbit(datas, _flag); // ��Ҫ������λ��״̬д��
		}

		Register_Conversion_Write(pData, pNomber, datas); // ����������д��
	}
}

/**
 * @brief ��� 16bit �����е� 1Bit
 *
 * @param pdatas ����
 * @param len Ҫ�����λ
 * @return λ״̬
 */
static uint8_t Modbus_HalfWord_Read_Bit(uint16_t pdatas, uint8_t len)
{
	uint8_t pData;

	if (len == 0)
		pData = pdatas & 0x01; // ���λbai
	else if (len == 1)
		pData = (pdatas & 0x02) >> 1;
	else if (len == 2)
		pData = (pdatas & 0x04) >> 2;
	else if (len == 3)
		pData = (pdatas & 0x08) >> 3;
	else if (len == 4)
		pData = (pdatas & 0x10) >> 4;
	else if (len == 5)
		pData = (pdatas & 0x20) >> 5;
	else if (len == 6)
		pData = (pdatas & 0x40) >> 6;
	else if (len == 7)
		pData = (pdatas & 0x80) >> 7;
	else if (len == 8)
		pData = (pdatas & 0x100) >> 8;
	else if (len == 9)
		pData = (pdatas & 0x200) >> 9;
	else if (len == 10)
		pData = (pdatas & 0x400) >> 10;
	else if (len == 11)
		pData = (pdatas & 0x800) >> 11;
	else if (len == 12)
		pData = (pdatas & 0x1000) >> 12;
	else if (len == 13)
		pData = (pdatas & 0x2000) >> 13;
	else if (len == 14)
		pData = (pdatas & 0x4000) >> 14;
	else if (len == 15)
		pData = (pdatas & 0x8000) >> 15;

	return pData;
}

/**
 * @brief : ��� 32bit �����е� 1Bit
 * @param  pdatas    ����
 * @param  len       Ҫ�����λ
 * @return λ״̬
 */
static uint8_t Modbus_Word_Read_Bit(uint32_t pdatas, uint8_t len)
{
	uint8_t pData;
	pData = (pdatas & (0x00000001 << len)) >> len;

	return pData;
}

/**
 * @brief ���ʹ���Ӧ��
 *
 * @param _ucErrCode �������
 */
static void Modbus_SendAckErr(_Modbus_Slave *pModbus_Slave, uint8_t _ucErrCode)
{
	uint8_t txbuf[3];

	txbuf[0] = pModbus_Slave->RxBuff[0];		// ��ַ
	txbuf[1] = pModbus_Slave->RxBuff[1] | 0x80; // �쳣�Ĺ�����
	txbuf[2] = _ucErrCode;						// �������(01,02,03,04)

	Modbus_SendWithCRC(pModbus_Slave, txbuf, 3);
}

/**
 * @brief ������ȷ��Ӧ��
 *
 */
static void Modbus_SendAckOk(_Modbus_Slave *pModbus_Slave)
{
	uint8_t txbuf[6];

	for (uint8_t i = 0; i < 6; i++)
		txbuf[i] = pModbus_Slave->RxBuff[i];
	Modbus_SendWithCRC(pModbus_Slave, txbuf, 6);
}

/**
 * @brief �Ĵ���ת���� ����ת�Ĵ��� modbus 01H ����ʹ��
 *
 * @param Register �Ĵ���
 * @param pNomber ���Ĵ�����
 * @return uint16_t  �Ĵ���ֵ
 */
static uint16_t Modbus_Register_Conversion_01HRead(__IO uint8_t *Register, uint32_t pNomber)
{
	uint16_t pData;
	pData = ((Register[(pNomber * 2) + 1] << 8) | Register[(pNomber * 2)]); // �ϲ��Ĵ���ֵ
	return pData;
}

/**
 * @brief �Ĵ���ת���� ����ת�Ĵ���
 *
 * @param Register �Ĵ���
 * @param pNomber ���Ĵ�����
 * @return uint16_t  �Ĵ���ֵ
 */
uint16_t Register_Conversion_Read(__IO uint8_t *Register, uint32_t pNomber)
{
	uint16_t pData;
	pData = ((Register[(pNomber * 2)] << 8) | Register[(pNomber * 2) + 1]); // �ϲ��Ĵ���ֵ
	return pData;
}

/**
 * @brief �Ĵ���ת��д ����ת�Ĵ���
 *
 * @param Register �Ĵ���
 * @param pNomber ���Ĵ�����
 * @param pDATA д��Ĵ�����ֵ
 */
void Register_Conversion_Write(__IO uint8_t *Register, uint32_t pNomber, uint16_t pDATA)
{
	Register[(pNomber * 2)] = pDATA >> 8;
	Register[(pNomber * 2) + 1] = pDATA;
}

/**
**************************************************************************************************
* @Brief           ���ֽ����ݷ�ת
* @Param
*            @DesBuf: destination buffer
*            @SrcBuf: source buffer
* @RetVal    None
* @Note      (MSB)0101_0101 ---> 1010_1010(LSB)
**************************************************************************************************
*/
static void InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf)
{
	int i;
	unsigned char temp = 0;
	for (i = 0; i < 8; i++)
	{
		if (SrcBuf[0] & (1 << i))
		{
			temp |= 1 << (7 - i);
		}
	}
	DesBuf[0] = temp;
}

/**
 **************************************************************************************************
 * @Brief       ˫�ֽ����ݷ�ת
 * @Param
 *            @DesBuf: destination buffer
 *            @SrcBuf: source buffer
 * @RetVal    None
 * @Note      (MSB)0101_0101_1010_1010 ---> 0101_0101_1010_1010(LSB)
 **************************************************************************************************
 */
static void InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf)
{
	int i;
	unsigned short temp = 0;
	for (i = 0; i < 16; i++)
	{
		if (SrcBuf[0] & (1 << i))
		{
			temp |= 1 << (15 - i);
		}
	}
	DesBuf[0] = temp;
}

/**
 * CRCУ��
 * ���ƣ� CRC-16/MODBUS
 * ����ʽ�� x16 + x15 + x12 + 1
 * puchMsg ��Ҫ����CRC������
 * usDataLen ������������ֵ�ĸ���
 * flip ����ߵ�λ�Ƿ�ת
 * ��� flip = 0 ����ת ��λ��ǰ  flip = 1 ��ת ��λ��ǰ
 */
static uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint32_t usDataLen, uint8_t flip)
{
	uint16_t wCRCin = 0xFFFF;
	uint16_t wCPoly = 0x8005;
	uint8_t wChar = 0;
	uint8_t t_data[2] = {0};

	while (usDataLen--)
	{
		wChar = *(puchMsg++);
		InvertUint8(&wChar, &wChar);
		wCRCin ^= (wChar << 8);
		for (int i = 0; i < 8; i++)
		{
			if (wCRCin & 0x8000)
			{
				wCRCin = (wCRCin << 1) ^ wCPoly;
			}
			else
			{
				wCRCin = wCRCin << 1;
			}
		}
	}

	InvertUint16(&wCRCin, &wCRCin);

	if (flip == 0)
	{							   // ��ת ��λ��ǰ
		t_data[0] = wCRCin >> 8;   // ��8λ
		t_data[1] = wCRCin & 0xFF; // ��8λ

		wCRCin = ((t_data[1] << 8) | t_data[0]);
	}

	return (wCRCin);
}

/*
 * ��������
 * *pData ����
 * pNumber ����  ��� 65536 ��
 */
static void MODBUS_Array_Zero(uint8_t *pData, uint32_t pNumber)
{
	for (uint32_t i = 0; i < pNumber; i++)
		pData[i] = 0;
}

/*
 * �ϲ�����8λ Ϊ16λ
 * dataH ��λ
 * dataL ��λ
 */
static uint16_t MODBUS_Merge_array_16(uint8_t dataH, uint8_t dataL)
{
	uint16_t data = 0;
	data = ((dataH << 8) | dataL);
	return data;
}
