/**
 * @File Name: modbus_slave.c
 * @Brief : 通信模块 (从站）
 * @Author : JYK (yakun81131@yeah.net)
 * @Version : 1.4
 * @Creat Date : 2022-03-5
 *
 */

/********************************************************************************************************************************
 * 模块使用说明
 * 1、可声明多个串口从机
 * 2、每声明一个串口从机都要设置全套的 (初始化 中断回调 接收发送缓存 接收发送缓存容量【容量必须为2的整倍数，因为一个寄存器需要两个字节】）
 * 3、需注意在初始化的时候 如不需要485的使能收发引脚 那么请将 标志位设置为 1 以屏蔽引脚驱动函数
 * 4、为方便移植，请参照已有的 参数结构体 来添加参数结构体
 * 5、多从机的设置 需注意收发缓存的大小 MCU是否可容纳
 * 6、在新创建串口时需要更改发送函数  发送方式 发送使能引脚等
 * 7、在modbus_slave.h文件中 注释加 *** 的函数必须在对应地方调用
 * 8、更改 Slave_Maximum_read_length 参数可更改最大读取长度 最大65535  不能超过发送缓存，否则没有意义
 *******************************************************************************************************************************/

#include "modbus_slave.h"

_Modbus_Slave Modbus_Slave_485; // 主站参数结构体

// == 自定义收发缓存 ==
uint8_t Modbus_Slave_485_RXbuffer[ModbusS_RXbuffer_size] = {0};
uint8_t Modbus_Slave_485_TXbuffer[ModbusS_TXbuffer_size] = {0};

// == 限制参数 ==
#define Slave_Maximum_read_length 1024 // 最大读取长度

// ===========    以下已屏蔽不需要改动   =================================================================
__IO uint8_t Parameter_Register[Parameter_Register_Number] = {0};	  // 参数寄存器
__IO uint8_t Data_Register[Data_Register_Number] = {0};				  // 数据寄存器
__IO uint8_t Calibration_Register[Calibration_Register_Number] = {0}; // 校准寄存器
__IO uint8_t Status_Register[Status_Register_Number] = {0};			  // 状态寄存器
__IO uint8_t Operation_Register[Operation_Register_Number] = {0};	  // 操作寄存器
__IO uint8_t Error_code_Register[Error_code_Register_Number] = {0};	  // 错误代码寄存器

static uint8_t Modbus_Read_Register_value(__IO uint8_t *Register, uint16_t Register_number, uint8_t *pData, uint16_t red_addr, uint16_t red_number); // 读寄存器数据
static uint8_t Modbus_Write_Order_H_register(__IO uint8_t *Register, uint16_t Register_number, uint16_t reg_addr, uint16_t reg_value);				 // 写单寄存器 - 双字节

static void Modbus_Slave_AnalyzeApp(_Modbus_Slave *pModbus_Slave); // 分析应用层协议

static void Modbus_S_01H(_Modbus_Slave *pModbus_Slave); // 01H功能码解析
static void Modbus_S_02H(_Modbus_Slave *pModbus_Slave); // 02H功能码解析
static void Modbus_S_03H(_Modbus_Slave *pModbus_Slave); // 03H功能码解析
static void Modbus_S_04H(_Modbus_Slave *pModbus_Slave); // 04H功能码解析
static void Modbus_S_05H(_Modbus_Slave *pModbus_Slave); // 05H功能码解析
static void Modbus_S_06H(_Modbus_Slave *pModbus_Slave); // 06H功能码解析
static void Modbus_S_07H(_Modbus_Slave *pModbus_Slave); // 07H功能码解析
static void Modbus_S_10H(_Modbus_Slave *pModbus_Slave); // 10H功能码解析
static void Modbus_S_65H(_Modbus_Slave *pModbus_Slave); // 65H功能码解析

static void Modbus_SendAckErr(_Modbus_Slave *pModbus_Slave, uint8_t _ucErrCode);			   // 发送错误应答
static void Modbus_SendAckOk(_Modbus_Slave *pModbus_Slave);									   // 发送正确的应答
static void Modbus_SendWithCRC(_Modbus_Slave *pModbus_Slave, uint8_t *_pBuf, uint16_t _ucLen); // 发送一串数据, 自动追加2字节CRC

static uint16_t Modbus_Register_Conversion_01HRead(__IO uint8_t *Register, uint32_t pNomber);

static uint8_t Modbus_HalfWord_Read_Bit(uint16_t pdatas, uint8_t len); // 输出 16bit 数据中的 1Bit
static uint8_t Modbus_Word_Read_Bit(uint32_t pdatas, uint8_t len);	   // 输出 32bit 数据中的 1Bit

// ======================  计算函数 ================================================
#define Setbit(x, n) x |= (1 << n)	// 将x的第n位置1
#define Clrbit(x, n) x &= ~(1 << n) // 将x的第n位清0

static uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint32_t usDataLen, uint8_t flip);
static void InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf);
static void InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf);
static void MODBUS_Array_Zero(uint8_t *pData, uint32_t pNumber);
static uint16_t MODBUS_Merge_array_16(uint8_t dataH, uint8_t dataL);
// ======================  计算函数 END ============================================

/**
 * @Brief : Modbus Slave 初始化
 * @param  pModbus_Slave    参数指针
 * @param  pSlave_ID        从机 ID
 * @param  pUART_HandleTypeDef_Slave 指定串口
 * @param  pRxBuff          定义接收缓存
 * @param  pTxBuff          定义发送缓存
 * @param  pRxBuff_Size     定义接收缓存长度
 * @param  pTxBuff_Size     定义发送缓存长度
 * @param  pDIR_GPIO_Port   定义485使能发送引脚类
 * @param  pDIR_GPIO_pins   定义485使能发送引脚号
 * @param  send_choose      是否带485发送 没有485 置 1 否则置 0  （当置1时，引脚号和引脚类不再起作用）
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

	__HAL_UART_ENABLE_IT(pModbus_Slave->pUART_HandleTypeDef_Slave, UART_IT_IDLE); // 空闲中断

	HAL_UART_Receive_DMA(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->RxBuff, pModbus_Slave->RxBuff_Size); // DMA接收

	// HAL_UART_Receive_IT(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->RxBuff, pModbus_Slave->RxBuff_Size); // 中断接收
}

/**s
 * @Brief : 接收空闲中断回调
 *          ***  每声明一个从机都要重新在中断函数里添加当前从机的中断回调 ***
 * @param  pModbus_Slave    从站参数指针
 */
void Modbus_Slave_Receive_IDLE(_Modbus_Slave *pModbus_Slave)
{
	if ((__HAL_UART_GET_FLAG(pModbus_Slave->pUART_HandleTypeDef_Slave, UART_FLAG_IDLE) != RESET)) // 空闲中断
	{

		// 清中断
		__HAL_UART_CLEAR_IDLEFLAG(pModbus_Slave->pUART_HandleTypeDef_Slave); // 清除 IDLE中断
		HAL_UART_DMAStop(pModbus_Slave->pUART_HandleTypeDef_Slave);			 // 需要停止这次的DMA传输 否则接收的数据不能从头填充

		// 总计数减去未传输的数据个数，得到已经接收的数据个数
		pModbus_Slave->RxCount = pModbus_Slave->RxBuff_Size - pModbus_Slave->pUART_HandleTypeDef_Slave->RxXferCount;

		// HAL_UART_AbortReceive_IT(pModbus_Slave->pUART_HandleTypeDef_Slave);

		// 接受完成标志位置1
		pModbus_Slave->RxStatus = 1;

		// 空闲中断开启
		__HAL_UART_ENABLE_IT(pModbus_Slave->pUART_HandleTypeDef_Slave, UART_IT_IDLE);
		// 中断接收开启
		// HAL_UART_Receive_IT(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->RxBuff, pModbus_Slave->RxBuff_Size);
		// DMA 接收开启
		HAL_UART_Receive_DMA(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->RxBuff, pModbus_Slave->RxBuff_Size);

		//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);									 // 空闲中断
		// HAL_UART_Receive_DMA(&huart1, Modbus_Slave_485_RXbuffer, ModbusS_RXbuffer_size); // DMA接收
	}

	if ((__HAL_UART_GET_FLAG(pModbus_Slave->pUART_HandleTypeDef_Slave, UART_FLAG_TC) != RESET)) // 发送完成中断  // 中断发送 DMA发送
	{
		// 清中断
		__HAL_UART_CLEAR_IDLEFLAG(pModbus_Slave->pUART_HandleTypeDef_Slave);
		HAL_GPIO_WritePin(pModbus_Slave->DIR_GPIO_Port, pModbus_Slave->DIR_GPIO_pins, GPIO_PIN_RESET); // 485 使能接收
	}
}

/**
 * @Brief : Modbus 从站数据包解析
 * 			在主循环中循环调用  每个从机调用一次
 * @param  pModbus_Slave    从站参数指针
 */
void Modbus_Slave_Poll(_Modbus_Slave *pModbus_Slave)
{
	uint16_t crc1 = 0, crc2 = 0;

	if (pModbus_Slave->RxStatus != 1) // 没有收到数据
	{
		return; // 退出函数
	}
	if (pModbus_Slave->RxCount < 5) // 接收小于5个字节
	{
		goto exit; // 跳转到最后
	}

	// 合并 CRC 低位在前
	crc1 = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[pModbus_Slave->RxCount - 2], pModbus_Slave->RxBuff[pModbus_Slave->RxCount - 1]);
	// 计算 CRC 低位在前
	crc2 = CRC16_MODBUS(pModbus_Slave->RxBuff, (pModbus_Slave->RxCount - 2), 0);
	if (crc1 != crc2) // 检查CRC
	{
		goto exit; // 跳转到最后
	}

	if (pModbus_Slave->RxBuff[0] == pModbus_Slave->Slave_ID) // 判断ID
	{
		// == 分析应用层协议 ==
		Modbus_Slave_AnalyzeApp(pModbus_Slave);
	}

exit:
	MODBUS_Array_Zero(pModbus_Slave->RxBuff, pModbus_Slave->RxCount); // 接收缓存清零
	pModbus_Slave->RxStatus = 0;									  // 接收状态清零
	pModbus_Slave->RxCount = 0;										  // 清零接收计数器
}

/**
 * @brief 发送一串数据, 自动追加2字节CRC
 *
 * @param _pBuf 数据
 * @param _ucLen 数据长度（不带CRC）
 */
static void Modbus_SendWithCRC(_Modbus_Slave *pModbus_Slave, uint8_t *_pBuf, uint16_t _ucLen)
{
	uint16_t crc = 0;

	for (uint16_t i = 0; i < _ucLen; i++) // 将数据给发送缓存
		pModbus_Slave->TxBuff[i] = _pBuf[i];

	crc = CRC16_MODBUS(pModbus_Slave->TxBuff, _ucLen, 0);
	pModbus_Slave->TxBuff[_ucLen++] = crc >> 8;
	pModbus_Slave->TxBuff[_ucLen++] = crc;

	if (pModbus_Slave->Send_choose == 0)
	{
		HAL_GPIO_WritePin(pModbus_Slave->DIR_GPIO_Port, pModbus_Slave->DIR_GPIO_pins, GPIO_PIN_SET); // 485 使能发送
		// HAL_UART_Transmit(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->TxBuff, _ucLen, 0xFFFF); // 阻塞发送
		HAL_UART_Transmit_DMA(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->TxBuff, _ucLen); // DMA 发送

		// 当时用DMA发送时要等发送完成再使能接收
		// HAL_GPIO_WritePin(pModbus_Slave->DIR_GPIO_Port, pModbus_Slave->DIR_GPIO_pins, GPIO_PIN_RESET); // 485 使能接收
	}
	else if (pModbus_Slave->Send_choose == 1)
	{
		// HAL_UART_Transmit(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->TxBuff, _ucLen, 0xFFFFFFFF);
		HAL_UART_Transmit_DMA(pModbus_Slave->pUART_HandleTypeDef_Slave, pModbus_Slave->TxBuff, _ucLen); // DMA 发送
	}
}

/**
 * @brief 分析应用层协议
 *        内部函数
 */
static void Modbus_Slave_AnalyzeApp(_Modbus_Slave *pModbus_Slave)
{
	switch (pModbus_Slave->RxBuff[1]) // 第2个字节 功能码
	{
	case Slave_FUNC_01H:					 // 操作寄存器
		Modbus_S_01H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_01H = Alert_OK; // 消息
		break;
	case Slave_FUNC_02H:
		Modbus_S_02H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_02H = Alert_OK; // 消息
		break;
	case Slave_FUNC_03H:					 // 读保持
		Modbus_S_03H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_03H = Alert_OK; // 消息
		break;
	case Slave_FUNC_04H:					 // 读数据
		Modbus_S_04H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_04H = Alert_OK; // 消息
		break;
	case Slave_FUNC_05H:					 // 写操作
		Modbus_S_05H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_05H = Alert_OK; // 消息
		break;
	case Slave_FUNC_06H:					 // 写单寄存器
		Modbus_S_06H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_06H = Alert_OK; // 消息
		break;
	case Slave_FUNC_07H:					 // 读错误
		Modbus_S_07H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_07H = Alert_OK; // 消息
		break;
	case Slave_FUNC_10H:					 // 写多个保持
		Modbus_S_10H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_10H = Alert_OK; // 消息
		break;
	case Slave_FUNC_65H:					 // 校准模式
		Modbus_S_65H(pModbus_Slave);		 // 解析
		pModbus_Slave->Alert_65H = Alert_OK; // 消息
		break;

	default:
		pModbus_Slave->RspCode = Slave_RSP_ERR_CMD;				  // 不支持的功能码
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 告诉主机命令错误
		break;
	}
}

/**
 * @brief 01H功能码解析
 *        内部函数
 */
static void Modbus_S_01H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[Slave_Maximum_read_length];
	uint16_t tim_data = 0;
	uint16_t data_len = 0;

	pModbus_Slave->RspCode = Slave_RSP_OK; // 默认成功

	if (pModbus_Slave->RxCount < 6) // 接收小于6字节
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // 寄存器号
	num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // 寄存器个数

	if (num > pModbus_Slave->TxBuff_Size / 2)
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	// if (Modbus_Read_Register_value(Status_Register, Status_Register_Number, reg_value, reg, num) == 0) // 读出寄存器值放入reg_value
	//{
	// pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // 寄存器地址错误
	// }

	if (reg * 2 <= Status_Register_Number) // 地址小于总容量
	{
		for (uint16_t i = 0; i < num; i++)
		{
			tim_data = Modbus_Register_Conversion_01HRead(Status_Register, reg + i); // 索引回 指定寄存器的数据
			reg_value[data_len++] = tim_data >> 8;									 // 数据拆分存入缓存
			reg_value[data_len++] = tim_data;
		}
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK) // 正确应答
	{
		pModbus_Slave->TxCount = 0;
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[0]; // ID
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[1]; // 功能码
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = num * 2;					// 返回字节数

		for (i = 0; i < num * 2; i++)
			pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = reg_value[i];

		Modbus_SendWithCRC(pModbus_Slave, pModbus_Slave->TxBuff, pModbus_Slave->TxCount); // 发送正确应答
	}
	else
	{
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 发送错误应答
	}
}

/**
 * @brief 02H功能码解析
 *        内部函数
 */
static void Modbus_S_02H(_Modbus_Slave *pModbus_Slave)
{
}

/**
 * @brief 03H功能码解析
 *        内部函数
 */
static void Modbus_S_03H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[Slave_Maximum_read_length];

	pModbus_Slave->RspCode = Slave_RSP_OK; // 默认成功

	if (pModbus_Slave->RxCount < 6) // 接收小于6字节
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // 寄存器号
	num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // 寄存器个数

	if (num > pModbus_Slave->TxBuff_Size / 2)
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	if (Modbus_Read_Register_value(Parameter_Register, Parameter_Register_Number, reg_value, reg, num) == 0) // 读出寄存器值放入reg_value
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // 寄存器地址错误
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK) // 正确应答
	{
		pModbus_Slave->TxCount = 0;
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[0]; // ID
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[1]; // 功能码
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = num * 2;					// 返回字节数

		for (i = 0; i < num * 2; i++)
			pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = reg_value[i];

		Modbus_SendWithCRC(pModbus_Slave, pModbus_Slave->TxBuff, pModbus_Slave->TxCount); // 发送正确应答
	}
	else
	{
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 发送错误应答
	}
}

/**
 * @brief 04H功能码解析
 *        内部函数
 */
static void Modbus_S_04H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[Slave_Maximum_read_length];

	pModbus_Slave->RspCode = Slave_RSP_OK; // 默认成功

	if (pModbus_Slave->RxCount < 6) // 接收小于6字节
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // 寄存器号
	num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // 寄存器个数

	if (num > pModbus_Slave->TxBuff_Size / 2)
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	if (Modbus_Read_Register_value(Data_Register, Data_Register_Number, reg_value, reg, num) == 0) // 读出寄存器值放入reg_value
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // 寄存器地址错误
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK) // 正确应答
	{
		pModbus_Slave->TxCount = 0;
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[0]; // ID
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[1]; // 功能码
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = num * 2;					// 返回字节数

		for (i = 0; i < num * 2; i++)
			pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = reg_value[i];

		Modbus_SendWithCRC(pModbus_Slave, pModbus_Slave->TxBuff, pModbus_Slave->TxCount); // 发送正确应答
	}
	else
	{
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 发送错误应答
	}
}

/**
 * @brief 05H功能码解析
 *        内部函数
 */
static void Modbus_S_05H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t value;

	pModbus_Slave->RspCode = Slave_RSP_OK; // 默认成功

	if (pModbus_Slave->RxCount < 6) // 接收小于6字节
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]);   // 寄存器号
	value = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // 寄存器值

	if (Modbus_Write_Order_H_register(Operation_Register, Operation_Register_Number, reg, value) != 1) // 把写入的值存入寄存器
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR;											   // 寄存器地址错误

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK)
		Modbus_SendAckOk(pModbus_Slave); // 正确应答
	else
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 告诉主机命令错误
}

/**
 * @brief 06H功能码解析
 *        内部函数
 */
static void Modbus_S_06H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t value;

	pModbus_Slave->RspCode = Slave_RSP_OK; // 默认成功

	if (pModbus_Slave->RxCount < 6) // 接收小于6字节
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]);   // 寄存器号
	value = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // 寄存器值

	if (Modbus_Write_Order_H_register(Parameter_Register, Parameter_Register_Number, reg, value) != 1) // 把写入的值存入寄存器
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR;											   // 寄存器地址错误

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK)
		Modbus_SendAckOk(pModbus_Slave); // 正确应答
	else
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 告诉主机命令错误
}

/**
 * @brief 07H功能码解析
 *        内部函数
 */
static void Modbus_S_07H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[Slave_Maximum_read_length];

	pModbus_Slave->RspCode = Slave_RSP_OK; // 默认成功

	if (pModbus_Slave->RxCount < 6) // 接收小于6字节
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // 寄存器号
	num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // 寄存器个数

	if (num > pModbus_Slave->TxBuff_Size / 2)
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	if (Modbus_Read_Register_value(Error_code_Register, Error_code_Register_Number, reg_value, reg, num) == 0) // 读出寄存器值放入reg_value
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // 寄存器地址错误
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK) // 正确应答
	{
		pModbus_Slave->TxCount = 0;
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[0]; // ID
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = pModbus_Slave->RxBuff[1]; // 功能码
		pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = num * 2;					// 返回字节数

		for (i = 0; i < num * 2; i++)
			pModbus_Slave->TxBuff[pModbus_Slave->TxCount++] = reg_value[i];

		Modbus_SendWithCRC(pModbus_Slave, pModbus_Slave->TxBuff, pModbus_Slave->TxCount); // 发送正确应答
	}
	else
	{
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 发送错误应答
	}
}

/**
 * @brief 10H功能码解析
 *        内部函数
 */
static void Modbus_S_10H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	// uint16_t reg_num;
	uint16_t byte_num;

	pModbus_Slave->RspCode = Slave_RSP_OK; // 默认成功

	if (pModbus_Slave->RxCount < 6) // 接收小于6字节
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]); // 寄存器号
	// reg_num = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // 寄存器个数
	byte_num = pModbus_Slave->RxBuff[6];

	if (byte_num * 2 <= Parameter_Register_Number && reg <= Parameter_Register_Number)
	{
		for (uint16_t i = 0; i < byte_num; i++)
			Parameter_Register[(reg * 2) + i] = pModbus_Slave->RxBuff[7 + i]; // 写入
	}
	else
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR; // 寄存器地址错误
	}

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK)
		Modbus_SendAckOk(pModbus_Slave); // 正确应答
	else
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 告诉主机命令错误
}

/**
 * @brief 65H功能码解析
 *        内部函数
 */
static void Modbus_S_65H(_Modbus_Slave *pModbus_Slave)
{
	uint16_t reg;
	uint16_t value;

	pModbus_Slave->RspCode = Slave_RSP_OK; // 默认成功

	if (pModbus_Slave->RxCount < 6) // 接收小于6字节
	{
		pModbus_Slave->RspCode = Slave_RSP_ERR_VALUE; // 数据值域错误
		goto exit;
	}

	reg = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[2], pModbus_Slave->RxBuff[3]);   // 寄存器号
	value = MODBUS_Merge_array_16(pModbus_Slave->RxBuff[4], pModbus_Slave->RxBuff[5]); // 寄存器值

	if (Modbus_Write_Order_H_register(Calibration_Register, Calibration_Register_Number, reg, value) != 1) // 把写入的值存入寄存器
		pModbus_Slave->RspCode = Slave_RSP_ERR_REG_ADDR;												   // 寄存器地址错误

exit:
	if (pModbus_Slave->RspCode == Slave_RSP_OK)
		Modbus_SendAckOk(pModbus_Slave); // 正确应答
	else
		Modbus_SendAckErr(pModbus_Slave, pModbus_Slave->RspCode); // 告诉主机命令错误
}

/**
 * @brief 读寄存器数据
 *
 * @param Register 寄存器
 * @param Register_number 寄存器容量
 * @param pData 存储缓存
 * @param red_addr 读地址
 * @param red_number 读寄存器个数
 * @return uint8_t
 */
static uint8_t Modbus_Read_Register_value(__IO uint8_t *Register, uint16_t Register_number, uint8_t *pData, uint16_t red_addr, uint16_t red_number)
{
	uint16_t tim_data = 0;
	uint16_t data_len = 0;
	if (red_addr * 2 <= Register_number) // 地址小于总容量
	{
		for (uint16_t i = 0; i < red_number; i++)
		{
			tim_data = Register_Conversion_Read(Register, red_addr + i); // 索引回 指定寄存器的数据
			pData[data_len++] = tim_data >> 8;							 // 数据拆分存入缓存
			pData[data_len++] = tim_data;
		}
	}
	else
		return 0; // 参数异常，返回 0

	return 1; // 写成功
}

/**
 * @brief 写单寄存器 - 双字节
 *
 * @param Register 寄存器
 * @param Register_number 寄存器容量
 * @param reg_addr 地址
 * @param reg_value 数据
 * @return uint8_t
 */
static uint8_t Modbus_Write_Order_H_register(__IO uint8_t *Register, uint16_t Register_number, uint16_t reg_addr, uint16_t reg_value)
{
	if (reg_addr * 2 <= Register_number) // 地址小于总容量
	{
		Register_Conversion_Write(Register, reg_addr, reg_value);
	}
	else
		return 0; // 参数异常，返回 0

	return 1; // 写成功
}

/**
 * @brief 操作主寄存器数据中的1bit
 *
 * @param pData 主寄存器
 * @param pNomber 主寄存器号
 * @param pdataNO 要操作的位
 * @param flag 位的状态
 */
void HalfWord_Write_Bit(__IO uint8_t *pData, uint32_t pNomber, uint16_t pdataNO, uint16_t flag)
{
	uint16_t datas;
	if (pdataNO < 16 || (flag != 0 && flag != 1)) // 操作的位序号不能大于容量 且 位的状态只有 0 与 1
	{
		datas = Register_Conversion_Read(pData, pNomber); // 先将数据读出

		if (flag == 0)
			Clrbit(datas, pdataNO); // 将datas的第pdatas位清0
		else if (flag == 1)
			Setbit(datas, pdataNO); // 将datas的第pdatas位置1

		Register_Conversion_Write(pData, pNomber, datas); // 将数据重新写回
	}
}

/**
 * @brief : 操作主寄存器 保留某一位，操作其他位
 * @param  pData        主寄存器
 * @param  pNomber      主寄存器号
 * @param  keep_data    要保留的位
 * @param  flag         其他位的状态
 */
void HalfWord_Write_Keep_Bit_Set_Remaining(__IO uint8_t *pData, uint32_t pNomber, uint16_t keep_data, uint16_t flag)
{
	uint16_t datas;
	uint8_t _flag;

	if (keep_data < 16 || (flag != 0 && flag != 1)) // 保留的位序号不能大于容量 且 位的状态只有 0 与 1
	{
		datas = Register_Conversion_Read(pData, pNomber);	// 先将数据读出
		_flag = Modbus_HalfWord_Read_Bit(datas, keep_data); // 先将数据位取出

		if (flag == 0) // 其他位的状态为 0
		{
			datas = 0x0000;

			if (_flag == 0)
				Clrbit(datas, _flag); // 将要保留的位的状态写回
			else if (_flag == 1)
				Setbit(datas, _flag); // 将要保留的位的状态写回
		}
		else // 其他位的状态为 1
		{
			datas = 0xFFFF;

			if (_flag == 0)
				Clrbit(datas, _flag); // 将要保留的位的状态写回
			else if (_flag == 1)
				Setbit(datas, _flag); // 将要保留的位的状态写回
		}

		Register_Conversion_Write(pData, pNomber, datas); // 将数据重新写回
	}
}

/**
 * @brief 输出 16bit 数据中的 1Bit
 *
 * @param pdatas 数据
 * @param len 要输出的位
 * @return 位状态
 */
static uint8_t Modbus_HalfWord_Read_Bit(uint16_t pdatas, uint8_t len)
{
	uint8_t pData;

	if (len == 0)
		pData = pdatas & 0x01; // 最低位bai
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
 * @brief : 输出 32bit 数据中的 1Bit
 * @param  pdatas    数据
 * @param  len       要输出的位
 * @return 位状态
 */
static uint8_t Modbus_Word_Read_Bit(uint32_t pdatas, uint8_t len)
{
	uint8_t pData;
	pData = (pdatas & (0x00000001 << len)) >> len;

	return pData;
}

/**
 * @brief 发送错误应答
 *
 * @param _ucErrCode 错误代码
 */
static void Modbus_SendAckErr(_Modbus_Slave *pModbus_Slave, uint8_t _ucErrCode)
{
	uint8_t txbuf[3];

	txbuf[0] = pModbus_Slave->RxBuff[0];		// 地址
	txbuf[1] = pModbus_Slave->RxBuff[1] | 0x80; // 异常的功能码
	txbuf[2] = _ucErrCode;						// 错误代码(01,02,03,04)

	Modbus_SendWithCRC(pModbus_Slave, txbuf, 3);
}

/**
 * @brief 发送正确的应答
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
 * @brief 寄存器转换读 主号转寄存器 modbus 01H 发送使用
 *
 * @param Register 寄存器
 * @param pNomber 主寄存器号
 * @return uint16_t  寄存器值
 */
static uint16_t Modbus_Register_Conversion_01HRead(__IO uint8_t *Register, uint32_t pNomber)
{
	uint16_t pData;
	pData = ((Register[(pNomber * 2) + 1] << 8) | Register[(pNomber * 2)]); // 合并寄存器值
	return pData;
}

/**
 * @brief 寄存器转换读 主号转寄存器
 *
 * @param Register 寄存器
 * @param pNomber 主寄存器号
 * @return uint16_t  寄存器值
 */
uint16_t Register_Conversion_Read(__IO uint8_t *Register, uint32_t pNomber)
{
	uint16_t pData;
	pData = ((Register[(pNomber * 2)] << 8) | Register[(pNomber * 2) + 1]); // 合并寄存器值
	return pData;
}

/**
 * @brief 寄存器转换写 主号转寄存器
 *
 * @param Register 寄存器
 * @param pNomber 主寄存器号
 * @param pDATA 写入寄存器的值
 */
void Register_Conversion_Write(__IO uint8_t *Register, uint32_t pNomber, uint16_t pDATA)
{
	Register[(pNomber * 2)] = pDATA >> 8;
	Register[(pNomber * 2) + 1] = pDATA;
}

/**
**************************************************************************************************
* @Brief           单字节数据反转
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
 * @Brief       双字节数据反转
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
 * CRC校验
 * 名称： CRC-16/MODBUS
 * 多项式： x16 + x15 + x12 + 1
 * puchMsg 需要计算CRC的数组
 * usDataLen 计算数组中数值的个数
 * flip 输出高低位是否翻转
 * 输出 flip = 0 不翻转 低位在前  flip = 1 翻转 高位在前
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
	{							   // 翻转 高位在前
		t_data[0] = wCRCin >> 8;   // 高8位
		t_data[1] = wCRCin & 0xFF; // 低8位

		wCRCin = ((t_data[1] << 8) | t_data[0]);
	}

	return (wCRCin);
}

/*
 * 数组清零
 * *pData 数据
 * pNumber 个数  最多 65536 个
 */
static void MODBUS_Array_Zero(uint8_t *pData, uint32_t pNumber)
{
	for (uint32_t i = 0; i < pNumber; i++)
		pData[i] = 0;
}

/*
 * 合并两个8位 为16位
 * dataH 高位
 * dataL 低位
 */
static uint16_t MODBUS_Merge_array_16(uint8_t dataH, uint8_t dataL)
{
	uint16_t data = 0;
	data = ((dataH << 8) | dataL);
	return data;
}
