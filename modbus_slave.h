/**
 * @File Name: modbus_slave.h
 * @Brief : 通信模块 (从站）
 * @Author : JYK (yakun81131@yeah.net)
 * @Version : 1.4
 * @Creat Date : 2022-03-5
 *
 */

#ifndef __MODBUY_SLAVE_H
#define __MODBUY_SLAVE_H
#include "main.h"

// ***  485 从机 收发缓存 ***
#define ModbusS_RXbuffer_size 100
#define ModbusS_TXbuffer_size 1024
extern uint8_t Modbus_Slave_485_RXbuffer[ModbusS_RXbuffer_size];
extern uint8_t Modbus_Slave_485_TXbuffer[ModbusS_TXbuffer_size];

typedef struct
{
	uint8_t Slave_ID;

	UART_HandleTypeDef *pUART_HandleTypeDef_Slave; // 串口号指针
	GPIO_TypeDef *DIR_GPIO_Port;				   // 485使能收发引脚类
	uint16_t DIR_GPIO_pins;						   // 485使能收发引脚号

	uint8_t *RxBuff;	   // 接收缓存
	__IO uint16_t RxCount; // 接收计数
	__IO uint8_t RxStatus; // 接收状态

	uint8_t *TxBuff;	   // 发送缓存
	__IO uint16_t TxCount; // 发送计数

	uint16_t RxBuff_Size; // 发送缓存容量
	uint16_t TxBuff_Size; // 接收缓存容量

	uint8_t Send_choose; // 发送选择

	__IO uint8_t RspCode; // 应答代码

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

#define RS485EN_ARE 0 // 有485使能
#define RS485EN_NO 1  // 无485使能

// == Modbus RTU 从机 应答代码 ==
#define Slave_RSP_OK 0				// 成功
#define Slave_RSP_ERR_CMD 0x01		// 不支持的功能码
#define Slave_RSP_ERR_REG_ADDR 0x02 // 寄存器地址错误
#define Slave_RSP_ERR_VALUE 0x03	// 数据值域错误
#define Slave_RSP_ERR_WRITE 0x04	// 写入失败

// == 功能码 ==
#define Slave_FUNC_01H 0x01 // 读功能状态
#define Slave_FUNC_02H 0x02 // 读输入
#define Slave_FUNC_03H 0x03 // 读设备
#define Slave_FUNC_04H 0x04 // 读传感器数据
#define Slave_FUNC_05H 0x05 // 开关功能码
#define Slave_FUNC_06H 0x06 // 写单个参数
#define Slave_FUNC_07H 0x07 // 读错误状态
#define Slave_FUNC_10H 0x10 // 写多个参数
#define Slave_FUNC_65H 0x65 // 校准模式

// == 操作寄存器状态 ==
#define Operation_Enable 0xFF00
#define Operation_Disabled 0x0000

// == 寄存器 数组  ==
#define Parameter_Register_Number 1024 // 参数寄存器容量
#define Data_Register_Number 10		   // 数据寄存器容量
#define Calibration_Register_Number 10 // 校准寄存器容量
#define Status_Register_Number 50	   // 状态寄存器容量
#define Operation_Register_Number 100  // 操作寄存器容量
#define Error_code_Register_Number 100 // 错误代码寄存器容量

#define Alert_OK 1	// 功能码被标志
#define Alert_ERR 0 // 功能码取消标志

extern __IO uint8_t Parameter_Register[Parameter_Register_Number];	   // 参数寄存器
extern __IO uint8_t Data_Register[Data_Register_Number];			   // 数据寄存器
extern __IO uint8_t Calibration_Register[Calibration_Register_Number]; // 校准寄存器
extern __IO uint8_t Status_Register[Status_Register_Number];		   // 状态寄存器
extern __IO uint8_t Operation_Register[Operation_Register_Number];	   // 操作寄存器
extern __IO uint8_t Error_code_Register[Error_code_Register_Number];   // 错误代码寄存器

// == 模块接口 ==

// 参数初始化 ***
void Modbus_Slave_init(_Modbus_Slave *pModbus_Slave, uint8_t pSlave_ID, UART_HandleTypeDef *pUART_HandleTypeDef_Slave, uint8_t *pRxBuff, uint8_t *pTxBuff, uint16_t pRxBuff_Size, uint16_t pTxBuff_Size, GPIO_TypeDef *pDIR_GPIO_Port, uint16_t pDIR_GPIO_pins, uint8_t send_choose);

// 串口中断回调 ***
void Modbus_Slave_Receive_IDLE(_Modbus_Slave *pModbus_Slave);

// Modbus 从站数据包解析 ***
void Modbus_Slave_Poll(_Modbus_Slave *pModbus_Slave);

// 寄存器转换读 主号转寄存器
uint16_t Register_Conversion_Read(__IO uint8_t *Register, uint32_t pNomber);

// 寄存器转换写 主号转寄存器
void Register_Conversion_Write(__IO uint8_t *Register, uint32_t pNomber, uint16_t pDATA);

// 操作主号寄存器的某一个位
void HalfWord_Write_Bit(__IO uint8_t *pData, uint32_t pNomber, uint16_t pdataNO, uint16_t flag);

// 操作主寄存器 保留某一位，操作其他位
void HalfWord_Write_Keep_Bit_Set_Remaining(__IO uint8_t *pData, uint32_t pNomber, uint16_t keep_data, uint16_t flag);

#endif
