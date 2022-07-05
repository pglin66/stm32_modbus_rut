# 基于STM32 HAL 串口 Modbus RUT 从机

#### 介绍
制作一个好用的 Modbus RUT 从机模块

#### 软件架构
串口使用了 STM32的DMA不定长接收与空闲中断
支持485通讯


#### 安装教程

模块只有两个文件 modbus_slave.c 和 modbus_slave.h，该屏蔽已经屏蔽，只留出了寄存器操作、空闲循环和初始化函数可被更改​。​
支持创建多个从机；只需重新初始化，重新声明缓存与参数结构体即可。



初始化函数  
    ​    ​需添加到主循环前


```
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
`void Modbus_Slave_init(_Modbus_Slave *pModbus_Slave, uint8_t pSlave_ID, UART_HandleTypeDef *pUART_HandleTypeDef_Slave, uint8_t *pRxBuff, uint8_t *pTxBuff, uint16_t pRxBuff_Size, uint16_t pTxBuff_Size, GPIO_TypeDef *pDIR_GPIO_Port, uint16_t pDIR_GPIO_pins, uint8_t send_choose);`

```


空闲中断回调函数

    ​    ​为了不和其他模块的中断函数起冲突，所以没有用HAL库自带的中断回调，所以在使用的过程中，需要自行将中断回调添加到中断函数里​。


```
/**
 * @Brief : 接收空闲中断回调
 *          ***  每声明一个从机都要重新在中断函数里添加当前从机的中断回调 ***
 * @param  huart            串口指针
 * @param  pModbus_Slave    从站参数指针
 */
void Modbus_Slave_Receive_IDLE(_Modbus_Slave *pModbus_Slave);
```



空闲循环函数

    ​    ​为了使模块可以在需要的时候对主机发送的指令做出响应，那么需要将本函数添加到每个耗时的循环里​。至少作者是这样认为的，但是时也会有内部数据更新不及时就收到读取指令，那么数据就会有延时。根据需要权衡使用发发送方式​。


```
/**
 * @Brief : Modbus 从站数据包解析
 *       在主循环中循环调用  每个从机调用一次
 * @param  pModbus_Slave    从站参数指针
 */
void Modbus_Slave_Poll(_Modbus_Slave *pModbus_Slave);
```



多从机应用

    ​    ​当MCU需要多个串口接收指令，那么就需要多从机应用，那么只需要声明一个新的数据结构体，添加对应结构体的模块初始化，在需要的地方添加中断回调和空闲循环即可​。

#### 使用说明


```
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  Modbus_Slave_Receive_IDLE(&Modbus_Slave_485);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

```


#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
