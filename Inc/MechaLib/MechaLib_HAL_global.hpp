/*
 * 2019/06/27
 * Mecha library��HAL�p�}�X�^�[
 * HAL�̃C���N���[�h���b�@�\�̎���
 */
#pragma once

/*
 * HAL���C�u������c++�Ŏg�p���邽�߂�extern
 */
extern "C"{
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#elif defined STM32F042x6
#include "stm32f0xx_hal.h"
#endif
}

#include "sys_timer.hpp"
#include "Base/calculation.hpp"

namespace Mecha{

/*
 * GPIO�\����
 */
class IOPin{
public:
	GPIO_TypeDef* const	port;
	const uint16_t		pin;

	/*
	 * �R���X�g���N�^
	 * ������:�|�[�g�A�h���X, ������:�s��bit
	 */
	IOPin(GPIO_TypeDef* io_port, uint16_t io_pin):port(io_port), pin(io_pin){};

	inline void write(bool state) const{port->BSRR = pin << (state ? 0 : 16);}//HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
	inline bool read() const{return (port->IDR | pin) != 0;}//HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
	inline void toggle() const{port->ODR ^= pin;}//HAL_GPIO_TogglePin(port, pin);
};

/*�ȉ�HAL�ŕs�ւȂƂ����b�v�����z*/

inline void CAN_sendData(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t data[], uint8_t size){
	uint32_t            TxMailbox = 0;
	CAN_TxHeaderTypeDef TxHeader;

	TxHeader.StdId = id;
	TxHeader.ExtId = 0x00;
	TxHeader.RTR = 0x0U;//CAN_RTR_DATA;
	TxHeader.IDE = 0x0U;//CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;

	TxHeader.DLC = size;

	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);
	HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
}
inline void CAN_sendData(CAN_HandleTypeDef* hcan, uint32_t id, std::vector<uint8_t> &data){
	uint32_t            TxMailbox = 0;
	CAN_TxHeaderTypeDef TxHeader;

	TxHeader.StdId = id;
	TxHeader.ExtId = 0x00;
	TxHeader.RTR = 0x0U;//CAN_RTR_DATA;
	TxHeader.IDE = 0x0U;//CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;

	TxHeader.DLC = data.size();

	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);
	HAL_CAN_AddTxMessage(hcan, &TxHeader, data.data(), &TxMailbox);
}
}


