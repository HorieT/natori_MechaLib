/*
 * –¢
 */
#pragma once

#include "MechaLib_HAL_global.hpp"

namespace Mecha{


template <size_t T>
class Ultrasonic{
private:
	std::array<IOPin, T> io;
	TIM_HandleTypeDef* tim;
	timeScheduler<Ultrasonic*> ts;
	std::array<double, T> distance = {0};
	std::array<int32_t, T> count = {0};
	bool flag_end = false;
	std::array<bool, T> flag_get = {false};


	void radiate(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		uint16_t pin = 0;
		flag_end = false;
		for(auto& f : flag_get)f = false;
		GPIO_InitStruct.Pin = 0;
		for(auto p : io)pin |= p.pin;

		GPIO_InitStruct.Pin = pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(io.at(0).port, &GPIO_InitStruct);

		tim->Instance->CNT = 0;
		tim->Instance->ARR = 10;

		HAL_GPIO_WritePin(io.at(0).port, pin, GPIO_PIN_SET);

		HAL_TIM_Base_Start_IT(tim);
	}
	static void call(Ultrasonic* us){us->radiate();}

	static constexpr double multi = 58;
	static constexpr double MAX = 38000;
public:
	Ultrasonic(TIM_HandleTypeDef* htim, std::array<IOPin, T> gpio): tim(htim), io(gpio){}

	void init(){}

	void radiateEND(TIM_HandleTypeDef *htim){
		if((tim->Instance->ARR == 10) && (htim == tim)){//o—ÍI—¹
			GPIO_InitTypeDef GPIO_InitStruct;
			uint16_t pin = 0;

			tim->Instance->CNT = 0;
			tim->Instance->ARR = 0xFFFF;
			flag_end = true;
			for(auto p : io)pin |= p.pin;

			HAL_TIM_Base_Start_IT(tim);
			HAL_TIM_Base_Stop(tim);
			HAL_GPIO_WritePin(io.at(0).port, pin, GPIO_PIN_RESET);
			GPIO_InitStruct.Pin = pin;
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(io.at(0).port, &GPIO_InitStruct);
		}
	}
	void timeout(TIM_HandleTypeDef *htim){
		if((tim->Instance->ARR == 0xFFFF) && (htim == tim) && flag_end){
			distance = 0;
			HAL_TIM_Base_Stop(tim);
			flag_end = false;
			for(auto& f : flag_get)f = false;
		}
	}
	void catch_sonic(uint16_t pin){
		if((tim->Instance->ARR == 0xFFFF) &&  flag_end){
			{
				uint8_t num = 0;
				for(auto p : io){
					if(pin == p.pin){
						if(p.read()){
							count.at(num) = tim->Instance->CNT;
							flag_get.at(num) = true;
						}
						else if(flag_get.at(num)){
							uint32_t cnt  = tim->Instance->CNT;
							distance.at(num) = ((cnt - count.at(num)) > MAX) ? 0 : (cnt - count.at(num)) / multi;
							HAL_TIM_Base_Stop(tim);
							count.at(num) = 0;
							flag_end = false;
							flag_get.at(num) = false;
						}
						break;
					}
				}
			}
		}
	}
	const double& get_distance(uint8_t num){return distance.at(num);}

};
}
