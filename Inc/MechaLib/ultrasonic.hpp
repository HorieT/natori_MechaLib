/*
 * 2019/07/12 Horie
 */
#pragma once

#include "MechaLib_HAL_global.hpp"
#include <tuple>
#include <bitset>

namespace Mecha{

/*
 * 大量の某係数分の在庫を抱えるseedの超音波センサのクラス
 * */
template <size_t T>
class ultrasonic{
	static_assert(T > 0, "Don't set 0 in ultrasonic-class's template.");
private:
	enum class sig_state : uint8_t{
		SEND_PULSE = 0U,
		CHATCH_RISE,
		CATCH_FALL
	};
	struct sensor{
		uint16_t _pin;

		int32_t _count = 0;
		float _distance = INFINITY;
		sig_state _got_flag = sig_state::CATCH_FALL;
	};

	static constexpr float COEFFICIENT = 58.0f;
	static constexpr float MAX = 38000.0f;

	TIM_HandleTypeDef* const _tim;
	//timeScheduler<ultrasonic*> _scheduler;

	GPIO_TypeDef* const _gpio;
	std::array<sensor, T> _line;

	/*
	 * トリガパルス放射
	 */
	void radiate(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		uint16_t pin = 0;

		for(auto& l : _line){
			l._got_flag = sig_state::SEND_PULSE;
			pin |= l._pin;
		}

		GPIO_InitStruct.Pin = pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(_gpio, &GPIO_InitStruct);

		_tim->Instance->CNT = 0;
		_tim->Instance->ARR = 10;

		HAL_GPIO_WritePin(_gpio, pin, GPIO_PIN_SET);

		HAL_TIM_Base_Start_IT(_tim);
	}
	static void call(ultrasonic* us){us->radiate();}

	void radiate_end(TIM_HandleTypeDef *htim){
		GPIO_InitTypeDef GPIO_InitStruct;
		uint16_t pin = 0;

		_tim->Instance->CNT = 0;
		_tim->Instance->ARR = 0xFFFF;
		for(auto l : _line)pin |= l.pin;

		HAL_TIM_Base_Stop(_tim);
		HAL_GPIO_WritePin(_gpio, pin, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(_gpio, &GPIO_InitStruct);
	}
	void timeout(TIM_HandleTypeDef *htim){
		HAL_TIM_Base_Stop(_tim);
		for(auto& l : _line){
			if(l.__got_flag != sig_state::CATCH_FALL){
				l._distance = INFINITY;
				l.__got_flag = sig_state::CATCH_FALL;
			}
		}
	}

public:
	ultrasonic(TIM_HandleTypeDef* htim, IOPin gpio): T(std::bitset<16>(gpio.pin).count()), _tim(htim), _gpio(gpio.port){
		uint8_t i = 0;
		for(auto& l : _line){
			while(1){
				if(uint16_t pin = 1U << i; pin & gpio.pin){
					l._pin = pin;
					break;
				}
				++i;
			}
		}
	}
	template<uint16_t... Args>
	ultrasonic(TIM_HandleTypeDef* htim, GPIO_TypeDef gpio, Args Pins) : T(sizeof(Pins)), _tim(htim), _gpio(gpio){
		std::array<uint16_t, T>pin = Pins;
		auto it = pin.begin();
		for(auto& l : _line)l._pin = *it++;
	}


	void interput_timer(TIM_HandleTypeDef *htim){
		if(htim == _tim){
			if(_tim->Instance->ARR == 10)radiate_end(htim);
			if(_tim->Instance->ARR == 0xFFFF)timeout(htim);
		}
	}


	void catch_sonic(uint16_t pin){
		if((_tim->Instance->ARR == 0xFFFF)){

			for(auto l : _line){
				if(pin == l.pin){
					if(HAL_GPIO_ReadPin(_gpio, pin) == GPIO_PIN_SET){
						l._count = _tim->Instance->CNT;
						l._got_flag = sig_state::CHATCH_RISE;
					}
					else if(l._got_flag == sig_state::CHATCH_RISE){
						uint32_t cnt  = _tim->Instance->CNT;
						l._distance = ((cnt - l._count) > MAX) ? 0 : (cnt - l._count) / COEFFICIENT;
						HAL_TIM_Base_Stop(_tim);
						l._count = 0;
						l._got_flag = sig_state::CATCH_FALL;
					}
					break;
				}
			}
		}
	}
	/*
	 * 距離取得
	 */
	const float& get_distance(uint8_t num){return _line.at(num)._distance;}

};
}
