/*
 * 2019/07/13 Horie
 */
#pragma once

#include "MechaLib_HAL_global.hpp"
#include "posEstimation.hpp"

namespace Mecha{

class R1370P : public multiAxisComboSensor{
private:
	static constexpr uint8_t GYRO_DATA_SIZE = 15;
	static constexpr uint8_t GYRO_BUFF_SIZE = GYRO_DATA_SIZE * 2;
	static constexpr uint8_t GYRO_BUFF_SIZE_D = GYRO_BUFF_SIZE - 1;

	timeScheduler<R1370P*> _scheduler;
	std::array<uint8_t, GYRO_BUFF_SIZE> _buff;
	UART_HandleTypeDef* const _huart;

	static void scheduler_fanc(R1370P* me){me->timeout_func();}
	void timeout_func(){
		_sensor_data_acceleration = {0.0f, 0.0f, 0.0f};
		_sensor_data_rot_velocity = {0.0f, 0.0f, 0.0f};
		_global_data_acceleration = {0.0f, 0.0f, 0.0f};
		_global_data_rot_velocity = {0.0f, 0.0f, 0.0f};
		_global_data_rot_position = {0.0f, 0.0f, 0.0f};
	}
public:
	R1370P(UART_HandleTypeDef* uart) : _scheduler(scheduler_fanc, 25), _huart(uart){}
	~R1370P(){_scheduler.erase();}

	void init(){
		HAL_UART_Receive_DMA(_huart, _buff.data(), GYRO_BUFF_SIZE);
		_scheduler.set(this);
	}
	bool receive(UART_HandleTypeDef* uart){
		bool state = false;
		if(uart == _huart){
			std::array<uint8_t, GYRO_BUFF_SIZE> hold_buff(_buff);
			uint32_t ndtr_ptr = _huart->hdmarx->Instance->NDTR;

			for(uint8_t j = 0;j < GYRO_BUFF_SIZE;++j){
				if((hold_buff[j] == 0xAA) &&
						(hold_buff[(j + 1) % GYRO_BUFF_SIZE] == 0x00) &&
						(((j + ndtr_ptr) % GYRO_BUFF_SIZE) < GYRO_DATA_SIZE)){
					std::array<uint8_t, GYRO_DATA_SIZE> read_data;
					uint8_t check_sum = 0;

					for(uint8_t i = 0;i < GYRO_DATA_SIZE - 2;i++)
						read_data[i] = hold_buff[(j + i + 2) % GYRO_BUFF_SIZE];

					/*
					for(uint8_t i = 0;i < GYRO_DATA_SIZE - 3;i++)
						check_sum = static_cast<uint8_t>(check_sum + read_data[i]);
						Å´
					*/
					for(auto it = std::next(read_data.begin(), 1), e = std::next(read_data.end(), -3);it != e;++it)
						check_sum = static_cast<uint8_t>(check_sum + *it);
					if(read_data[12] == check_sum){
						_scheduler.reset();
						int16_t angle, angle_vel, acc;
						memcpy(&angle, &read_data[1], 2);
						memcpy(&angle_vel, &read_data[3], 2);
						_global_data_rot_position.z() = - static_cast<float>(angle) * 0.01f * static_cast<float>(M_PI) / 180.0f;
						_global_data_rot_velocity.z()=  - static_cast<float>(angle_vel) * 0.01f * static_cast<float>(M_PI) / 180.0f;

						memcpy(&acc, &read_data[5], 2);
						_sensor_data_acceleration.x() = static_cast<float>(acc);
						memcpy(&acc, &read_data[7], 2);
						_sensor_data_acceleration.y() = static_cast<float>(acc);
						memcpy(&acc, &read_data[9], 2);
						_sensor_data_acceleration.z() = static_cast<float>(acc);
						state = true;
						break;
					}
				}
			}
		}
		return state;
	}
};
}
