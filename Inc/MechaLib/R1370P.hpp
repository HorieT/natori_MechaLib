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
	static constexpr uint32_t TIMEOUT_TIME = 25;

	timeScheduler<void> _scheduler;
	std::array<uint8_t, GYRO_BUFF_SIZE> _buff;
	UART_HandleTypeDef* const _huart;

	void timeout_func(){
		_sensor_data_acceleration = {0.0f, 0.0f, 0.0f};
		_sensor_data_rot_velocity = {0.0f, 0.0f, 0.0f};
		_global_data_acceleration = {0.0f, 0.0f, 0.0f};
		_global_data_rot_velocity = {0.0f, 0.0f, 0.0f};
		_global_data_rot_position = {0.0f, 0.0f, 0.0f};
	}
public:
	R1370P(UART_HandleTypeDef* uart) : _scheduler([this]{timeout_func();}, TIMEOUT_TIME), _huart(uart){}
	~R1370P(){_scheduler.erase();}

	virtual void init() final{
		HAL_UART_Receive_DMA(_huart, _buff.data(), GYRO_BUFF_SIZE);
		_scheduler.set();
	}
	/*Å@	éÛêMä÷êî
	 *  HAL_UART_RxHalfCpltCallback()Ç≈åƒÇ—èoÇ∑
	 */
	virtual bool receive(std::any uart_handle) final{
		UART_HandleTypeDef* uart;

		uart = std::any_cast<UART_HandleTypeDef*>(uart_handle);
		/*
		try{
			uart = std::any_cast<UART_HandleTypeDef*>(uart_handle);
		}catch (std::bad_any_cast& e) {
			return false;
		}*/

		if(uart == _huart){
			std::array<uint8_t, GYRO_BUFF_SIZE> tmp_buff(_buff);
			uint32_t ndtr_ptr = _huart->hdmarx->Instance->NDTR;

			for(uint8_t j = 0;j < GYRO_BUFF_SIZE;++j){
				if((tmp_buff[j] == 0xAA) &&
						(tmp_buff[(j + 1) % GYRO_BUFF_SIZE] == 0x00) &&
						(((j + ndtr_ptr) % GYRO_BUFF_SIZE) < GYRO_DATA_SIZE)){
					std::array<uint8_t, GYRO_DATA_SIZE> read_data;
					uint8_t check_sum = 0;

					for(uint8_t i = 0;i < GYRO_DATA_SIZE - 2;i++)
						read_data[i] = tmp_buff[(j + i + 2) % GYRO_BUFF_SIZE];

					check_sum = static_cast<uint8_t>(std::accumulate(std::next(read_data.begin(), 1), std::next(read_data.end(), -3), 0));

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
						return true;
					}
				}
			}
		}
		return false;
	}
};
}
