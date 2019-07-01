/*
 * 未
 */
#pragma once


#include "MechaLib_HAL_global.hpp"
#include "calculation.hpp"
#include <vector>
#include <array>

namespace Mecha{


class RS40xCB{
public:
	typedef enum{
		/*const memory*/
		MODEL_NUMBER_L = 0U,
		MODEL_NUMBER_H,
		FIRMWARE_VERSION,
		/*ROM*/
		ID = 0x04,
		BAUD_RATE = 0x06,
		RETURN_DELAY,
		CW_ANGLE_LIMIT_L,
		CW_ANGLE_LIMIT_H,
		CCW_ANGLE_LIMIT_L,
		CCW_ANGLE_LIMIT_H,
		TEMPERATURE_LIMIT_L = 0x0E,
		TEMPERATURE_LIMIT_H,
		DAMPER = 0x14,
		TORQUE_IN_SILENCE = 0x16,
		WARM_UP_TIME,
		CW_COMPLIANCE_MARGIN,
		CCW_COMPLIANCE_MARGIN,
		CW_COMPLIANCE_SLOPE,
		CCW_COMPLIANCE_SLOPE,
		PUNCH_L,
		PUNCH_H,
		/*RAM*/
		GOAL_POSITION_L,
		GOAL_POSITION_H,
		GOAL_TIME_L,
		GOAL_TIME_H,
		MAX_TORQUE = 0x23,
		TORQUE_ENABLE,
		PRESENT_POSITION_L = 0x2A,
		PRESENT_POSITION_H,
		PRESENT_TIME_L,
		PRESENT_TIME_H,
		PRESENT_SPEED_L,
		PRESENT_SPEED_H,
		PRESENT_CURRENT_L,
		PRESENT_CURRENT_H,
		PRESENT_TEMPERATURE_L,
		PRESENT_TEMPERATURE_H,
		PRESENT_VOLTS_L,
		PRESENT_VOLTS_H
	}memory_adr;

	typedef enum{
		RETURN_ADR_1 = 0x01,
		RETURN_ADR_2 = 0x02,
		RETURN_ADR_3 = 0x04,
		RETURN_ADR_4 = 0x08,
		RETURN_ADR_ALL = 0x0F,
		MEMORY_INITIALIZE = 0x10,
		REBOOT = 0x20,
		WRITE_FLASH_ROM = 0x40
	}flags;

	typedef enum{
		READ_PACET_ERROR = 0x02,
		WRITE_ROM_ERROR = 0x08,
		TEMPERATURE_LIMIT_ARRAM = 0x20,
		TEMPERATURE_LIMIT_ERROR = 0x80
	}return_flags;

	typedef struct{
		uint8_t id;
		std::vector<uint8_t>* data;
	}data_pack;

	static constexpr uint8_t ID_ALL = 0xFF;
	static constexpr double RAD_LIMIT = (150.0 / 180.0 * M_PI);
private:
	UART_HandleTypeDef* uart;
	IOPin io;


public:
	/*ショートパケット*/
	void send_data(uint8_t id, uint8_t adr, std::vector<uint8_t> data){
		std::vector<uint8_t> packet;
		packet.reserve(8 + data.size());

		/*header*/
		packet.push_back(0xFA);
		packet.push_back(0xAF);

		/*servo id*/
		packet.push_back(id);
		/*flags*/
		packet.push_back(0);
		/*address*/
		packet.push_back(adr);
		/*length*/
		packet.push_back((uint8_t)data.size());
		/*count*/
		packet.push_back(1);
		/*data*/
		for(auto d : data)packet.push_back(d);

		/*sum*/
		packet.push_back(0);
		for(uint8_t i = 2;i < (packet.size() - 1);++i)
			packet.back() ^= packet.at(i);

		io.write(true);
		//sysClock::sys_delay(1);
		HAL_UART_Transmit(uart, packet.data(), (uint16_t)packet.size(), 5);
		//sysClock::sys_delay(5);
		io.write(false);
	}
	template <typename T>
	void send_data(uint8_t id, uint8_t adr, T data){
			std::vector<uint8_t> packet;
			packet.reserve(8 + sizeof(T));

			/*header*/
			packet.push_back(0xFA);
			packet.push_back(0xAF);

			/*servo id*/
			packet.push_back(id);
			/*flags*/
			packet.push_back(0);
			/*address*/
			packet.push_back(adr);
			/*length*/
			packet.push_back(sizeof(T));
			/*count*/
			packet.push_back(1);
			/*data*/
			packet.resize(7 + sizeof(T));
			memcpy(&packet[7], &data, sizeof(T));

			/*sum*/
			packet.push_back(0);
			for(uint8_t i = 2;i < (packet.size() - 1);++i)
				packet.back() ^= packet.at(i);

			io.write(true);
			//sysClock::sys_delay(1);
			HAL_UART_Transmit(uart, packet.data(), (uint16_t)packet.size(), 5);
			//sysClock::sys_delay(5);
			io.write(false);
		}


	/*ロングパケット*/
	void send_data(std::vector<uint8_t> id, uint8_t adr, std::vector<std::vector<uint8_t>> data){
		std::vector<uint8_t> packet;
		uint8_t length = (uint8_t)(data.at(0).size() + 1U);
		packet.reserve(8 + length * id.size());

		/*header*/
		packet.push_back(0xFA);
		packet.push_back(0xAF);

		/*servo id*/
		packet.push_back(0);
		/*flags*/
		packet.push_back(0);
		/*address*/
		packet.push_back(adr);
		/*length*/
		packet.push_back(length);
		/*count*/
		packet.push_back((uint8_t)id.size());

		/*data*/
		uint8_t data_count = 0;
		for(auto servo : id){
			packet.push_back(servo);
			for(auto d : data.at(data_count))packet.push_back(d);
			data_count++;
		}
		/*sum*/
		packet.push_back(0);
		for(uint8_t i = 2;i < (packet.size() - 1);++i)
			packet.back() ^= packet.at(i);

		io.write(true);
		//sysClock::sys_delay(1);
		HAL_UART_Transmit(uart, packet.data(), (uint16_t)packet.size(), 5);
		//sysClock::sys_delay(5);
		io.write(false);
	}
	template<typename T>
	void send_data(std::vector<uint8_t> id, uint8_t adr, std::vector<T> data){
		std::vector<uint8_t> packet;
		uint8_t length = sizeof(T) + 1;
		packet.reserve(8 + length * id.size());

		/*header*/
		packet.push_back(0xFA);
		packet.push_back(0xAF);

		/*servo id*/
		packet.push_back(0);
		/*flags*/
		packet.push_back(0);
		/*address*/
		packet.push_back(adr);
		/*length*/
		packet.push_back(length);
		/*count*/
		packet.push_back((uint8_t)id.size());

		/*data*/
		uint8_t data_count = 0;
		for(auto servo : id){
			packet.push_back(servo);
			uint16_t lead = (uint16_t)packet.size();
			packet.resize(lead + sizeof(T));
			memcpy(&packet[lead], &data[data_count], sizeof(T));
			data_count++;
		}
		/*sum*/
		packet.push_back(0);
		for(uint8_t i = 2;i < (packet.size() - 1);++i)
			packet.back() ^= packet.at(i);

		io.write(true);
		//sysClock::sys_delay(1);
		HAL_UART_Transmit(uart, packet.data(), (uint16_t)packet.size(), 5);
		//sysClock::sys_delay(5);
		io.write(false);
	}
	/*特殊操作*/
	void send_data(uint8_t id, uint8_t adr, flags flag, size_t len = 0, uint8_t* buff = nullptr){
		std::vector<uint8_t> packet;
		packet.reserve(8);

		/*header*/
		packet.push_back(0xFA);
		packet.push_back(0xAF);

		/*servo id*/
		packet.push_back(id);
		/*flags*/
		packet.push_back(flag);
		/*address*/
		packet.push_back(0xFF);
		/*length*/
		if((flag & MEMORY_INITIALIZE) != 0)packet.push_back(0xFF);
		else if((flag & RETURN_ADR_ALL) == RETURN_ADR_ALL)packet.push_back((uint8_t)len);
		else packet.push_back(0);
		/*count*/
		packet.push_back(0);
		/*sum*/
		packet.push_back(0);
		for(uint8_t i = 2;i < (packet.size() - 1);++i)
			packet.back() ^= packet.at(i);

		/*リターンパケット設定*/
		if((flag | RETURN_ADR_ALL) != 0)HAL_UART_Receive_DMA(uart, buff, (uint16_t)len + 8U);


		io.write(true);
		//sysClock::sys_delay(1);
		HAL_UART_Transmit(uart, packet.data(), (uint16_t)packet.size(), 5U);
		//sysClock::sys_delay(5);
		io.write(false);
	}


public:
	RS40xCB(UART_HandleTypeDef* huart, GPIO_TypeDef* port, uint16_t pin):uart(huart), io(port, pin){}
	RS40xCB(UART_HandleTypeDef* huart, IOPin pin):uart(huart), io(pin){}
	~RS40xCB(){}

	/*flag処理系*/
	void write_rom(uint8_t id){
		send_data(id, (uint8_t)0, WRITE_FLASH_ROM);
	}
	void reboot(uint8_t id){
		send_data(id, (uint8_t)0, REBOOT);
	}
	void read_mem(uint8_t id, uint8_t adr, std::vector<uint8_t> data){
		send_data(id, adr, RETURN_ADR_ALL, data.size(), data.data());
	}


	/*常用書き込み類*/
	void change_id(uint8_t old_id, uint8_t new_id){
		send_data(old_id, ID, (char)new_id);
		sysClock::sys_delay(30);
		write_rom(new_id);
	}
	bool set_move(uint8_t id, double rad, uint32_t ms){
		if(fabs(rad) > (150.0 / 180.0 * M_PI))return false;


		int16_t deg_data = (int16_t)(rad / M_PI * 1800.0);
		uint16_t speed_data = (uint16_t)(ms / 10.0);

		std::vector<uint8_t> packet_data(4);
		std::memcpy(&packet_data[0], &deg_data, 2);
		std::memcpy(&packet_data[2], &speed_data, 2);

		send_data(id, GOAL_POSITION_L, packet_data);
		return true;
	}
	void set_torque(uint8_t id, bool torque){
		send_data(id, TORQUE_ENABLE, torque ? (uint8_t)0x01 : (uint8_t)0x00);
	}
};


}
