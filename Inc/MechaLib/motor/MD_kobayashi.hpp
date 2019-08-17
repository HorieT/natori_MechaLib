/*
 *2019/06/27(horie)totyuu
 *小林MD ver2.x~3.x対応
 *角度系の関数はMD_kobayashiだと現在使えない
 */
#pragma once
#include "MechaLib/MechaLib_HAL_global.hpp"
#include "MechaLib/motor/motor.hpp"
#include <algorithm>

namespace Mecha{

//小林MDクラス
class MD_kobayashi : public motor{

private:
	CAN_HandleTypeDef* const _hcan;
	CAN_TxHeaderTypeDef _tx_header;
	std::array<uint8_t, 8> _rx_data= {0};
	uint32_t _tx_mailbox = 0U;

	virtual void speed_set() override{
		switch(_mode){
		case drive_mode::INPUT_DUTY:
		{
			int32_t duty = static_cast<int32_t>(_target_duty_per);
			write_data(md_address::DUTY, &duty);
			break;
		}
		case drive_mode::INPUT_RPM:
			write_data(md_address::PID_RPM, &_target_rpm);
			break;
		case drive_mode::INPUT_ANGLE:
		case drive_mode::EMERGENCY:
		default:
			write_data(md_address::EMERGENCY_STOP);
			break;
		}
	}

protected:
	enum class md_address : uint8_t{
		DUTY = 0U,
		PID_RPM,
		WRITE_P_GAIN,
		WRITE_I_GAIN,
		WRITE_D_GAIN,
		WRITE_MAX_ROT_RPM,
		WRITE_RESOLITION,
		EMERGENCY_STOP = 0x80
	};

	template<class T>
	void write_data(md_address adr, const T* data){
		std::array<uint8_t, sizeof(T)> buff = {0};
		buff.at(0) = static_cast<uint8_t>(adr);

		if(adr != md_address::EMERGENCY_STOP){
			_tx_header.DLC = 1 + sizeof(T);
			std::memcpy(&buff[0], data, sizeof(T));
			std::reverse(buff.begin(), buff.end());
		}else{
			_tx_header.DLC = 1;
		}

		while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);
		HAL_CAN_AddTxMessage(_hcan, &_tx_header, buff.data(), &_tx_mailbox);
	}
	void write_data(md_address adr){
		if(adr != md_address::EMERGENCY_STOP)return;

		std::array<uint8_t, 8> buff = {0};
		buff.at(0) = static_cast<uint8_t>(adr);
		_tx_header.DLC = 1;

		while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);
		HAL_CAN_AddTxMessage(_hcan, &_tx_header, buff.data(), &_tx_mailbox);
	}

public:
	MD_kobayashi(CAN_HandleTypeDef* can, uint32_t id, uint8_t period, uint32_t resolution, uint32_t limit) :
		motor(period, resolution, limit), _hcan(can){
		_tx_header.StdId = id;
		_tx_header.ExtId = 0x0U;
		_tx_header.RTR = CAN_RTR_DATA;
		_tx_header.IDE = CAN_ID_STD;
		_tx_header.TransmitGlobalTime = DISABLE;
	}
	virtual ~MD_kobayashi(){
		stop();
	}
	virtual void start() override{
		_target_duty_per = 0.0f;
		_target_rpm = 0;
		_target_rad = 0;
		write_data(md_address::WRITE_RESOLITION, &_resolution);
		_scheduler.set();
	}
	virtual void stop() override{
		_scheduler.erase();
		_target_duty_per = 0.0f;
		_target_rpm = 0;
		_target_rad = 0.0;
		write_data(md_address::EMERGENCY_STOP);
	}
	/*
	 * Don't use this function in MD_kobayashi subclass!
	 */
	virtual void set_rad(int32_t max_rpm, float rad) override{

	}
	virtual void set_PID (float p_gain, float i_gain, float d_gain) override{
		float p = p_gain, i = i_gain, d = d_gain;
		write_data(md_address::WRITE_P_GAIN, &p);
		sys_delayCall([this, &i]{write_data(md_address::WRITE_I_GAIN, &i);}, 10);
		sys_delayCall([this, &d]{write_data(md_address::WRITE_D_GAIN, &d);}, 20);
		/*sysClock::sys_delay(10);
		write_data(md_address::WRITE_I_GAIN, &i);
		sysClock::sys_delay(10);
		write_data(md_address::WRITE_D_GAIN, &d);*/
	}
	virtual void set_limitRPM(uint32_t limit) override{
		_limit_rpm = limit;
		write_data(md_address::WRITE_MAX_ROT_RPM, &_limit_rpm);
	}
	
	void read_CANdata(uint8_t* data){
		if(data[0] == _tx_header.StdId){
			int16_t oneTimeBox;
			std::memcpy(data, _rx_data.data(), 8);
			//int16_t input_speed;
			std::memcpy(&oneTimeBox, &data[2], 2);
			_read_rpm = (int32_t)oneTimeBox;
		}
	}
};





//エンコーダ制御型小林MD
class outEncMD_kobayashi : public MD_kobayashi{
private:
	TIM_HandleTypeDef* const _htim;
	const float _diameter;
	PID<float> _duty_pid;
	PID<float> _rad_pid;
	IOPin* oregin_limit = nullptr;
	IOPin* oregin_limit2 = nullptr;
	std::vector<std::pair<IOPin, void (*)(void)>> limit;
	int32_t rot_count = 0;

	//位置初期化フラグ
	bool pos_init_state = false;
	//範囲を-π~πに制限するか
	bool rot_overflow = false;
	//正転方向にリミットがあればtrue
	bool limit_polarity = true;

	virtual void speed_set() override{
		float rpm;
		uint32_t count = _htim->Instance->CNT;
		_htim->Instance->CNT = 0U;

		//32bit counter
		if(_htim->Instance->ARR == 0xFFFFFFFF){
			int32_t signedCount;
			std::memcpy(&signedCount, &count, 4);
			rot_count += signedCount;
			rpm = static_cast<float>(signedCount) * 60000.0f / static_cast<float>(_resolution * _scheduler.get_period());
		}
		//16bit counter
		else if(_htim->Instance->ARR == 0xFFFF){
			int16_t signedCount;
			std::memcpy(&signedCount, &count, 2);
			rot_count += static_cast<int32_t>(signedCount);
			rpm = static_cast<float>(signedCount) * 60000.0f / static_cast<float>(_resolution * _scheduler.get_period());
		}
		//例外的なリロードレジスタ処理
		else{
			int32_t signedCount = (count > (_htim->Instance->ARR / 2U)) ?
					-(static_cast<int32_t>(_htim->Instance->ARR - count)) : static_cast<int32_t>(count);
			rot_count += signedCount;
			rpm = static_cast<float>(signedCount) * 60000.0f / static_cast<float>(_resolution * _scheduler.get_period());
		}

		if(rot_overflow && (static_cast<uint32_t>(abs(rot_count)) > _resolution / 2U))
			rot_count %= static_cast<int32_t>(_resolution) / 2;

		_read_rpm = static_cast<int32_t>(rpm);
		_read_rad = static_cast<float>(rot_count) / static_cast<float>(_resolution) * static_cast<float>(M_PI) * 2.0f;


		auto limited_judgment =
				[this](float input){
					if(oregin_limit->read()){
						if((limit_polarity && (input > 0.0f)) || (!limit_polarity && (input < 0.0f))){
							write_data(md_address::EMERGENCY_STOP);
							return false;
						}
					}else if(oregin_limit2->read()){
						if((limit_polarity && (input < 0.0f)) || (!limit_polarity && (input > 0.0f))){
							write_data(md_address::EMERGENCY_STOP);
							return false;
						}
					}
					return true;
				};

		switch(_mode){
		case drive_mode::INPUT_DUTY:{
			_duty_pid.set(0.0f);
			_rad_pid.set(0.0f);
			if(!limited_judgment(_target_duty_per))return;
			int32_t duty = static_cast<int32_t>(_target_duty_per);
			write_data(md_address::DUTY, &duty);
			break;
		}

		case drive_mode::INPUT_ANGLE:
			_rad_pid.control(_read_rad - _target_rad, _scheduler.get_period());
			_duty_pid.control(_rad_pid.get() - rpm, _scheduler.get_period());
			[[fallthrough]];
		case drive_mode::INPUT_RPM:{
			if(_mode != drive_mode::INPUT_ANGLE){
				_rad_pid.set(0.0f);
				_duty_pid.control(static_cast<float>(_target_rpm) - rpm, _scheduler.get_period());
			}
			if(!limited_judgment(_duty_pid.get()))return;
			if(fabs(_duty_pid.get()) > 100.0f)_duty_pid.set(_duty_pid.get() < 0.0f ? -100.0f : 100.0f);
			int32_t duty = static_cast<int32_t>(_duty_pid.get());
			write_data(md_address::DUTY, &duty);
			write_data(md_address::PID_RPM, &_target_rpm);
			break;
		}
		case drive_mode::EMERGENCY:
		default:
			_duty_pid.set(0.0f);
			_rad_pid.set(0.0f);
			write_data(md_address::EMERGENCY_STOP);
			break;
		}
	}


public:
	outEncMD_kobayashi(CAN_HandleTypeDef* can, TIM_HandleTypeDef* tim, uint32_t id, uint8_t period, uint32_t resolution, uint32_t limit, float diameter):
	MD_kobayashi(can, id, period, resolution, limit), _htim(tim), _diameter(diameter), oregin_limit(new IOPin(nullptr, 0)), oregin_limit2(new IOPin(nullptr, 0)){}
	virtual ~outEncMD_kobayashi(){}


	virtual void start() override{
		_target_duty_per = 0.0f;
		_target_rpm = 0;
		_target_rad = 0;
		HAL_TIM_Encoder_Start(_htim, TIM_CHANNEL_ALL);
		_scheduler.set();
	}
	virtual void stop() override{
		_scheduler.erase();
		_target_duty_per = 0.0f;
		_target_rpm = 0;
		_target_rad = 0;
		_duty_pid.set(0.0f);
		_rad_pid.set(0.0f);
		_mode = drive_mode::EMERGENCY;
		write_data(md_address::EMERGENCY_STOP);
	}
	virtual void set_rad(int32_t max_rpm, float rad) override{
		if(static_cast<uint32_t>(abs(max_rpm)) > _limit_rpm)return;
		_target_rpm = max_rpm;
		_target_rad = rad;
		_mode = drive_mode::INPUT_ANGLE;
	}
	virtual void set_PID(float kp, float ki = 0, float kd = 0) override{_duty_pid.chage_gaine(kp, ki, kd);}
	virtual void set_radPID(float kp, float ki = 0, float kd = 0){_rad_pid.chage_gaine(kp, ki, kd);}
	virtual void set_limitRPM(uint32_t limit) override{_limit_rpm = limit;};





	void set_limit_io(IOPin io, void (*func)(void) = nullptr){
		limit.push_back(std::pair<IOPin, void (*)(void)>(io, func));
	}
	void set_limit_io(GPIO_TypeDef* port, uint16_t pin, void (*func)(void) = nullptr){
		limit.push_back(std::pair<IOPin, void (*)(void)>(IOPin(port, pin), func));
	}


	void set_oreginLimit_io(const IOPin& io, bool polarity){
		oregin_limit = new IOPin(io);
		limit_polarity = polarity;
	}
	void set_oreginLimit_io(GPIO_TypeDef* port, uint16_t pin, bool polarity){
		oregin_limit = new IOPin(port, pin);
		limit_polarity = polarity;
	}
	void set_oreginLimit2_io(const IOPin& io){
		oregin_limit2 = new IOPin(io);
	}
	void set_oreginLimit2_io(GPIO_TypeDef* port, uint16_t pin){
		oregin_limit2 = new IOPin(port, pin);
	}

	bool limit_check(uint16_t pin){
		if(oregin_limit->pin == pin && oregin_limit->read()){
			pos_init_state = true;
			rot_count = 0;
			set_duty(0);
			return true;
		}else if(oregin_limit2->pin == pin && oregin_limit2->read()){
			set_duty(0);
			return true;
		}
		for(auto l : limit){
			if(l.first.pin == pin){
				if(l.second != nullptr)l.second();
				else set_duty(0);
				return true;
			}
		}
		return false;
	}
	inline bool init_state(){return pos_init_state;}





	inline void set_pos(int32_t rpm, float pos){
		set_rad(rpm, pos * 2.0f / _diameter);
	}
	inline float get_pos(){
		return _diameter / 2.0f * _read_rad;
	}
};


}
