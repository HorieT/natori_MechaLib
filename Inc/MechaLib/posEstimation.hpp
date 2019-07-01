/*
 * 2019/06/24 Horie
 */
#pragma once

#include "MechaLib_HAL_global.hpp"

namespace Mecha{

/*
 * �����փN���X
 */
struct measurWheel{
public:
	TIM_HandleTypeDef* const _htim;
	const uint32_t _resolution;
	const float _diameter;
	coordinate<float> _position;

	measurWheel(TIM_HandleTypeDef* tim, uint32_t resolution, float diameter_mm, coordinate<float> position):
	_htim(tim), _resolution(resolution), _diameter(diameter_mm), _position(position){}
	virtual ~measurWheel(){HAL_TIM_Encoder_Stop(_htim, TIM_CHANNEL_ALL);}

	void init(){HAL_TIM_Encoder_Start(_htim, TIM_CHANNEL_ALL);}
	float get_distance(){
		uint32_t count = _htim->Instance->CNT;
		_htim->Instance->CNT = 0;

		//32bit counter
		if(_htim->Instance->ARR == 0xFFFFFFFF){
			int32_t signedCount;
			std::memcpy(&signedCount, &count, 4);
			return static_cast<float>(signedCount) * _diameter * static_cast<float>(M_PI) / static_cast<float>(_resolution);
		}
		//16bit counter
		else if(_htim->Instance->ARR == 0xFFFF){
			int16_t signedCount;
			std::memcpy(&signedCount, &count, 2);
			return static_cast<float>(signedCount) * _diameter * static_cast<float>(M_PI) / static_cast<float>(_resolution);
		}
		//��O�I�ȃ����[�h���W�X�^����
		else{
			int32_t signedCount = (count > (_htim->Instance->ARR / 2U)) ?
					static_cast<int32_t>(count - _htim->Instance->ARR) : static_cast<int32_t>(count);
			return static_cast<float>(signedCount) * _diameter * static_cast<float>(M_PI) / static_cast<float>(_resolution);
		}
	}
};

/*
 * �R���{�Z���T���ۃN���X
 */
class multiAxisComboSensor{
protected:
	Eigen::Vector3f _sensor_data_acceleration;
	Eigen::Vector3f _sensor_data_rot_velocity;
	Eigen::Vector3f _global_data_acceleration;
	Eigen::Vector3f _global_data_rot_velocity;
	Eigen::Vector3f _global_data_rot_position;

public:
	multiAxisComboSensor() :
		_sensor_data_acceleration(0.0f, 0.0f, 0.0f),
		_sensor_data_rot_velocity(0.0f, 0.0f, 0.0f),
		_global_data_acceleration(0.0f, 0.0f, 0.0f),
		_global_data_rot_position(0.0f, 0.0f, 0.0f){}
	virtual ~multiAxisComboSensor(){}

	const Eigen::Vector3f& get_acc_data() const{return _global_data_acceleration;}
	const Eigen::Vector3f& get_rot_vel_data() const{return _global_data_rot_velocity;}
	const Eigen::Vector3f& get_rot_pos_data() const{return _global_data_rot_position;}
};


/*
 * ���Ȉʒu�v�Z�N���X
 */
class SelfPos{
private:
	timeScheduler<SelfPos*> _scheduler;
	multiAxisComboSensor* const _sensor;
	std::array<measurWheel*, 2> _encoder;

	std::array<float, 2> _encoder_radPos;//�ʒu�p
	std::array<float, 2> _encoder_norm;//���K������

	coordinate<float> _position;//���Ȉʒu
	coordinate<float> _velocity = {0};//���ȑ��x
	const float _initial_rad;//�����p�x

	float _sin_encoder_directiion_diff;

	std::array<float, 2> _cos_encoder_direction;
	std::array<float, 2> _sin_encoder_direction;
	std::array<float, 2> _cos_encoder_rad;
	std::array<float, 2> _sin_encoder_rad;


	static void scheduler_fanc(SelfPos* me){
		me->calculation_pos();
	}
	/*
	 *�@�ʒu�v�Z
	 */
	void calculation_pos(){
		//�e�����ւ̎����l
		std::array<float, 2> distance = {_encoder.at(0)->get_distance(), _encoder.at(1)->get_distance()};
		//���݈ʒu����p
		coordinate<float> now_pos = _position;
		now_pos.direction_rad = _initial_rad + _sensor->get_rot_pos_data().z();
		if(fabs(now_pos.direction_rad) > static_cast<float>(M_PI))now_pos.direction_rad += (now_pos.direction_rad > 0.0f) ? -2.0f*static_cast<float>(M_PI) : 2.0f*static_cast<float>(M_PI);
		//�P�ʎ��ԑ��x
		const float rot_vel = _sensor->get_rot_vel_data().z() * static_cast<float>(_scheduler.get_period()) / 1000.0f;
		//�O��ʒu
		const coordinate<float> last_pos = _position;

		//�����֊e�X�̃x�N�g��
		std::array<Eigen::Vector2f, 2> distance_vec{
			Eigen::Vector2f{distance.at(0) * _cos_encoder_direction.at(0), distance.at(0) * _sin_encoder_direction.at(0)},
			Eigen::Vector2f{distance.at(1) * _cos_encoder_direction.at(1), distance.at(1) * _sin_encoder_direction.at(1)}};


		for(uint8_t i = 0;i < 2;++i){
			//�G���R�[�_��]����
			float rot_len = rot_vel * _encoder_norm.at(i);
			Eigen::Vector2f calc_vec =
					distance_vec.at(i) - Eigen::Vector2f(rot_len * -_sin_encoder_rad.at(i), rot_len * _cos_encoder_rad.at(i));
			distance.at(i) = sqrtf(calc_vec.x() * calc_vec.x() + calc_vec.y() * calc_vec.y()) * cosf(_encoder.at(i)->_position.direction_rad - atan2f(calc_vec.y(), calc_vec.x()));
		}
		now_pos.x +=
				(distance.at(0) * sinf(_encoder.at(1)->_position.direction_rad + last_pos.direction_rad) - distance.at(1) * sinf(_encoder.at(0)->_position.direction_rad + last_pos.direction_rad)) /
				(-_sin_encoder_directiion_diff);
		now_pos.y +=
				(distance.at(0) * cosf(_encoder.at(1)->_position.direction_rad + last_pos.direction_rad) - distance.at(1) * cosf(_encoder.at(0)->_position.direction_rad+ last_pos.direction_rad)) /
				(_sin_encoder_directiion_diff);


		_position = now_pos;
		_velocity = (now_pos - last_pos) / static_cast<float>(_scheduler.get_period()) * 1000.0f;
	}



public:
	/*
	 * �R���X�g���N�^
	 */
	SelfPos(multiAxisComboSensor* gyro, std::array<measurWheel*, 2> enc, uint32_t period, coordinate<float> start):
	_scheduler(scheduler_fanc, period), _sensor(gyro), _initial_rad(start.direction_rad){
		_encoder.at(0)= enc.at(0);
		_encoder.at(1)= enc.at(1);
		_position = start;
		//�e�r���l�v�Z
		for(uint8_t i = 0;i < 2;++i){
			_encoder_radPos.at(i) = _encoder.at(i)->_position.angle();
			_encoder_norm.at(i) = _encoder.at(i)->_position.norm();
			_cos_encoder_direction.at(i) = cosf(_encoder.at(i)->_position.direction_rad);
			_sin_encoder_direction.at(i) = sinf(_encoder.at(i)->_position.direction_rad);
			_cos_encoder_rad.at(i) = cosf(_encoder_radPos.at(i));
			_sin_encoder_rad.at(i) = sinf(_encoder_radPos.at(i));
			_sin_encoder_directiion_diff = sinf(_encoder.at(0)->_position.direction_rad - _encoder.at(1)->_position.direction_rad);
		}
	}
	SelfPos(multiAxisComboSensor* gyro, measurWheel* enc1, measurWheel* enc2, uint32_t period, coordinate<float> start):
		_scheduler(scheduler_fanc, period), _sensor(gyro), _initial_rad(start.direction_rad){
		_encoder.at(0)= enc1;
		_encoder.at(1)= enc2;
		_position = start;
		//�e�r���l�v�Z
		for(uint8_t i = 0;i < 2;++i){
			_encoder_radPos.at(i) = _encoder.at(i)->_position.angle();
			_encoder_norm.at(i) = _encoder.at(i)->_position.norm();
			_cos_encoder_direction.at(i) = cosf(_encoder.at(i)->_position.direction_rad);
			_sin_encoder_direction.at(i) = sinf(_encoder.at(i)->_position.direction_rad);
			_cos_encoder_rad.at(i) = cosf(_encoder_radPos.at(i));
			_sin_encoder_rad.at(i) = sinf(_encoder_radPos.at(i));
			_sin_encoder_directiion_diff = sinf(_encoder.at(0)->_position.direction_rad - _encoder.at(1)->_position.direction_rad);
		}
	}
	/*
	 * �f�X�g���N�^
	 */
	virtual ~SelfPos(){_scheduler.erase();}

	/*
	 * ��������
	 */
	void init(){_scheduler.set(this);}


	const coordinate<float>& get_pos(){return _position;}
	const coordinate<float>& get_vel(){return _velocity;}

	/*
	 * �ʒu�đ��
	 */
	void reset_pos(const coordinate<float>& position){_position = position;}
};
}
