/*
 *2018/12/9(horie)
 *���[�^�[�̒��ۊ��N���X(�G���R�[�_�[���ĂȂ���Ȃ���ˁH)
 *���̑����[�^�[�g�����
 */
#pragma once
#include "MechaLib/sys_timer.hpp"
#include "MechaLib/Base/calculation.hpp"


namespace Mecha{

/*
 * ���[�^�[���ۃN���X
 */
class motor{
private:
	//�q�N���X�̃^�C���X�P�W���[���֐�
	virtual void speed_set() = 0;

protected:
	enum class drive_mode : uint8_t{
		EMERGENCY = 0,
		INPUT_DUTY,
		INPUT_RPM,
		INPUT_ANGLE
	};

	drive_mode _mode = drive_mode::EMERGENCY;
	timeScheduler<void> _scheduler;
	const uint32_t _resolution;//�G���R�[�_����\.

	float _target_duty_per = 0.0f;//�ڕW�f���[�e�B��[%]
	int32_t _target_rpm = 0;//�ڕW��]��[RPM]
	float _target_rad = 0.0f;//�ڕW�p�x[rad]
	int32_t _read_rpm = 0;//���ݑ��x
	float _read_rad = 0.0f;//���݊p�x
	uint32_t _limit_rpm;//��]������i�ڕW�ɂ̂ݗL���j

public:
	motor(uint8_t period, uint32_t resolution, uint32_t limit)
	: _scheduler([this]{speed_set();}, period), _resolution(resolution), _limit_rpm(limit){}
	virtual ~motor(){
		_scheduler.erase();
	}

	//�J�n����
	virtual void start() = 0;
	//��~,�I������
	virtual void stop() = 0;
	//duty����
	virtual void set_duty(float duty_per){
		if(fabs(duty_per) > 100.0f)return;
		_target_duty_per = duty_per;
		_mode = drive_mode::INPUT_DUTY;
	}
	//RPM����(PID)
	virtual void set_RPM(int32_t rpm){
		if(static_cast<uint32_t>(abs(rpm)) > _limit_rpm)return;
		_target_rpm = rpm;
		_mode = drive_mode::INPUT_RPM;
	}
	//�ڕW�p����
	virtual void set_rad(int32_t max_rpm, float rad){
		if(static_cast<uint32_t>(abs(max_rpm)) > _limit_rpm)return;
		_target_rpm = max_rpm;
		_target_rad = rad;
		_mode = drive_mode::INPUT_ANGLE;
	}

	//�Q�C���ݒ�
	virtual void set_PID(float p_gain, float i_gain, float d_gain) = 0;
	//��]������ݒ�(�ڕW�ɂ̂ݗL��)
	virtual void set_limitRPM(uint32_t limit){_limit_rpm = limit;}

	//��]���擾
	inline virtual int32_t get_RPM() const final{return _read_rpm;}
	//�p�x�擾
	inline virtual float get_rad() const final{return _read_rad;}
};



/*
 * �^�C���̃N���X
 */
class DriveWheel{
private:
	motor* const _drive_motor;
	const float _diameter_mm;
public:
	DriveWheel(motor* d_mo, float dia_mm) : _drive_motor(d_mo), _diameter_mm(dia_mm){}
	~DriveWheel(){}

	void set_mps(float mps){
		_drive_motor->set_RPM(static_cast<int32_t>(mps * 60000.0f / (_diameter_mm * static_cast<float>(M_PI))));
	}
	float get_mps() const{
		return static_cast<float>(_drive_motor->get_RPM()) / 60.0f * _diameter_mm * static_cast<float>(M_PI);
	}
};
}
