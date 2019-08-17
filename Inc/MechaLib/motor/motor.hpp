/*
 *2018/12/9(horie)
 *モーターの抽象基底クラス(エンコーダーついてない訳ないよね？)
 *その他モーター使うやつ
 */
#pragma once
#include "MechaLib/sys_timer.hpp"
#include "MechaLib/Base/calculation.hpp"


namespace Mecha{

/*
 * モーター抽象クラス
 */
class motor{
private:
	//子クラスのタイムスケジューラ関数
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
	const uint32_t _resolution;//エンコーダ分解能.

	float _target_duty_per = 0.0f;//目標デューティ比[%]
	int32_t _target_rpm = 0;//目標回転数[RPM]
	float _target_rad = 0.0f;//目標角度[rad]
	int32_t _read_rpm = 0;//現在速度
	float _read_rad = 0.0f;//現在角度
	uint32_t _limit_rpm;//回転数上限（目標にのみ有効）

public:
	motor(uint8_t period, uint32_t resolution, uint32_t limit)
	: _scheduler([this]{speed_set();}, period), _resolution(resolution), _limit_rpm(limit){}
	virtual ~motor(){
		_scheduler.erase();
	}

	//開始処理
	virtual void start() = 0;
	//停止,終了処理
	virtual void stop() = 0;
	//duty入力
	virtual void set_duty(float duty_per){
		if(fabs(duty_per) > 100.0f)return;
		_target_duty_per = duty_per;
		_mode = drive_mode::INPUT_DUTY;
	}
	//RPM入力(PID)
	virtual void set_RPM(int32_t rpm){
		if(static_cast<uint32_t>(abs(rpm)) > _limit_rpm)return;
		_target_rpm = rpm;
		_mode = drive_mode::INPUT_RPM;
	}
	//目標角入力
	virtual void set_rad(int32_t max_rpm, float rad){
		if(static_cast<uint32_t>(abs(max_rpm)) > _limit_rpm)return;
		_target_rpm = max_rpm;
		_target_rad = rad;
		_mode = drive_mode::INPUT_ANGLE;
	}

	//ゲイン設定
	virtual void set_PID(float p_gain, float i_gain, float d_gain) = 0;
	//回転数上限設定(目標にのみ有効)
	virtual void set_limitRPM(uint32_t limit){_limit_rpm = limit;}

	//回転数取得
	inline virtual int32_t get_RPM() const final{return _read_rpm;}
	//角度取得
	inline virtual float get_rad() const final{return _read_rad;}
};



/*
 * タイヤのクラス
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
