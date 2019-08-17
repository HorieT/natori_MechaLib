/*
 * 2019/07/02 Horie
 */
#pragma once

#include "MechaLib/Base/calculation.hpp"
#include "MechaLib/chassis/moveRouteline.hpp"
#include "MechaLib/sys_timer.hpp"
#include "MechaLib/posEstimation.hpp"
#include <Eigen/Geometry>


namespace Mecha{

/*
 * ライン追従のためのラインの構造
 */
struct move_line{
private:
	struct point_data{
		coordinate<float> point;
		Eigen::Vector2f tangent_vector;
		float length;
	};
public:
	float all_length;
	float num_point;
	std::vector<point_data> point_croud;
};

/*
 * 足回り基底クラス
 * 全方向移動マシンにのみ適用
 */
class chassis{
protected:
	//走行モード
	enum class move_mode : uint8_t{
		EMERGENCY = 0,
		MANUAL,
		SET_GOAL,
		SET_LINE,
	};
	move_mode _mode = move_mode::EMERGENCY;

	//自己位置
	SelfPos* const _my_position;

	//周期制御
	timeScheduler<void> _scheduler;

	//走行目標系
	routeLine* _target_line = nullptr;
	coordinate<float> _target_position_mm{0.0f, 0.0f, 0.0f};
	Eigen::Vector2f _target_vel_vec_mps{0.0f, 0.0f};// m/s
	float _target_vel_rot_rps = 0.0f;// rad/s
	//(位置制御のみ)許容誤差
	coordinate<float> _tolerance{0.0f, 0.0f, 0.0f};
	uint32_t _in_tolerance_time = 100;
	uint32_t _in_tolerance_count = 0;
	//ライン外れセーフティー距離
	float _safe_distance_line_mm = 1500.0f;

	//上限値
	float _limit_vel_vec_mps = 0.0f;
	float _limit_vel_rot_rps = 0.0f;

	//加速度(絶対値)
	float _acc_vec_mps2 = INFINITY;
	//float _acc_rot_rps2 = INFINITY;

	//加速カウンタ
	uint32_t _count_acc_vec = 0;
	uint32_t _count_acc_vec_neg = 0;
	//uint32_t _count_acc_rot = 0;

	//PID制御類
	PID<float> _pid_x_direction;
	PID<float> _pid_y_direction;
	PID<float> _pid_rotaition;
	PID<float> _pid_line;

	float rot_rock = 0.0f;//角度保持
	std::function<void()> _callback_func = nullptr;//終了呼び出し関数


	virtual void move_set() final{
		//入力用変数
		Eigen::Vector2f input_vec_mps = {0.0, 0.0};
		float input_rot_radps = 0.0;
		//自己位置
		coordinate<float> now_position = _my_position->get_pos();
		[[maybe_unused]] coordinate<float> now_vel = _my_position->get_vel();
		//ベクトル回転
		Eigen::Rotation2Df rotate(-now_position.direction_rad);


		switch(_mode){
		//手動操作(加減速はない)
		case move_mode::MANUAL:
			input_vec_mps = _target_vel_vec_mps;
			input_rot_radps = _target_vel_rot_rps;
			break;

		//目標地点収束
		case move_mode::SET_GOAL:{
			coordinate<float> position_difference = (_target_position_mm - now_position);
			Eigen::Vector2f pid_vec_value(0.0f, 0.0f);

			//収束判定
			if((position_difference.x < _tolerance.x) && (position_difference.y < _tolerance.y) && (position_difference.direction_rad < _tolerance.direction_rad)){
				if(_in_tolerance_count > _in_tolerance_time){
					if(_callback_func != nullptr)_callback_func();
					_mode = move_mode::MANUAL;
					input_vec_mps << 0.0f, 0.0f;
					input_rot_radps = 0.0f;
					break;
				}
				_in_tolerance_count += _scheduler.get_period();
			}else{
				_in_tolerance_count = 0;
			}

			_pid_x_direction.control(position_difference.x / 1000.0f, _scheduler.get_period());
			_pid_y_direction.control(position_difference.y / 1000.0f, _scheduler.get_period());
			_pid_rotaition.control(position_difference.direction_rad, _scheduler.get_period());

			pid_vec_value << _pid_x_direction.get(), _pid_y_direction.get();
			if(pid_vec_value.norm() > _limit_vel_vec_mps){
				//頭打ち
				pid_vec_value.normalize();
				pid_vec_value *= _limit_vel_vec_mps;
				_pid_x_direction.set(pid_vec_value.x());
				_pid_y_direction.set(pid_vec_value.y());
			}
			input_vec_mps = rotate * pid_vec_value;

			//回転はPIDクラス内で頭打ち
			input_rot_radps = _pid_rotaition.get();

			break;
		}
		//線入力
		case move_mode::SET_LINE:
		{
			//探索
			float distance_norm;
			size_t index;
			auto near_point = _target_line->route.NN_search(now_position, distance_norm, index);

			/*終了割り込み実装予定*/
			//ライン外れセーフティー
			if(_safe_distance_line_mm > distance_norm){
				//マニュアルモード停止
				input_vec_mps << 0.0, 0.0;
				input_rot_radps = 0.0;
				_mode = move_mode::MANUAL;
			}else{
				//ライン収束PID
				_pid_line.control(distance_norm, _scheduler.get_period());
				//合成
				Eigen::Vector2f catch_vec;
				near_point.get_vector(catch_vec);
				input_vec_mps = _pid_line.get() * catch_vec  + _target_line->tangent_vector[index];

				//加減速判定
				if(std::isfinite(_acc_vec_mps2)){//有限
					input_vec_mps.normalize();
					//加減速ブロック
					if((_target_line->all_length_mm - _target_line->length_mm[index]) < static_cast<float>(M_PI) * powf(_limit_vel_vec_mps, 2.0f) / (4.0f * _acc_vec_mps2)){
						//減速
						input_vec_mps *= _limit_vel_vec_mps * (1.0f + cosf(2.0f * _acc_vec_mps2 * static_cast<float>(_count_acc_vec) * 0.001f / _limit_vel_vec_mps)) / 2.0f;
						_count_acc_vec_neg += _scheduler.get_period();
					}else if(static_cast<float>(_count_acc_vec) < static_cast<float>(M_PI) * _limit_vel_vec_mps * 1000.0f / (2.0f * _acc_vec_mps2)){
						//加速
						input_vec_mps *= _limit_vel_vec_mps * (1.0f - cosf(2.0f * _acc_vec_mps2 * static_cast<float>(_count_acc_vec) * 0.001f / _limit_vel_vec_mps)) / 2.0f;
						_count_acc_vec += _scheduler.get_period();
					}else{
						//最高速
						input_vec_mps *= _limit_vel_vec_mps;
					}
				}else{//無限
					if(input_vec_mps.norm() > _limit_vel_vec_mps)input_vec_mps = input_vec_mps.normalized() * _limit_vel_vec_mps;
				}

				float rot_difference = near_point.direction_rad - now_position.direction_rad;
				_pid_rotaition.control(rot_difference, _scheduler.get_period());
				input_rot_radps = _pid_rotaition.get();
			}
		}
		//停止
		case move_mode::EMERGENCY:
			input_vec_mps <<0.0, 0.0;
			input_rot_radps = 0.0;
			break;
		default:
			break;
		}

		//入力部
		input_mps(input_vec_mps, input_rot_radps);
	}

	virtual void input_mps(Eigen::Vector2f vector, float rotation) = 0;
public:
	/*
	 * コンストラクタ
	 */
	chassis(SelfPos* my_position, uint32_t period_ms) :
		_my_position(my_position), _scheduler([this]{move_set();}, period_ms),
		_pid_x_direction(), _pid_y_direction(), _pid_rotaition(), _pid_line(){}
	/*
	 * デストラクタ
	 */
	virtual ~chassis(){}

	/*
	 * 初期化処理
	 */
	void init(){_scheduler.set();}

	/*
	 * 加減速設定
	 */
	inline virtual bool set_acc(float vector_mps2) final{
		if(_mode == move_mode::SET_LINE)return false;
		_acc_vec_mps2 = fabsf(vector_mps2);
		//_acc_rot_rps2 = fabsf(rotation_rps2);
		return true;
	}

	/*
	 * pidゲインセット関数
	 */
	inline virtual bool set_pid_vector(float p_gain, float i_gain, float d_gain) final{
		if(_mode == move_mode::SET_GOAL)return false;
		_pid_x_direction.chage_gaine(p_gain, i_gain, d_gain);
		_pid_y_direction.chage_gaine(p_gain, i_gain, d_gain);
		return true;
	}
	inline virtual bool set_pid_rotation(float p_gain, float i_gain, float d_gain) final{
		if(_mode == move_mode::SET_LINE || _mode == move_mode::SET_GOAL)return false;
		_pid_rotaition.chage_gaine(p_gain, i_gain, d_gain);
		return true;
	}
	inline virtual bool set_pid_line(float p_gain, float i_gain, float d_gain) final{
		if(_mode == move_mode::SET_LINE)return false;
		_pid_line.chage_gaine(p_gain, i_gain, d_gain);
		return true;
	}
	/*
	 * ベクトル入力
	 */
	inline virtual void set_velVec(const Eigen::Vector2f& vector_mps, float rotation_rps) final{
		if(_mode == move_mode::EMERGENCY)return;
		_target_vel_vec_mps = vector_mps;
		_target_vel_rot_rps = rotation_rps;
		_mode = move_mode::MANUAL;
	}

	inline virtual void set_velVec(float velocity_mps, float direction_rad, float rotation_rps) final{
		if(_mode == move_mode::EMERGENCY)return;
		_target_vel_vec_mps << velocity_mps * cosf(direction_rad), velocity_mps * sinf(direction_rad);
		_target_vel_rot_rps = rotation_rps;
		_mode = move_mode::MANUAL;
	}
	/*
	 * 収束目標座標入力(開始時速度0と定義)
	 */
	void virtual set_goal(const coordinate<float>& position, float velocity_mps, float rotation_rps, const coordinate<float>& tolerance, std::function<void()>&& finish_callback = nullptr) final{
		if(_mode == move_mode::EMERGENCY)return;
		_target_vel_vec_mps << 0.0, 0.0;
		_target_vel_rot_rps = 0.0;


		_limit_vel_vec_mps = velocity_mps;
		_pid_x_direction.set(0.0f);
		_pid_y_direction.set(0.0f);
		_limit_vel_rot_rps = rotation_rps;
		_pid_rotaition.chage_limit(_limit_vel_rot_rps);

		_target_position_mm = position;

		_tolerance = tolerance;
		_callback_func = std::move(finish_callback);
		_mode = move_mode::SET_GOAL;
	}
	/*
	 * 経路入力
	 */
	void set_path(routeLine* line, float velocity_mps, float rotation_rps, std::function<void()>&& finish_callback = nullptr, float rad = INFINITY){
		if(_mode == move_mode::EMERGENCY)return;
		_target_vel_vec_mps << 0.0, 0.0;
		_target_vel_rot_rps = 0.0;
		_target_line = line;

		//加減速設定
		//移動ベクトル
		if(std::isfinite(_acc_vec_mps2)){//有限
			_count_acc_vec = 0U;
			_count_acc_vec_neg = 0;
			//加減速距離による最大値の設定
			_limit_vel_vec_mps =
					(line->all_length_mm / 1000.0f > (static_cast<float>(M_PI) * powf(velocity_mps, 2.0f) / (2.0f * _acc_vec_mps2))) ?
							velocity_mps :
							sqrtf(2.0f * _acc_vec_mps2 * line->all_length_mm / 1000.0f / static_cast<float>(M_PI));
		}else{//無限
			_limit_vel_vec_mps = velocity_mps;
		}

		/**/
		_limit_vel_rot_rps = rotation_rps;

		_pid_line.set(0.0f);
		_pid_rotaition.set(0.0f);
		_pid_rotaition.chage_limit(_limit_vel_rot_rps);


		_callback_func = finish_callback;
		rot_rock = (rad > M_PI || rad < -M_PI) ? _my_position->get_pos().direction_rad : rad;
		_mode = move_mode::SET_LINE;
	}
	/*
	 * 緊急停止
	 */
	virtual void emergency_stop(){
		_mode = move_mode::EMERGENCY;
		_target_vel_vec_mps << 0.0f, 0.0f;
		_target_vel_rot_rps = 0.0f;
	}
	virtual void release_emergency(){
		_target_vel_vec_mps << 0.0f, 0.0f;
		_target_vel_rot_rps = 0.0f;
		_mode = move_mode::MANUAL;
	}
};
}
