/*
 * 2019/07/02 Horie
 */
#pragma once

#include "MechaLib/Base/calculation.hpp"
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
	timeScheduler<chassis*> _scheduler;

	//走行目標系
	std::vector<std::pair<Eigen::Vector2f, coordinate<float>>>* _target_line = nullptr;
	coordinate<float> _target_position_mm{0.0f, 0.0f, 0.0f};
	Eigen::Vector2f _target_vel_vec_mps{0.0f, 0.0f};// m/s
	float _target_vel_rot_rps = 0.0f;// rad/s
	//(位置制御のみ)許容誤差
	coordinate<float> _tolerance{0.0f, 0.0f, 0.0f};
	uint32_t _in_tolerance_time = 100;
	uint32_t _in_tolerance_count = 0;

	//上限値
	float _limit_vel_vec_mps = 0.0f;
	float _limit_vel_rot_rps = 0.0f;

	//加速度(絶対値)
	float _acc_vec_mps2 = INFINITY;
	float _acc_rot_rps2 = INFINITY;

	//加速カウンタ
	uint32_t _count_acc_vec = 0;
	uint32_t _count_acc_rot = 0;

	//PID制御類
	PID<float> _pid_x_direction;
	PID<float> _pid_y_direction;
	PID<float> _pid_rotaition;
	PID<float> _pid_line;

	float rot_rock = 0.0f;//角度保持
	void (*_callback_func)(void) = nullptr;//終了呼び出し関数

	static void scheduler_fanc(chassis* me){
		me->move_set();
	}

	virtual void move_set() final{
		//入力用変数
		Eigen::Vector2f input_vec_mps = {0.0, 0.0};
		float input_rot_radps = 0.0;
		//自己位置
		coordinate<float> now_position = _my_position->get_pos();




		[[maybe_unused]] coordinate<float> now_vel = _my_position->get_vel();
		coordinate<float> def_pos;
		Eigen::Vector2f swap_vec(0.0f, 0.0f);

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


			/*
			//進行方向単位ベクトル
			input_vec_mps = rotate * position_difference_vec.normalized();
			if(std::isfinite(_acc_vec_mps2)){//有限
				//加減速ブロック
				if(position_difference.norm() < static_cast<float>(M_PI) * powf(_limit_vel_vec_mps, 2.0f) / (4.0f * _acc_vec_mps2)){
					//減速

				}else if(static_cast<float>(_count_acc_vec) < static_cast<float>(M_PI) * _limit_vel_vec_mps * 1000.0f / (2.0f * _acc_vec_mps2)){
					//加速
					input_vec_mps *= _limit_vel_vec_mps * (1.0f - cosf(2.0f * _acc_vec_mps2 * static_cast<float>(_count_acc_vec) * 0.001f / _limit_vel_vec_mps)) / 2.0f;
				}else{
					//最高速
				}
				_count_acc_vec += _scheduler.get_period();
			}else{//無限

			}

			input_rot_radps = position_difference.direction_rad;
			if(std::isfinite(_acc_rot_rps2)){

			}else{

			}
			*/
			break;
		}
		//線入力
		case move_mode::SET_LINE:
		{
			/*Eigen::Vector2f catch_vec(0.0f, 0.0f), vec_buff(0.0f, 0.0f);
			coordinate<float> def_buff;
			float rad_buff = 0.0f;
			float min_def_norm = 1000000.0f;
			[[maybe_unused]]uint32_t count = 0, get_count = 0;

			for(auto& l : *line){
				def_buff = l.first - now_position;


				if(auto def = (def_buff.x * def_buff.x +  def_buff.y * def_buff.y); def < min_def_norm){//normってなんぞ？？？？？？？？大きさじゃないんか？？？？？？？？

					min_def_norm = def;
					def_buff.get_vector(vec_buff);
					l.second.get_vector(catch_vec);
					rad_buff = l.second.direction_rad;
					get_count = count;

					if((count + 1) == line->size()){
						mode = MANUAL;
						target_vel_vec << 0.0, 0.0;
						target_vel_rot = 0.0;
						if(_returnFunc != nullptr)_returnFunc();
						break;
					}
				}
				++count;
			}
			swap_vec = rot * (catch_vec.normalized() * (float)limit_vel_vec * 1000.0f + vec_buff * 9.5f);
			if(min_def_norm < 1000000.0f){
				if(!std::isfinite(_acc_vec_mps2)){//無限
					input_vec_mps = (_acc_vec_mps2_neg * swap_vec.normalized().cast<double>());
				}else{
					float dist = 0.0f;
					for(auto i = line->begin() + count, e = line->end();i <= e;++i)
						dist += (*i).second.norm();
					if(float sp = (dist * (float)acc_vec);sp < limit_vel_vec){//減速
						input_vec_mps = (sp * swap_vec.normalized()).cast<double>();
					}else{//非減速
						if(float tag = sqrtf(powf((float)target_vel_vec.x(), 2)+powf((float)target_vel_vec.y(), 2));(float)limit_vel_vec > tag){//加速
							tag += (float)acc_vec / 100.0f;
							target_vel_vec = (double)tag * target_vel_vec.normalized();
							input_vec_mps = (tag * swap_vec.normalized()).cast<double>();
						}else{//無限と同じ処理
							input_vec_mps = (limit_vel_vec * swap_vec.normalized().cast<double>());
						}
					}
				}


				input_rot_radps = rad_buff + (rot_rock - now_position.direction_rad) * 3.8;

				if(abs(input_rot_radps) > limit_vel_rot){
					input_rot_radps = (input_rot_radps > 0) ? limit_vel_rot : -limit_vel_rot;
				}
			}else{
				input_rot_radps = 0.0;
				input_vec_mps = Eigen::Vector2d(0.0, 0.0);
			}
			break;*/
		}







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
		_my_position(my_position), _scheduler(scheduler_fanc, period_ms),
		_pid_x_direction(), _pid_y_direction(), _pid_rotaition(), _pid_line(){}
	/*
	 * デストラクタ
	 */
	virtual ~chassis(){}

	/*
	 * 初期化処理
	 */
	void init(){_scheduler.set(this);}

	/*
	 * 加減速設定
	 */
	inline virtual bool set_acc(float vector_mps2, float rotation_rps2) final{
		if(_mode == move_mode::SET_LINE)return false;
		_acc_vec_mps2 = fabsf(vector_mps2);
		_acc_rot_rps2 = fabsf(rotation_rps2);
		return true;
	}
	/*
	 * ベクトル入力
	 */
	inline virtual void set_velVec(const Eigen::Vector2f& vector_mps, float rotation_rps) final{
		_target_vel_vec_mps = vector_mps;
		_target_vel_rot_rps = rotation_rps;
		_mode = move_mode::MANUAL;
	}

	inline virtual void set_velVec(float velocity_mps, float direction_rad, float rotation_rps) final{
		_target_vel_vec_mps << velocity_mps * cosf(direction_rad), velocity_mps * sinf(direction_rad);
		_target_vel_rot_rps = rotation_rps;
		_mode = move_mode::MANUAL;
	}
	/*
	 * 収束目標座標入力(開始時速度0と定義)
	 */
	void virtual set_goal(const coordinate<float>& position, float velocity_mps, float rotation_rps, const coordinate<float>& tolerance, void (*finish_callback)(void) = nullptr) final{
		_target_vel_vec_mps << 0.0, 0.0;
		_target_vel_rot_rps = 0.0;

		/*
		//走行距離
		coordinate<float> position_distance = position - _my_position->get_pos();

		//加減速設定
		//移動ベクトル
		if(std::isfinite(_acc_vec_mps2)){//有限
			_count_acc_vec = 0U;
			//加減速距離による最大値の設定
			_limit_vel_vec_mps =
					(position_distance.norm() > (static_cast<float>(M_PI) * powf(velocity_mps, 2.0f) / (2.0f * _acc_vec_mps2))) ?
							velocity_mps :
							sqrtf(2.0f * _acc_vec_mps2 * position_distance.norm() / static_cast<float>(M_PI));

		}else{//無限
			_limit_vel_vec_mps = velocity_mps;
		}
		//回転ベクトル
		if(std::isfinite(_acc_rot_rps2)){//有限
			_count_acc_rot = 0U;
			//加減速距離による最大値の設定
			_limit_vel_rot_rps =
					(position_distance.direction_rad > (static_cast<float>(M_PI) * powf(_limit_vel_rot_rps, 2.0f) / (2.0f * _acc_rot_rps2))) ?
							rotation_rps :
							sqrtf(2.0f * _acc_rot_rps2 * position_distance.direction_rad / static_cast<float>(M_PI));

		}else{//無限
			_limit_vel_rot_rps = rotation_rps;
		}
*/

		_limit_vel_vec_mps = velocity_mps;
		_pid_x_direction.set(0.0f);
		_pid_y_direction.set(0.0f);
		_limit_vel_rot_rps = rotation_rps;
		_pid_rotaition.chage_limit(_limit_vel_rot_rps);

		_target_position_mm = position;

		_tolerance = tolerance;
		_callback_func = finish_callback;
		_mode = move_mode::SET_GOAL;
	}
	/*
	 * 経路入力
	 */
	void set_path(std::vector<std::pair<Eigen::Vector2f, coordinate<float>>>* l, float velocity_mps, float rotation_rps, void (*finish_callback)(void) = nullptr, float rad = INFINITY){
		_target_vel_vec_mps << 0.0, 0.0;
		_target_vel_rot_rps = 0.0;
		_target_line = l;
		_limit_vel_vec_mps = velocity_mps;
		_limit_vel_rot_rps = rotation_rps;
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
};
}
