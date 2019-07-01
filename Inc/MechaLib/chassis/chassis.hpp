/*
 * 2019/06/ Horie
 */
#pragma once

#include "MechaLib/calculation.hpp"
#include "MechaLib/sys_timer.hpp"
#include "MechaLib/posEstimation.hpp"


namespace Mecha{

/*
 * �������N���X
 */
class chassis{
protected:
	//���s���[�h
	enum class move_mode : uint8_t{
		EMERGENCY = 0,
		MANUAL,
		SET_GOAL,
		SET_LINE,
	};
	move_mode _mode = move_mode::EMERGENCY;

	//���Ȉʒu
	SelfPos* const _my_position;

	//��������
	timeScheduler<chassis*> _scheduler;

	//���s�ڕW�n
	std::vector<std::pair<Eigen::Vector2f, coordinate<float>>>* _target_line = nullptr;
	coordinate<float> _target_position_mm{0.0f, 0.0f, 0.0f};
	Eigen::Vector2f _target_vel_vec_mps{0.0f, 0.0f};// m/s
	float _target_vel_rot_rps = 0.0f;// rad/s
	//(�ʒu����̂�)���e�덷
	coordinate<float> _tolerance{0.0f, 0.0f, 0.0f};

	//����l
	float _limit_vel_vec_mps = 0.0f;
	float _limit_vel_rot_rps = 0.0f;

	//�����x(��Βl)
	float _acc_vec_mps2 = INFINITY;
	float _acc_rot_rps2 = INFINITY;

	//�����J�E���^
	uint32_t _count_acc_vec = 0;
	uint32_t _count_acc_rot = 0;

	//PID�����
	PID<float> _pid_x_direction;
	PID<float> _pid_y_direction;
	PID<float> _pid_rotaition;
	PID<float> _pid_line;

	float rot_rock = 0.0f;//�p�x�ێ�
	void (*_callback_func)(void) = nullptr;//�I���Ăяo���֐�

	static void scheduler_fanc(chassis* me){
		me->move_set();
	}

	virtual void move_set() final{
		//���͗p�ϐ�
		/*Eigen::Vector2d input_vec_mps = {0.0, 0.0};
		double input_rot_radps = 0.0;
		//���Ȉʒu
		coordinate<float> now_position = _my_position->get_pos();




		[[maybe_unused]] coordinate<float> now_vel = _my_position->get_vel();
		coordinate<float> def_pos;
		Eigen::Vector2f swap_vec(0.0f, 0.0f);
		Eigen::Matrix2f rot;
		rot << cosf(-now_position.direction_rad), -sinf(-now_position.direction_rad),
				sinf(-now_position.direction_rad), cosf(-now_position.direction_rad);


		switch(_mode){
		//�蓮����(�������͂Ȃ�)
		case move_mode::MANUAL:
			input_vec_mps = _target_vel_vec_mps;
			input_rot_radps = _target_vel_rot_rps;
			break;
		//�ڕW�n�_�ړ�
		case move_mode::SET_GOAL:{
			Eigen::Vector2f position_difference = (_target_position_mm - now_position) / 1000.0f;

			if(std::isfinite(_acc_vec_mps2)){//�L��
				//�������u���b�N

			}else{//����
			}


			break;
		}


			def_pos = ((target_goal - now_pos) / 1000.0);
			def_pos.get_vector(swap_vec);

			if(def_pos.norm() > limit_vel_vec){
				Eigen::Vector2f box = (rot * (limit_vel_vec * swap_vec.normalized()));
				input_vec_mps = box.cast<double>();
			}else if(def_pos.norm() < limit_vel_vec / 100.0){
				swap_vec *= 10.0;
				input_vec_��mps = (rot * swap_vec).cast<double>();
			}else{
				input_vec_mps = (rot * swap_vec).cast<double>();
			}

			input_rot_radps = def_pos.direction_rad;
			input_rot_radps = (input_rot_radps > 0) ? limit_vel_rot : -limit_vel_rot;

			if(def_pos.norm() < 10.0 && def_pos.direction_rad < M_PI/20.0){
				mode = MANUAL;
				target_vel_vec << 0.0, 0.0;
				target_vel_rot = 0.0;
				if(returnFunc != nullptr)returnFunc();
			}
			break;


		//������
		case move_mode::SET_LINE:
		{
			Eigen::Vector2f catch_vec(0.0f, 0.0f), vec_buff(0.0f, 0.0f);
			coordinate<float> def_buff;
			float rad_buff = 0.0f;
			float min_def_norm = 1000000.0f;
			[[maybe_unused]]uint32_t count = 0, get_count = 0;

			for(auto& l : *line){
				def_buff = l.first - now_position;


				if(auto def = (def_buff.x * def_buff.x +  def_buff.y * def_buff.y); def < min_def_norm){//norm���ĂȂ񂼁H�H�H�H�H�H�H�H�傫������Ȃ��񂩁H�H�H�H�H�H�H�H

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
				if(!std::isfinite(_acc_vec_mps2)){//����
					input_vec_mps = (_acc_vec_mps2_neg * swap_vec.normalized().cast<double>());
				}else{
					float dist = 0.0f;
					for(auto i = line->begin() + count, e = line->end();i <= e;++i)
						dist += (*i).second.norm();
					if(float sp = (dist * (float)acc_vec);sp < limit_vel_vec){//����
						input_vec_mps = (sp * swap_vec.normalized()).cast<double>();
					}else{//�񌸑�
						if(float tag = sqrtf(powf((float)target_vel_vec.x(), 2)+powf((float)target_vel_vec.y(), 2));(float)limit_vel_vec > tag){//����
							tag += (float)acc_vec / 100.0f;
							target_vel_vec = (double)tag * target_vel_vec.normalized();
							input_vec_mps = (tag * swap_vec.normalized()).cast<double>();
						}else{//�����Ɠ�������
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
			break;
		}
		case move_mode::EMERGENCY:
			input_vec_mps <<0.0, 0.0;
			input_rot_radps = 0.0;
			break;
		default:
			break;
		}

		//���͕�
		input_mps(input_vec_mps, input_rot_radps);*/
	}

	virtual void input_mps(Eigen::Vector2f vector, float rotation) = 0;
public:
	/*
	 * �R���X�g���N�^
	 */
	chassis(SelfPos* my_position, uint32_t period_ms) :
		_my_position(my_position), _scheduler(scheduler_fanc, period_ms),
		_pid_x_direction(), _pid_y_direction(), _pid_rotaition(), _pid_line(){}
	/*
	 * �f�X�g���N�^
	 */
	virtual ~chassis(){}

	/*
	 * ����������
	 */
	void init(){_scheduler.set(this);}

	/*
	 * �������ݒ�
	 */
	inline virtual bool set_acc(float vector_mps2, float rotation_rps2) final{
		if(_mode != move_mode::MANUAL || _mode != move_mode::EMERGENCY)return false;
		_acc_vec_mps2 = fabsf(vector_mps2);
		_acc_rot_rps2 = fabsf(rotation_rps2);
		return true;
	}
	/*
	 * �x�N�g������
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
	 * �ڕW���W����(�J�n���A��~���Ƃ��ɑ��x0�ƒ�`)
	 */
	void virtual set_goal(const coordinate<float>& position, float velocity_mps, float rotation_rps, const coordinate<float>& tolerance, void (*finish_callback)(void) = nullptr) final{
		_target_vel_vec_mps << 0.0, 0.0;
		_target_vel_rot_rps = 0.0;
		//���s����
		coordinate<float> position_distance = position - _my_position->get_pos();

		//�������ݒ�
		//�ړ��x�N�g��
		if(std::isfinite(_acc_vec_mps2)){//�L��
			_count_acc_vec = 0U;
			//�����������ɂ��ő�l�̐ݒ�
			_limit_vel_vec_mps =
					(position_distance.norm() > (static_cast<float>(M_PI) * powf(velocity_mps, 2.0f) / (2.0f * _acc_vec_mps2))) ?
							velocity_mps :
							sqrtf(2.0f * _acc_vec_mps2 * position_distance.norm() / static_cast<float>(M_PI));

		}else{//����

		}
		//��]�x�N�g��
		if(std::isfinite(_acc_rot_rps2)){//�L��
			_count_acc_rot = 0U;
			//�����������ɂ��ő�l�̐ݒ�
			_limit_vel_rot_rps =
					(position_distance.direction_rad > (static_cast<float>(M_PI) * powf(_limit_vel_rot_rps, 2.0f) / (2.0f * _acc_rot_rps2))) ?
							rotation_rps :
							sqrtf(2.0f * _acc_rot_rps2 * position_distance.direction_rad / static_cast<float>(M_PI));

		}else{//����

		}


		//_target_position_mm = position;

		_tolerance = tolerance;
		_callback_func = finish_callback;
		_mode = move_mode::SET_GOAL;
	}
	/*
	 * �o�H����
	 */
	void set_path(std::vector<std::pair<Eigen::Vector2f, coordinate<float>>>* l, float velocity_mps, float rotation_rps, void (*finish_callback)(void) = nullptr, double rad = INFINITY){
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
	 * �ً}��~
	 */
	virtual void emergency_stop(){
		_mode = move_mode::EMERGENCY;
		_target_vel_vec_mps << 0.0f, 0.0f;
		_target_vel_rot_rps = 0.0f;
	}
};
}
