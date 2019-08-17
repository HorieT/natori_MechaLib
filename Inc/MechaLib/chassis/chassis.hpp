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
 * ���C���Ǐ]�̂��߂̃��C���̍\��
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
 * �������N���X
 * �S�����ړ��}�V���ɂ̂ݓK�p
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
	timeScheduler<void> _scheduler;

	//���s�ڕW�n
	routeLine* _target_line = nullptr;
	coordinate<float> _target_position_mm{0.0f, 0.0f, 0.0f};
	Eigen::Vector2f _target_vel_vec_mps{0.0f, 0.0f};// m/s
	float _target_vel_rot_rps = 0.0f;// rad/s
	//(�ʒu����̂�)���e�덷
	coordinate<float> _tolerance{0.0f, 0.0f, 0.0f};
	uint32_t _in_tolerance_time = 100;
	uint32_t _in_tolerance_count = 0;
	//���C���O��Z�[�t�e�B�[����
	float _safe_distance_line_mm = 1500.0f;

	//����l
	float _limit_vel_vec_mps = 0.0f;
	float _limit_vel_rot_rps = 0.0f;

	//�����x(��Βl)
	float _acc_vec_mps2 = INFINITY;
	//float _acc_rot_rps2 = INFINITY;

	//�����J�E���^
	uint32_t _count_acc_vec = 0;
	uint32_t _count_acc_vec_neg = 0;
	//uint32_t _count_acc_rot = 0;

	//PID�����
	PID<float> _pid_x_direction;
	PID<float> _pid_y_direction;
	PID<float> _pid_rotaition;
	PID<float> _pid_line;

	float rot_rock = 0.0f;//�p�x�ێ�
	std::function<void()> _callback_func = nullptr;//�I���Ăяo���֐�


	virtual void move_set() final{
		//���͗p�ϐ�
		Eigen::Vector2f input_vec_mps = {0.0, 0.0};
		float input_rot_radps = 0.0;
		//���Ȉʒu
		coordinate<float> now_position = _my_position->get_pos();
		[[maybe_unused]] coordinate<float> now_vel = _my_position->get_vel();
		//�x�N�g����]
		Eigen::Rotation2Df rotate(-now_position.direction_rad);


		switch(_mode){
		//�蓮����(�������͂Ȃ�)
		case move_mode::MANUAL:
			input_vec_mps = _target_vel_vec_mps;
			input_rot_radps = _target_vel_rot_rps;
			break;

		//�ڕW�n�_����
		case move_mode::SET_GOAL:{
			coordinate<float> position_difference = (_target_position_mm - now_position);
			Eigen::Vector2f pid_vec_value(0.0f, 0.0f);

			//��������
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
				//���ł�
				pid_vec_value.normalize();
				pid_vec_value *= _limit_vel_vec_mps;
				_pid_x_direction.set(pid_vec_value.x());
				_pid_y_direction.set(pid_vec_value.y());
			}
			input_vec_mps = rotate * pid_vec_value;

			//��]��PID�N���X���œ��ł�
			input_rot_radps = _pid_rotaition.get();

			break;
		}
		//������
		case move_mode::SET_LINE:
		{
			//�T��
			float distance_norm;
			size_t index;
			auto near_point = _target_line->route.NN_search(now_position, distance_norm, index);

			/*�I�����荞�ݎ����\��*/
			//���C���O��Z�[�t�e�B�[
			if(_safe_distance_line_mm > distance_norm){
				//�}�j���A�����[�h��~
				input_vec_mps << 0.0, 0.0;
				input_rot_radps = 0.0;
				_mode = move_mode::MANUAL;
			}else{
				//���C������PID
				_pid_line.control(distance_norm, _scheduler.get_period());
				//����
				Eigen::Vector2f catch_vec;
				near_point.get_vector(catch_vec);
				input_vec_mps = _pid_line.get() * catch_vec  + _target_line->tangent_vector[index];

				//����������
				if(std::isfinite(_acc_vec_mps2)){//�L��
					input_vec_mps.normalize();
					//�������u���b�N
					if((_target_line->all_length_mm - _target_line->length_mm[index]) < static_cast<float>(M_PI) * powf(_limit_vel_vec_mps, 2.0f) / (4.0f * _acc_vec_mps2)){
						//����
						input_vec_mps *= _limit_vel_vec_mps * (1.0f + cosf(2.0f * _acc_vec_mps2 * static_cast<float>(_count_acc_vec) * 0.001f / _limit_vel_vec_mps)) / 2.0f;
						_count_acc_vec_neg += _scheduler.get_period();
					}else if(static_cast<float>(_count_acc_vec) < static_cast<float>(M_PI) * _limit_vel_vec_mps * 1000.0f / (2.0f * _acc_vec_mps2)){
						//����
						input_vec_mps *= _limit_vel_vec_mps * (1.0f - cosf(2.0f * _acc_vec_mps2 * static_cast<float>(_count_acc_vec) * 0.001f / _limit_vel_vec_mps)) / 2.0f;
						_count_acc_vec += _scheduler.get_period();
					}else{
						//�ō���
						input_vec_mps *= _limit_vel_vec_mps;
					}
				}else{//����
					if(input_vec_mps.norm() > _limit_vel_vec_mps)input_vec_mps = input_vec_mps.normalized() * _limit_vel_vec_mps;
				}

				float rot_difference = near_point.direction_rad - now_position.direction_rad;
				_pid_rotaition.control(rot_difference, _scheduler.get_period());
				input_rot_radps = _pid_rotaition.get();
			}
		}
		//��~
		case move_mode::EMERGENCY:
			input_vec_mps <<0.0, 0.0;
			input_rot_radps = 0.0;
			break;
		default:
			break;
		}

		//���͕�
		input_mps(input_vec_mps, input_rot_radps);
	}

	virtual void input_mps(Eigen::Vector2f vector, float rotation) = 0;
public:
	/*
	 * �R���X�g���N�^
	 */
	chassis(SelfPos* my_position, uint32_t period_ms) :
		_my_position(my_position), _scheduler([this]{move_set();}, period_ms),
		_pid_x_direction(), _pid_y_direction(), _pid_rotaition(), _pid_line(){}
	/*
	 * �f�X�g���N�^
	 */
	virtual ~chassis(){}

	/*
	 * ����������
	 */
	void init(){_scheduler.set();}

	/*
	 * �������ݒ�
	 */
	inline virtual bool set_acc(float vector_mps2) final{
		if(_mode == move_mode::SET_LINE)return false;
		_acc_vec_mps2 = fabsf(vector_mps2);
		//_acc_rot_rps2 = fabsf(rotation_rps2);
		return true;
	}

	/*
	 * pid�Q�C���Z�b�g�֐�
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
	 * �x�N�g������
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
	 * �����ڕW���W����(�J�n�����x0�ƒ�`)
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
	 * �o�H����
	 */
	void set_path(routeLine* line, float velocity_mps, float rotation_rps, std::function<void()>&& finish_callback = nullptr, float rad = INFINITY){
		if(_mode == move_mode::EMERGENCY)return;
		_target_vel_vec_mps << 0.0, 0.0;
		_target_vel_rot_rps = 0.0;
		_target_line = line;

		//�������ݒ�
		//�ړ��x�N�g��
		if(std::isfinite(_acc_vec_mps2)){//�L��
			_count_acc_vec = 0U;
			_count_acc_vec_neg = 0;
			//�����������ɂ��ő�l�̐ݒ�
			_limit_vel_vec_mps =
					(line->all_length_mm / 1000.0f > (static_cast<float>(M_PI) * powf(velocity_mps, 2.0f) / (2.0f * _acc_vec_mps2))) ?
							velocity_mps :
							sqrtf(2.0f * _acc_vec_mps2 * line->all_length_mm / 1000.0f / static_cast<float>(M_PI));
		}else{//����
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
	 * �ً}��~
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
