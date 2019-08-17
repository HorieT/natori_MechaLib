#pragma once

#include "MechaLib/MechaLib_HAL_global.hpp"
#include "MechaLib/chassis/chassis.hpp"
#include "MechaLib/motor/motor.hpp"



namespace Mecha{

/*
 * �I���j�z�C�[���N���X
 * �ԗ֐�T�̑����      255�ւ܂ŃT�|�[�g!!!!!!!!!  �^�C����255����Ƒ����Ȃ�񂾁I
 * �ԗւ͓��p�z�u����Ă���O��
 */
template<uint8_t T>
class omni : private chassis{
	static_assert(T > 1U, "Are you serious?");
private:
	std::array<DriveWheel*, T> _wheel;//�e�z�C�[��
	const coordinate<float> _wheel_position;//�z�C�[���ʒu
	const float _wheel_length;
	const bool _wheel_polarity;
	const std::array<std::array<float, 2>, T> _coefficient;


	/*
	 * �z�C�[������
	 */
	virtual void input_mps(Eigen::Vector2f vector, float rotation) override final{
		std::array<float, T> input;
		float rot_component = _wheel_length * rotation;
		//��r�񐔑����邯�ǎO�����Z�q�̕������������H
		if(_wheel_polarity){
			uint8_t i = 0;
			for(auto in : input){
				in = vector.x() * _coefficient[i][0] + vector.y() * _coefficient[i][1] + rot_component;
				++i;
			}

		}else{
			uint8_t i = 0;
			for(auto in : input){
				in = -vector.x() * _coefficient[i][0] - vector.y() * _coefficient[i][1] - rot_component;
				++i;
			}

		}
		{
			uint8_t i = 0;
			for(auto& w : _wheel){
				w->set_mps(input[i]);
				++i;
			}
		}
		}

public:
	/*
	 * �R���X�g���N�^
	�@*/
	omni(const std::array<DriveWheel*, T>& wheel, const coordinate<float>& first_wheel_point, SelfPos* my_position, uint32_t period_ms, bool polarity):
		chassis(my_position, period_ms), _wheel(wheel), _wheel_position(first_wheel_point), _wheel_length(first_wheel_point.norm() * 0.001f), _wheel_polarity(polarity){
		{
			uint8_t i = 0;
			for(auto& c : const_cast<std::array<std::array<float, 2>, T>&>(_coefficient)){
				c[0] = -std::sin(_wheel_position.angle() + static_cast<float>(M_PI) * 2.0f * static_cast<float>(i) / static_cast<float>(T));
				c[1] = std::cos(_wheel_position.angle() + static_cast<float>(M_PI) * 2.0f * static_cast<float>(i)/ static_cast<float>(T));
				++i;
			}
		}
	}
	/*
	 * �f�X�g���N�^
	 */
	virtual ~omni(){_scheduler.erase();}
};
}
