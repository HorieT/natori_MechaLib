/*
 * 2019/06/24 Horie
 * 加速度制御は手動モードで適用されない
 */
#pragma once

#include "MechaLib/MechaLib_HAL_global.hpp"
#include "MechaLib/chassis/chassis.hpp"
#include "MechaLib/motor/motor.hpp"
#include "MechaLib/posEstimation.hpp"
#include "Eigen/Core"

namespace Mecha{
/*
 * メカナムホイールクラス
 * ４輪・長方形接地前提
 */
class mechanum : public chassis{
private:

	std::array<DriveWheel*, 4> _wheel;//各ホイール
	const coordinate<float> _wheel_position;//ホイール位置
	const float _side_length;
	const bool _wheel_polarity;


	/*
	 * ホイール入力
	 */
	virtual void input_mps(Eigen::Vector2f vector, float rotation) override final{
		std::array<float, 4> input;
		float rot_component = _side_length * rotation;
		if(_wheel_polarity){
			input[0] = -vector.x() + vector.y() + rot_component;
			input[1] = -vector.x() - vector.y() + rot_component;
			input[2] =  vector.x() - vector.y() + rot_component;
			input[3] =  vector.x() + vector.y() + rot_component;
		}else{
			input[0] =  vector.x() - vector.y() - rot_component;
			input[1] =  vector.x() + vector.y() - rot_component;
			input[2] = -vector.x() + vector.y() - rot_component;
			input[3] = -vector.x() - vector.y() - rot_component;
		}
		{
			uint8_t i = 0;
			for(auto& w : _wheel){
				w->set_mps(input[i]);
				++i;
			}
		}
	}
protected:


public:

	/*
	 * コンストラクタ
	　*/
	mechanum(const std::array<DriveWheel*, 4>& wheel, const coordinate<float>& first_wheel_point, SelfPos* my_position, uint32_t period_ms, bool polarity):
		chassis(my_position, period_ms), _wheel(wheel), _wheel_position(first_wheel_point), _side_length((first_wheel_point.x + first_wheel_point.y) * 0.001f), _wheel_polarity(polarity){
	}
	/*
	 * デストラクタ
	 */
	virtual ~mechanum(){_scheduler.erase();}
};
}
