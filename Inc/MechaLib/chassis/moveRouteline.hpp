#pragma once

#include "MechaLib/Base/kdtree.hpp"
#include "MechaLib/Base/calculation.hpp"
#include <Eigen/Core>

namespace Mecha{

/*
 * �c���[�A�ڐ��x�N�g���A�����̗v�f���͂��ꂼ�ꓙ�����Ȃ�悤�Ƀc�[���Ő�������B
 * */
class routeLine{
public:
	kdtree<coordinate<float>, 2> route;
	std::vector<Eigen::Vector2f> tangent_vector;
	std::vector<float> length_mm;
	float all_length_mm;
};
}
