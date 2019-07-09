#pragma once

#include "MechaLib/Base/kdtree.hpp"
#include "MechaLib/Base/calculation.hpp"
#include <Eigen/Core>

namespace Mecha{

/*
 * ツリー、接線ベクトル、距離の要素数はそれぞれ等しくなるようにツールで生成する。
 * */
class routeLine{
public:
	kdtree<coordinate<float>, 2> route;
	std::vector<Eigen::Vector2f> tangent_vector;
	std::vector<float> length_mm;
	float all_length_mm;
};
}
