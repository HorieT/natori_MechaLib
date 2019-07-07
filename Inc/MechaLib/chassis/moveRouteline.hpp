#pragma once

#include "MechaLib/Base/kdtree.hpp"
#include "MechaLib/Base/calculation.hpp"
#include <Eigen/Core>

namespace Mecha{

class routeLine{
public:
	kdtree<coordinate<float>, std::pair<Eigen::Vector2f, float>, 2> route;
	float length_mm;
};
}
