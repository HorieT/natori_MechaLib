#pragma once

#include "MechaLib/Base/kdtree.hpp"
#include "MechaLib/Base/calculation.hpp"
#include <Eigen/Core>

namespace Mecha{

class routeLine{
public:
	kdtree<coordinate<float>, Eigen::Vector2f, 2> route;
	float length_mm;
};
}
