/*
 * 2019/07/12 Horie
 */
#pragma once

#include <type_traits>
#include <tuple>
#include <utility>


namespace Mecha{

/*
 * ビットカウント関数
 */
template<typename T>
constexpr uint16_t bit_count(T num){
	static_assert(std::is_integral<T>::value, "Variable is not integer type.");

	uint16_t count = 0;
	for(uint16_t i = 0;i < sizeof(T);++i)
		if(num & (1U << i))++count;
	return count;
}

}
