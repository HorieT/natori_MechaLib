/*
 * 2019/07/12 Horie
 */
#pragma once

#include <type_traits>
#include <tuple>
#include <utility>


namespace Mecha{
/*
 * 可変長テンプレート　型比較クラス
 */
namespace{
using namespace std;

template<size_t Index, typename... Args>
class checkSameType {
public:
	static constexpr bool check(){
		if (!is_same<decltype(std::get<Index>((declval<tuple<Args...>>()))), decltype(std::get<Index - 1>((declval<tuple<Args...>>())))>::value)return false;
		return checkSameType<Index - 1, Args...>::check();
	}
};
template<typename... Args>
class checkSameType <0, Args...> {
public:
	static constexpr bool check() {
		return true;
	}
};
}
//可変長テンプレート　型比較関数
template<typename... Args>
constexpr bool check_same_type() {
	return checkSameType<(sizeof...(Args)) - 1, Args...>::check();
}



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
