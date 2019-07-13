/*
 * 2019/07/12 Horie
 */
#pragma once

#include <type_traits>
#include <tuple>
#include <utility>


namespace Mecha{
/*
 * �ϒ��e���v���[�g�@�^��r�N���X
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
//�ϒ��e���v���[�g�@�^��r�֐�
template<typename... Args>
constexpr bool check_same_type() {
	return checkSameType<(sizeof...(Args)) - 1, Args...>::check();
}
}
