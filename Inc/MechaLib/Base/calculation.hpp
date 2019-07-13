/*
 * 2019/07/02 Horie
 * 計算系機能の実装
 * Eigenライブラリを使用する
 */
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <array>
#include <iterator>

#include <Eigen/Core>
/*
 * 数学定数がなぜか使えないときのため
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Mecha{

/*
 * 座標構造体
 */
template<typename T>
struct coordinate{
	T x;
	T y;
	T direction_rad;//方向角

	/*
	 * コンストラクタ
	 * 第一引数:X座標,第二引数:Y座標,第三引数:方向角
	 */
	coordinate(T X = 0.0, T Y = 0.0, T Rad = 0.0) : x(X), y(Y), direction_rad(Rad){}
	//明示的なコピーコンストラクタ(必須)
	coordinate(const coordinate<T>& copy) : x(copy.x), y(copy.y), direction_rad(copy.direction_rad){}
	inline coordinate& operator=(const coordinate<T>& obj){
		x = obj.x;
		y = obj.y;
		direction_rad = obj.direction_rad;
		return *this;
	}

	/*
	 * ベクトル取得関数
	 * 第一引数:取得ベクトル
	 */
	template<class V>
	inline void get_vector(V& vector) const{
		vector.x() = x;
		vector.y() = y;
	}
	/*
	 * 原点からの距離算出関数
	 * 戻り値:距離
	 */
	inline constexpr auto norm() const -> decltype(x + 0.0f) {return sqrtf(powf(x, 2.0f) + powf(y, 2.0f));}//自動昇格でサイズ抑え

	/*
	 * 位置ベクトルの傾き算出関数
	 * 戻り値:角度
	 */
	inline constexpr auto angle() const ->decltype(x + 0.0f) {return atan2f(y, x);}

	/*
	 * 算術演算子オーバーロード
	 * EigenのVectorとも一部互換
	 */
	inline coordinate<T>& operator+=(const coordinate<T>& r_operand){
		x += r_operand.x;
		y += r_operand.y;
		direction_rad += r_operand.direction_rad;
		return *this;
	}
	template<typename V>
	inline coordinate<T>& operator+=(const V& r_operand){
		x += static_cast<T>(r_operand.x());
		y += static_cast<T>(r_operand.y());
		return *this;
	}
	inline coordinate<T>& operator-=(const coordinate<T>& r_operand){
		x -= r_operand.x;
		y -= r_operand.y;
		direction_rad -= r_operand.direction_rad;
		return *this;
	}
	template<typename V>
	inline coordinate<T>& operator-=(const V& r_operand){
		x -= static_cast<T>(r_operand.x());
		y -= static_cast<T>(r_operand.y());
		return *this;
	}
	template<typename V>
	inline coordinate<T>& operator*=(const V& r_operand){
		x *= static_cast<T>(r_operand);
		y *= static_cast<T>(r_operand);
		direction_rad *= static_cast<T>(r_operand);
		return *this;
	}
	template<typename V>
	inline coordinate<T>& operator/=(const V& r_operand){
		x /= static_cast<T>(r_operand);
		y /= static_cast<T>(r_operand);
		direction_rad /= static_cast<T>(r_operand);
		return *this;
	}

	/*
	 * 添え字による要素アクセス
	 */
	const T& operator[](size_t index) const& {return *((&x) + index);}
	T& operator[](size_t index) & {return *((&x) + index);}
	T operator[](size_t index) const&& {return *((&x) + index);}
};

/*以下doubleに対する特殊化*/
template<>
inline auto coordinate<double>::norm() const ->decltype(x + 0.0f) {return sqrt(pow(x, 2.0) + pow(y, 2.0));}
template<>
inline auto coordinate<double>::angle() const ->decltype(x + 0.0f) {return atan2(y, x);}

/*以下座標系(主にEigen　Vector系)に対しての算術演算子オーバーロード*/
template<typename T>
const inline Mecha::coordinate<T> operator+(const Mecha::coordinate<T>& l_operand, const Mecha::coordinate<T>& r_operand){
	return Mecha::coordinate<T>(l_operand) += r_operand;
}
template<typename T>
const inline Mecha::coordinate<T> operator-(const Mecha::coordinate<T>& l_operand, const Mecha::coordinate<T>& r_operand){
	return Mecha::coordinate<T>(l_operand) -= r_operand;
}
template<typename T>
const inline Mecha::coordinate<T> operator*(const Mecha::coordinate<T>& l_operand, const double& r_operand){
	return Mecha::coordinate<T>(l_operand) *= r_operand;
}
template<typename T>
const inline Mecha::coordinate<T> operator/(const Mecha::coordinate<T>& l_operand, const double& r_operand){
	return Mecha::coordinate<T>(l_operand) /= r_operand;
}

template<typename T, typename U>
const inline Mecha::coordinate<T> operator+(const Mecha::coordinate<T>& l_operand, const U& r_operand){
	return Mecha::coordinate<T>(l_operand) += r_operand;
}
template<typename T, typename U>
const inline Mecha::coordinate<T> operator+(const U& l_operand, const Mecha::coordinate<T>& r_operand){
	return Mecha::coordinate<T>(r_operand) += l_operand;
}
template<typename T, typename U>
const inline Mecha::coordinate<T> operator-(const Mecha::coordinate<T>& l_operand, const U& r_operand){
	return Mecha::coordinate<T>(l_operand) -= r_operand;
}
template<typename T, typename U>
const inline Mecha::coordinate<T> operator-(const U& l_operand, const Mecha::coordinate<T>& r_operand){
	return Mecha::coordinate<T>(T{l_operand.x()}-r_operand.x, T{l_operand.y()}-r_operand.y, r_operand.direction_rad);
}


/*
 * PID計算クラス
 */
template<class T>
class PID {
private:
	T* const _value;//出力値
	std::array<float, 3> _deviation = {0.0f, 0.0f, 0.0f};//偏差

	const bool _make_value_flag;

protected:
	/*各ゲイン*/
	float _P_gain;
	float _I_gain;
	float _D_gain;

	T _limit;
public:
	/*
	 * コンストラクタ
	 * */
	PID(T* value, float P_gain = 0.0f, float I_gain = 0.0f, float D_gain = 0.0f, T limit = static_cast<T>(0))
	: _value(value), _make_value_flag(false), _P_gain(P_gain), _I_gain(I_gain), _D_gain(D_gain),_limit(limit){}

	PID(float P_gain = 0.0f, float I_gain = 0.0f, float D_gain = 0.0f, T limit = static_cast<T>(0))
	: _value(new T(0)), _make_value_flag(true), _P_gain(P_gain), _I_gain(I_gain), _D_gain(D_gain),_limit(limit){}
	//コピー禁止
	PID(const PID<T>& pid) = delete;
	PID& operator=(const PID<T>& pid) = delete;


	/*
	 * デストラクタ
	 */
	~PID(){
		if(_make_value_flag)delete(_value);
	}

	/*
	 * ゲイン変更
	 */
	inline void chage_gaine(float P_gain = 0.0f, float I_gain = 0.0f, float D_gain = 0.0f){
		*_value = T{0};

		_P_gain = P_gain;
		_I_gain = I_gain;
		_D_gain = D_gain;
	}
	inline void chage_limit(T limit){
		*_value = T{0};

		_limit = limit;
	}

	/*
	 *制御演算
	 */
	void control(float deviation, uint32_t ms = 1){
		_deviation[2] = _deviation[1];
		_deviation[1] = _deviation[0];
		_deviation[0] = deviation;
		*_value +=
				_P_gain * (_deviation[0] - _deviation[1]) +
				_I_gain * static_cast<float>(ms) * (_deviation[0]) +
				_D_gain / static_cast<float>(ms) * (_deviation[0] - 2 * _deviation[1] + _deviation[2]);

		if((_limit != static_cast<T>(0)) && (std::abs(*_value) > _limit))*_value = (*_value > 0) ? _limit : -_limit;
	}


	/*
	 * ゲッター
	 */
	inline const T& get(){return *_value;}
	/*
	 * セッター
	 */
	inline void set(T value){*_value = value;}
};
}


