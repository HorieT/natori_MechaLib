/*
  * 2019/07/13 Horie
 */
#pragma once
#include <cstdint>
#include <vector>
#include <iterator>
#include <functional>
#include <memory>

namespace Mecha{

/*
 * 時間制御ベース
 * check()を1ms周期で呼び出すように
 */
class sysClock : public std::enable_shared_from_this<sysClock>{
private:
	uint32_t _start = 0;
	uint32_t _period;
	bool _setting = false;
	static inline volatile uint32_t _time = 0;
	static inline std::vector<std::shared_ptr<sysClock>> _scheduler;


	static inline void inc_time(){_time++;}

protected:
	//一時オブジェクト用
	const bool _one_time;

	//タイムスケジューラセット
	void schedule_set(){
		if(_setting)return;
		_setting = true;
		_scheduler.push_back(shared_from_this());
		_start = _time;
	}
	virtual void callback() = 0;


public:
	/*
	 * コンストラクタ・デストラク
	 */
	sysClock(uint32_t ms) : _period(ms), _one_time(false){}
	virtual ~sysClock(){erase();}

	/*
	 * タイムスケジューラ消去
	 */
	virtual void erase() final{
		if(!_setting)return;
		for(auto it = _scheduler.begin(), e = _scheduler.end();it != e;++it){
			if(*it == shared_from_this()){
				_setting = false;
				_scheduler.erase(it);
				break;
			}
		}
	}
	/*
	 * タイムスケジューラリセット
	 */
	virtual inline void reset() final{_start = _time;}

	/*
	 * ゲッター
	 */
	virtual inline uint32_t get_period() final{return _period;}


	/*
	 * 時間計測　周期割り込み関数内で呼び出し
	 */
	static void check(){
		inc_time();
		for(auto t : _scheduler){
			if((_time - t->_start) >= t->_period){
				t->callback();
				if(t->_one_time)t->erase();
				else t->_start = _time;
			}
		}
	}


	//普通のdelay関数(timeScheduler内では使わないように)
	[[deprecated("It may adversely affect the control system of 'sysClock'.")]]
	static void sys_delay(uint32_t ms){
		volatile uint32_t end = _time + ms;
		while(sysClock::get_time() <= end);//関数呼び出しによる最適化の除外
	}

	//関数遅延呼び出し→クラス外に記述

	//時刻取得
	volatile static uint32_t get_time(){return _time;}
};

/*
 * タイムスケジューラ
 */
template<class Args>
class timeScheduler final: public sysClock{
public:
	timeScheduler(std::function<void(Args)>&& func, uint32_t ms) : sysClock(ms), _callbuck_funk(func){}
	virtual ~timeScheduler() override{erase();}

	void set(Args arg) &{
		argment = arg;
		sysClock::schedule_set();
	}
	void set(Args arg) &&{
		argment = arg;
		const_cast<bool&>(_one_time) = true;
		sysClock::schedule_set();
	}

private:
	std::function<void(Args)> _callbuck_funk;
	Args argment = static_cast<Args>(0);

	void callback() override{_callbuck_funk(argment);}
};

/*
 * void型の完全特殊化タイムスケジューラ
 */
template<>
class timeScheduler<void> final: public sysClock{
public:
	timeScheduler(std::function<void(void)>&& func, uint32_t ms) : sysClock(ms), _callbuck_funk(func){}
	virtual ~timeScheduler() override{erase();}

	void set() &{sysClock::schedule_set();}
	void set() &&{
		const_cast<bool&>(_one_time) = true;
		sysClock::schedule_set();
	}
private:
	std::function<void(void)> _callbuck_funk;

	void callback() override{_callbuck_funk();}
};


//遅延関数
template<typename T>
inline void sys_delay_call(std::function<void(T)>&& callback, T arg, uint32_t ms){
	typename timeScheduler<T>::timeScheduler(std::move(callback), ms).set(arg);
}
inline void sys_delay_call(std::function<void(void)>&& callback, uint32_t ms){
	typename timeScheduler<void>::timeScheduler(std::move(callback), ms).set();
}
}













