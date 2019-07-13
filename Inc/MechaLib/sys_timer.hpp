/*
 * 2019/07/13 Horie
 */
#pragma once
#include <cstdint>
#include <vector>
#include <iterator>


namespace Mecha{

/*
 * 時間制御ベース
 */
class sysClock {
private:
	uint32_t _start = 0;
	uint32_t _period;
	bool _setting = false;
	static inline volatile uint32_t _time = 0;
	static inline std::vector<sysClock*> _scheduler;


	inline static void inc_time(){_time++;}

protected:
	//タイムスケジューラセット
	void set(){
		if(_setting)return;
		_setting = true;
		_scheduler.push_back(this);
		_start = _time;
	}
	virtual void callback() = 0;


public:
	/*
	 * コンストラクタ・デストラク
	 */
	sysClock(uint32_t ms) : _period(ms){}
	virtual ~sysClock(){erase();}

	/*
	 * タイムスケジューラ消去
	 */
	virtual void erase() final{
		for(auto it = _scheduler.begin(), e = _scheduler.end();it != e;++it){
			if(*it == this){
				_setting = false;
				_scheduler.erase(it);
				break;
			}
		}
	}
	/*
	 * タイムスケジューラリセット
	 */
	virtual void reset() final{_start = _time;}

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
				t->_start = _time;
			}
		}
	}


	//普通のdelay関数(timeScheduler内では使わないように)
	static void sys_delay(uint32_t ms){
		volatile uint32_t end = _time + ms;
		while(sysClock::get_time() <= end);//関数呼び出しによる最適化の除外
	}

	//関数遅延呼び出し
	/*
	template<class T>
	static void sys_delayCall(void (*callback)(T), T arg, uint32_t ms){

	}*/

	volatile static uint32_t get_time(){return _time;}
};

/*
 * タイムスケジューラ
 */
template<class T_arg>
class timeScheduler final: public sysClock{
public:
	timeScheduler(void (*func)(T_arg), uint32_t ms) : sysClock(ms), function(func){}

	void set(T_arg arg){
		argment = arg;
		sysClock::set();
	}

private:
	void (*function)(T_arg);
	T_arg argment = static_cast<T_arg>(0);

	void callback() override{function(argment);}
};

/*
 * void型の完全特殊化タイムスケジューラ
 */
template<>
class timeScheduler<void> final: public sysClock{
public:
	timeScheduler(void (*func)(void), uint32_t ms) : sysClock(ms), function(func){}

	void set(){sysClock::set();}

private:
	void (*function)(void);

	void callback() override{function();}
};

template<class T>
void class_scheduler_callback(T* c){
	c->scheduler_fanc();
}
}













