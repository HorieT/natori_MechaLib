/*
 * 2019/07/13 Horie
 */
#pragma once
#include <cstdint>
#include <vector>
#include <iterator>


namespace Mecha{

/*
 * ���Ԑ���x�[�X
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
	//�^�C���X�P�W���[���Z�b�g
	void set(){
		if(_setting)return;
		_setting = true;
		_scheduler.push_back(this);
		_start = _time;
	}
	virtual void callback() = 0;


public:
	/*
	 * �R���X�g���N�^�E�f�X�g���N
	 */
	sysClock(uint32_t ms) : _period(ms){}
	virtual ~sysClock(){erase();}

	/*
	 * �^�C���X�P�W���[������
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
	 * �^�C���X�P�W���[�����Z�b�g
	 */
	virtual void reset() final{_start = _time;}

	/*
	 * �Q�b�^�[
	 */
	virtual inline uint32_t get_period() final{return _period;}


	/*
	 * ���Ԍv���@�������荞�݊֐����ŌĂяo��
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


	//���ʂ�delay�֐�(timeScheduler���ł͎g��Ȃ��悤��)
	static void sys_delay(uint32_t ms){
		volatile uint32_t end = _time + ms;
		while(sysClock::get_time() <= end);//�֐��Ăяo���ɂ��œK���̏��O
	}

	//�֐��x���Ăяo��
	/*
	template<class T>
	static void sys_delayCall(void (*callback)(T), T arg, uint32_t ms){

	}*/

	volatile static uint32_t get_time(){return _time;}
};

/*
 * �^�C���X�P�W���[��
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
 * void�^�̊��S���ꉻ�^�C���X�P�W���[��
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













