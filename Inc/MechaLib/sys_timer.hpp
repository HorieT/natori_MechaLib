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
 * ���Ԑ���x�[�X
 * check()��1ms�����ŌĂяo���悤��
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
	//�ꎞ�I�u�W�F�N�g�p
	const bool _one_time;

	//�^�C���X�P�W���[���Z�b�g
	void schedule_set(){
		if(_setting)return;
		_setting = true;
		_scheduler.push_back(shared_from_this());
		_start = _time;
	}
	virtual void callback() = 0;


public:
	/*
	 * �R���X�g���N�^�E�f�X�g���N
	 */
	sysClock(uint32_t ms) : _period(ms), _one_time(false){}
	virtual ~sysClock(){erase();}

	/*
	 * �^�C���X�P�W���[������
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
	 * �^�C���X�P�W���[�����Z�b�g
	 */
	virtual inline void reset() final{_start = _time;}

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
				if(t->_one_time)t->erase();
				else t->_start = _time;
			}
		}
	}


	//���ʂ�delay�֐�(timeScheduler���ł͎g��Ȃ��悤��)
	[[deprecated("It may adversely affect the control system of 'sysClock'.")]]
	static void sys_delay(uint32_t ms){
		volatile uint32_t end = _time + ms;
		while(sysClock::get_time() <= end);//�֐��Ăяo���ɂ��œK���̏��O
	}

	//�֐��x���Ăяo�����N���X�O�ɋL�q

	//�����擾
	volatile static uint32_t get_time(){return _time;}
};

/*
 * �^�C���X�P�W���[��
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
 * void�^�̊��S���ꉻ�^�C���X�P�W���[��
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


//�x���֐�
template<typename T>
inline void sys_delay_call(std::function<void(T)>&& callback, T arg, uint32_t ms){
	typename timeScheduler<T>::timeScheduler(std::move(callback), ms).set(arg);
}
inline void sys_delay_call(std::function<void(void)>&& callback, uint32_t ms){
	typename timeScheduler<void>::timeScheduler(std::move(callback), ms).set();
}
}













