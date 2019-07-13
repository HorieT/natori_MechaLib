/*
 * 2019/06/27 Horie
 */
#pragma once
#include "MechaLib_HAL_global.hpp"

/*
 * 	アナログパッドL		アナログパッドR
 *		1
 *		↑				↑
 * 	1　←64→　127		　　←64→
 * 		↓				↓
 * 	　　　127
 *
 *		座標落とし込み
 *		　　　63
 *			↑
 *	　　-63　←0→　63
 *　　　　　　　　↓
 *		　　-63
 */



namespace Mecha{

class sbdbtPS3{
public:
	using button = uint32_t;

private:
	static constexpr uint8_t SBDBT_DATA_SIZE = 8;
	static constexpr uint8_t SBDBT_BUFF_SIZE = 16;
	static constexpr uint8_t SBDBT_BUFF_SIZE_D = SBDBT_BUFF_SIZE - 1;

	/*アナログパッド受信値*/
	static constexpr int8_t ANAROG_MAX =		127;
	static constexpr int8_t ANAROG_MIN =		0;
	static constexpr int8_t ANAROG_CENTER =		64;


	UART_HandleTypeDef* _uart;
	timeScheduler<sbdbtPS3*> _schduler;

	std::array<uint8_t, SBDBT_DATA_SIZE> _buttonn_data{0};
	std::array<uint8_t, SBDBT_DATA_SIZE> _last_buttonn_data{0};
	std::array<uint8_t, SBDBT_BUFF_SIZE> _buff{0};

	void (*_button_callback)(button bt) = 0;
	void (*_timeout_callback)(void);


	bool _continue_flag = false;

	static void scheduler_fanc(sbdbtPS3* me){me->timeout();}
	void timeout(void){
		_continue_flag = false;
		{
			uint8_t num = 0;
			for(auto& data : _buttonn_data){
				data = (num == 0)? 0x80 : ((num < 3 || num == 7) ? 0 : ANAROG_CENTER);
				++num;
			}
		}//for(uint8_t i = 0;i < buttonnData.size();i++)buttonnData.at(i) = 0;

		_last_buttonn_data = _buttonn_data;
		_timeout_callback();
	}

public:
	/*アナログパッド座標値*/
	static constexpr uint8_t USE_ANAROG_MAX =		63;
	static constexpr uint8_t USE_ANAROG_MIN =		-63;
	static constexpr uint8_t USE_ANAROG_CENTER =	0;

	/*下位*/
	static constexpr button PS3up =			0x0001;
	static constexpr button PS3down =		0x0002;
	static constexpr button PS3right =		0x0004;
	static constexpr button PS3left =		0x0008;
	static constexpr button PS3triangle =	0x0010;
	static constexpr button PS3cross =		0x0020;
	static constexpr button PS3circle =  	0x0040;
	static constexpr button PS3start =		0x0080;
	/*上位*/
	static constexpr button PS3square = 	0x0100;
	static constexpr button PS3L1 =			0x0200;
	static constexpr button PS3L2 =			0x0400;
	static constexpr button PS3R1 = 		0x0800;
	static constexpr button PS3R2 = 		0x1000;
	static constexpr button PS3L3 =			0x2000;
	static constexpr button PS3R3 =			0x4000;
	static constexpr button PS3select = 	0x8000;
	//便宜上の定義
	static constexpr button PS3AnalogLX =		0x000F0000;
	static constexpr button PS3AnalogLY = 		0x00F00000;
	static constexpr button PS3AnalogRX =		0x0F000000;
	static constexpr button PS3AnalogRY = 		0xF0000000;

	sbdbtPS3(UART_HandleTypeDef* huart, void (*Callback)(void), uint32_t time)
	: _uart(huart), _schduler(scheduler_fanc, time), _timeout_callback(Callback){}
	~sbdbtPS3(){_schduler.erase();}


	inline void init(){
		HAL_UART_Receive_DMA(_uart, _buff.data(), SBDBT_BUFF_SIZE);
		_schduler.set(this);
	}

	//PS3コントローラからの受信(※必ず HAL_UART_RxCpltCallback()内に記入すること)
	bool receive(UART_HandleTypeDef *huart){
		bool state = false;

		if(huart == _uart){
			std::array<uint8_t, SBDBT_BUFF_SIZE> hold_buff(_buff);
			uint32_t ndtr_ptr = _uart->hdmarx->Instance->NDTR;

			//リングバッファ全探索
			for(uint8_t j = 0;j < SBDBT_BUFF_SIZE;++j){

				//ヘッダ探索
				if((hold_buff[j] == 0x80) && (((j + ndtr_ptr) & SBDBT_BUFF_SIZE_D) < SBDBT_DATA_SIZE)){//受信中途データの場合はじく
					std::array<uint8_t, SBDBT_DATA_SIZE> data;
					uint8_t check_sum = 0;

					//バッファデータ移し
					/*
					for(uint8_t i = 0;i < SBDBT_DATA_SIZE;++i)
						data.at(i) = hold_buff.at((j + i) & SBDBT_BUFF_SIZE_D);
						↓
					 */
					{
						uint8_t i = 0;
						for(auto& d : data){
							d = hold_buff.at((j + i) & SBDBT_BUFF_SIZE_D);
							++i;
						}
					}

					//チェックサム生成
					/*
					for(uint8_t i = 1;i < SBDBT_DATA_SIZE - 1;i++)
						check_sum += data[i];
						↓
					 */
					for(auto it = std::next(data.begin(), 1), e = std::next(data.end(), -1);it != e;++it)
						check_sum = static_cast<uint8_t>(check_sum + *it);

					//チェックサム確認
					if((check_sum &  0x7F) == (data.at(7) & 0x7F)){
						_buttonn_data = data;

						if((_buttonn_data != _last_buttonn_data) && (_button_callback != nullptr)){//エッジ検出&NULLチェック
							button edge_button =
									(uint32_t)_buttonn_data.at(2) |
									((uint32_t)_buttonn_data.at(1) << 8) |
									((_buttonn_data.at(3) != _last_buttonn_data.at(3)) ? (0x0F << 16) : 0) |
									((_buttonn_data.at(4) != _last_buttonn_data.at(4)) ? (0xF0 << 16) : 0) |
									((_buttonn_data.at(5) != _last_buttonn_data.at(5)) ? (0x0F << 24) : 0) |
									((_buttonn_data.at(6) != _last_buttonn_data.at(6)) ? (0xF0 << 24) : 0);

							_continue_flag = true;
							_button_callback(edge_button);
						}else
							_continue_flag = true;

						_last_buttonn_data = _buttonn_data;
						state = true;
						_schduler.reset();
						break;
					}
				}
			}
		}
		return state;
	}

	//PS3コントローラの状態取得
	int8_t get_button(button bt){
		int8_t data = 0;

		if(!(((bt & 0xFFFF) != 0) && ((bt & 0xFFFF0000) != 0)) && _continue_flag){
			if(bt & 0xFFFF){
				data = ((((uint32_t)_buttonn_data.at(2) |	((uint32_t)_buttonn_data.at(1) << 8)) & bt) != 0);
			}else if(bt & 0xFFFF0000){
				switch(bt){
				case PS3AnalogLX:
					data = static_cast<uint8_t>(_buttonn_data.at(3) - ANAROG_CENTER);
					break;
				case PS3AnalogLY:
					data = static_cast<uint8_t>(ANAROG_MAX + 1 - static_cast<int8_t>(_buttonn_data.at(4)) - ANAROG_CENTER);
					break;
				case PS3AnalogRX:
					data = static_cast<uint8_t>(_buttonn_data.at(5) - ANAROG_CENTER);
					break;
				case PS3AnalogRY:
					data = static_cast<uint8_t>(_buttonn_data.at(6) - ANAROG_CENTER);
					break;
				default:
					break;
				}
			}

		}
		return data;
	}

	//PS3コントローラのボタン割り込み
	inline void set_sbdbt_Callback(void (*Callback)(button bt)){
		_button_callback = Callback;
	}


};

}
