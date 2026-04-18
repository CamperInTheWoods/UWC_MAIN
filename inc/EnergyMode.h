/*
 * EnergyMode.h
 *
 *  Created on: 2018. 7. 2.
 *      Author: ki
 */

#ifndef ENERGYMODE_H_
#define ENERGYMODE_H_

// ENERGY_MODE_TEST가 정의되면
// 적색 LED가 켜지면 동작 상태이고
// 청색 LED가 켜지면 슬립 상태를 표시한다.
//
//#define ENERGY_MODE_TEST
//
// 키입력 발생하면 10초동안 동작후 슬립으로 들러가고
// 아무키도 입력되지 않으면 0.5초동작후 슬립을 반복한다.
// 슬립상태는 5초간 지속되고 키입력이 발생하면 즉시 깨어난다.
//

//#define WORK_SHORT_TIME		5000L // 0.5초
#define WORK_SHORT_TIME		1000L // 0.1초
#define WORK_LONG_TIME		100000L // 10초
//#define WAKE_UP_TIME		30000L // 3초
//#define WAKE_UP_TIME		5000L // 0.5초
#define SLEEP_INHBIT_TIME	350000L // 35초
//#define SLEEP_INHBIT_TIME	100000L // 10초

// Clock defines
//#define LFXO_FREQUENCY         32768
#define ULFRCO_FREQUENCY         1000
//#define RTC_TIMEOUT_S          5
//#define RTC_TIMEOUT_S          2	//  in RTC_TIMEOUT_S * 100ms
#define RTC_TIMEOUT_S          50	//  in RTC_TIMEOUT_S * 100ms
#define RTC_TIMEOUT_S_MIN      1	//  in RTC_TIMEOUT_S * 100ms
//#define RTC_COUNT_TO_WAKEUP    (LFXO_FREQUENCY * RTC_TIMEOUT_S)
#define RTC_COUNT_TO_WAKEUP    		( (ULFRCO_FREQUENCY * RTC_TIMEOUT_S)/10 )
#define RTC_COUNT_TO_WAKEUP_MIN 	( (ULFRCO_FREQUENCY * RTC_TIMEOUT_S_MIN)/10 )




void ClearSleepCounter(long time);

#endif /* ENERGYMODE_H_ */
