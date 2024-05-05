/*
 * ChampiCan.h
 *
 *  Created on: Feb 19, 2024
 *      Author: arusso
 */

#ifndef INC_CHAMPICAN_H_
#define INC_CHAMPICAN_H_

#include "stm32g4xx_hal.h"

class ChampiCan {
public:
	ChampiCan();
	explicit ChampiCan(FDCAN_HandleTypeDef *handle_fdcan);
	int start();
    int stop();
	int send_frame(uint32_t id, uint8_t *frame_data, uint32_t size);
	int send_msg(uint32_t id, uint8_t *msg, uint32_t size);
	virtual ~ChampiCan();

private:
	FDCAN_HandleTypeDef *handle_fdcan_{};
	FDCAN_TxHeaderTypeDef tx_header_{};


};


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#endif /* INC_CHAMPICAN_H_ */
