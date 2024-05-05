/*
 * ChampiState.h
 *
 *  Created on: Mar 3, 2024
 *      Author: arusso
 */

#ifndef INC_CHAMPISTATE_H_
#define INC_CHAMPISTATE_H_

#include <pb_encode.h>
#include "msgs_can.pb.h"
#include "ChampiCan.h"
#include "can_ids.hpp"

#define LIMIT_LENGTH_MSG 20 // max nb of frames per message. TODO uniformize and document ChampiCAN protocol


class ChampiState {
public:
    ChampiState();
    ChampiState(ChampiCan *champi_can_interface, uint32_t tx_period_ms);
	virtual ~ChampiState();
    void report_status(msgs_can_ActStatus status);
    void spin_once();
private:
    msgs_can_ActStatus status_msg_;
    ChampiCan *champi_can_interface_;
    uint32_t tx_period_ms_;
    uint32_t last_tx_time_ms_;
    uint8_t tx_buffer_[6 * LIMIT_LENGTH_MSG]{}; // 6 because 6 bytes per frame TODO put that 6 in a constant in ChampiCAN

    void send_status();
};

#endif /* INC_CHAMPISTATE_H_ */
