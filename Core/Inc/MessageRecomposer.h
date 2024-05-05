/*
 * MessageRecomposer.h
 *
 *  Created on: Feb 18, 2024
 *      Author: arusso
 */
#ifndef MESSAGERECOMPOSER_H_
#define MESSAGERECOMPOSER_H_

/*
 * Max size in can frames (=6 bytes) of full message expected. Real max is 64(2**4) frames.
 * Because the memory for the messages is allocated statically for better performance,
 * it's good to lower this value if possible to save memory.
 */
#define MAX_LENGTH_MSG 20

#include "string"

class MessageRecomposer {
public:
	MessageRecomposer();
	void add_frame(uint8_t* frame_data, uint8_t length);
    bool check_if_new_full_msg();
    std::string get_full_msg();
	virtual ~MessageRecomposer();

private:
	int msg_number_;
	int n_frames_; // nb of frames needed to get the full message
    bool full_msg_received_;
    bool frames_received_[MAX_LENGTH_MSG];
    std::string msg_parts[MAX_LENGTH_MSG];
    std::string full_msg_;

    void decode_descriptor(uint8_t* frame_data, int &msg_number, int &msg_size, int &frame_index);
    bool all_frames_received();
};

#endif /* MESSAGERECOMPOSER_H_ */
