#pragma once
#include <stdint.h>
#include <ros/ros.h>

struct NetBuffer
{
    uint8_t frame_head_;
    uint8_t car_id_;
    uint16_t my_bullet_;
    uint16_t my_health_;
    uint16_t my_position_x_;
    uint16_t my_position_y_;
    uint16_t enemy1_health_;
    uint16_t enemy1_position_x_;
    uint16_t enemy1_position_y_;
    uint16_t enemy2_health_;
    uint16_t enemy2_position_x_;
    uint16_t enemy2_position_y_;
    uint8_t frame_tail_;
};


bool udp_encode(NetBuffer& buf, uint8_t send_bytes[])
{
    short* data_ptr = (short *)(send_bytes + 2);

    // buf protocol
    send_bytes[0]  = buf.frame_head_;        // 0           
    send_bytes[1]  = buf.car_id_;            // 1
    data_ptr[0]    = buf.my_bullet_;         // 2 3
    data_ptr[1]    = buf.my_health_;         // 4 5
    data_ptr[2]    = buf.my_position_x_;     // 6 7
    data_ptr[3]    = buf.my_position_y_;     // 8 9
    data_ptr[4]    = buf.enemy1_health_;     // 10 11
    data_ptr[5]    = buf.enemy1_position_x_; // 12 13
    data_ptr[6]    = buf.enemy1_position_y_; // 14 15
    data_ptr[7]    = buf.enemy2_health_;     // 16 17
    data_ptr[8]    = buf.enemy2_position_x_; // 18 19
    data_ptr[9]    = buf.enemy2_position_y_; // 20 21
    send_bytes[22] = buf.frame_tail_;        // 22
}

bool udp_decode(NetBuffer& buf, uint8_t send_bytes[], const int bytes_transferred)
{
    if (buf.car_id_     != send_bytes[1]      &&  // check if self-robot
        buf.frame_head_ == send_bytes[0]      &&
        buf.frame_tail_ == send_bytes[bytes_transferred - 1] )
    {
        // ok! decode.
        buf.car_id_            = send_bytes[1];
        buf.my_bullet_         = send_bytes[3]<<8 + send_bytes[2];
        buf.my_health_         = send_bytes[5]<<8 + send_bytes[4];
        buf.my_position_x_     = send_bytes[7]<<8 + send_bytes[6];
        buf.my_position_y_     = send_bytes[9]<<8 + send_bytes[8];
        buf.enemy1_health_     = send_bytes[11]<<8 + send_bytes[10];
        buf.enemy1_position_x_ = send_bytes[13]<<8 + send_bytes[12];
        buf.enemy1_position_y_ = send_bytes[15]<<8 + send_bytes[14];
        buf.enemy2_health_     = send_bytes[17]<<8 + send_bytes[16];
        buf.enemy2_position_x_ = send_bytes[19]<<8 + send_bytes[18];
        buf.enemy2_position_y_ = send_bytes[21]<<8 + send_bytes[20];
        return true;
    }
    else {
        // todo: add info
        return false;
    }
}