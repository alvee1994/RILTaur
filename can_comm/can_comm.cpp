#include "can_comm.h"
#include "CAN.h"
// #include "mbed.h"

void send(CAN& can_interface, uint8_t can_id, uint8_t cmsg[], int dlc) {
    if (can_interface.write(CANMessage(can_id, cmsg, dlc, CANData, CANStandard))) {
    } 
}

void receive(CAN& can_interface){
    int position;
    int velocity;
    int current;
    uint8_t ID;
    CANMessage msg;
    
    if(can_interface.read(msg)) {
        // printf("\nLength: %d\n", msg.len);

        ID = msg.data[0];
        position = msg.data[1] << 8 | msg.data[2];
        velocity = msg.data[3] << 4 | (msg.data[4] & 0xf0);
        current = (msg.data[4] & 0xf) << 8 | msg.data[5];
        // printf("\nID: %d, position: %d, velocity: %d, current: %d\n", ID, position, velocity, current);
    }
}

void transmit(CAN& can_interface, uint8_t can_id, int position, int velocity, int kp, int kd, int ff){

    CANMessage msg;
    // convert all the floats to uint
    // position
    msg.data[0] = position >> 8; // 8 bit
    msg.data[1] = position & 0xff; // 8 bit

    // velocity
    msg.data[2] = velocity >> 4; // 8 bit
    msg.data[3] = (velocity & 0xf) << 4; // 4 bit

    // kp
    msg.data[3] = (msg.data[3]|kp >> 8); // 4 bit by kp
    msg.data[4] = kp & 0xff; // 8 bit by kp

    // kd
    msg.data[5] = kd >> 4; // 8 bit
    msg.data[6] = (kd & 0xf) << 4; // 4 bit

    // ff
    msg.data[6] = (msg.data[6]|ff >> 8); // 4 bit by kp
    msg.data[7] = ff & 0xff; // 8 bit by kp

    send(can_interface, can_id, msg.data, 8);

    // printf("position %d\n", (msg[0]<<8|msg[1]));
    // printf("velocity %d\n", (msg[2]<<4|msg[3]>>4));
    // printf("kp %d\n", ((msg[3]&0xf)<<8|msg[4]));
    // printf("kd %d\n", (msg[5]<<4|msg[6]>>4));
    // printf("ff %d\n\n", ((msg[6]&0xf)<<8|msg[7]));

    int p_int = (msg.data[0]<<8|msg.data[1]);
    // float p_float = uint_to_float(int (p_int), -95.0f, +95.0f, 16);
    // printf("pos is p_int %i\n", p_int);
}

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
