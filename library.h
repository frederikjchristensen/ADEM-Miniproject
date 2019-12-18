#ifndef ADEM_MINIPROJECT_LIBRARY_H
#define ADEM_MINIPROJECT_LIBRARY_H

#define WRITE 0x03
#define READ 0x02
#define PING 0x01

#include <Arduino.h>
using namespace std;


class Dynamixel_p2 {
private:
    // ATTRIBUTES
    Stream *_serialport; //_serialport is set as a pointer to Stream from Arduino.h
    int _flow_control_pin = 13;

    struct status_packet_info{ //Holder for status packet info.
        unsigned char id;
        unsigned char error;
        unsigned char parameters[4];
    };

    // METHODS
    void CreateHeader(unsigned char *tx_packet, unsigned char id); // Function to build header (FF FF FD 00)
    void CreateInstruction(unsigned char *tx_packet, unsigned char instruction);
    unsigned short CreateLength(unsigned char *tx_packet, unsigned short blk_size);
    //void InsertBytes(unsigned char *tx_packet[], int position); // Function to insert an arbitrary amount of bytes at position in array
    void ConstructPacket(unsigned char *tx_packet, unsigned char device_id, unsigned char instruction,
                         unsigned long params, unsigned char address); // Function that constructs packets, given id, instr, and parameters
    void TransmitPacket(unsigned char *tx_packet); // Function to send a package
    char ChooseParams(unsigned long value, unsigned char address, unsigned char *tx_packet); // Takes a parameter and an address. Figures out how many bytes is needed.
    status_packet_info ReceiveStatusPacket(); // Function to read the contents of received packages


    unsigned short update_crc (unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size); // Calculates CRC
    void CreateCRC(unsigned char *tx_packet, unsigned short blk_size);
    void Create6Params (unsigned long value, unsigned char *package, unsigned char address); // Function designed to split up a 32 Bit value in a 4x8 bit array.
    void Create4Params (unsigned int value, unsigned char *package, unsigned char address); // 16it to 2x8 bit.
    void Create3Params (unsigned char value, unsigned char *package, unsigned char address);

    template <typename T>
    T charArrayToValue(unsigned char *array); // Function to convert array of chars to type T

    template <typename T>
    T genericGet(unsigned char id, unsigned short bytes, unsigned short address); // Function template for reading from motors

public:
    // CONSTRUCTORS
    Dynamixel_p2(int flow_control_pin);
    // UTILITY
    void begin(long baud_rate);
    void PERMRAM(); //Function sets all dynamixels to run in Position control mode
    void RAM(unsigned char id); // Function for initial values in the RAM AREA (Gain etc).
    void Reboot(unsigned char id);

    // SETTERS
    void setTorqueEnable(unsigned char id, unsigned char value);
    void setPositionGainP(unsigned char id, unsigned int value);
    void setGoalPosition(unsigned char id, unsigned long value);
    void setLedStatus(unsigned char id, unsigned char value);

    // GETTERS
    unsigned char getID(char ID); // Get ID for any available dynamixels.
    unsigned char getTemperature(unsigned char id);

    // PING
    void PingServo(unsigned char id);
};
#endif //ADEM_MINIPROJECT_LIBRARY_H