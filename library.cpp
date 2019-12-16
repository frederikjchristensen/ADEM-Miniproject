#include "library.h"
// Contains adresses of the RAM control table
unsigned char addresses[31] = {0, 64, 65, 68, 69, 70, 76, 78, 80, 82, 84, 88, 90, 98, 100, 102, 104, 108, 112, 116, 120,
                               122, 123, 124, 126, 128, 132, 136, 140, 144, 146};
// Contains the expected amount of Bytes to the addresses.
unsigned char prefBytes[31] = {0, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 3, 4, 4, 6, 6, 6, 6, 4, 3, 3, 4, 4, 6, 6, 6, 6, 4,
                               3};


// CONSTRUCTOR
Dynamixel_p2::Dynamixel_p2(int flow_control_pin) { // flow control pin is defined by default at pin 13. But can be changed.
    _flow_control_pin = flow_control_pin; //It is standard to use _Variable when storing private variables.
    pinMode(_flow_control_pin, OUTPUT);
}

// PUBLIC
void Dynamixel_p2::setTorqueEnable(unsigned char id, unsigned char value) {
    unsigned char TorquePkg[13];
    Dynamixel_p2::ConstructPacket(TorquePkg, id, 0x03, value, 0x40);
    Dynamixel_p2::TransmitPacket(TorquePkg);
}

void Dynamixel_p2::PingServo(unsigned char id) {
    unsigned char PingPkg[10];
    Dynamixel_p2::ConstructPacket(PingPkg, id, 0x01, 0x00, 0x00);
    Dynamixel_p2::TransmitPacket(PingPkg);

}

void Dynamixel_p2::PERMRAM(){ // Set all servos to PWM mode.
    unsigned char EEPROMPkg[16];
    Dynamixel_p2::ConstructPacket(EEPROMPkg, 0xFE, 0x03, 3, 11); //Sets Operation mode to 0 (current control) for servo ID 1.
    Dynamixel_p2::TransmitPacket(EEPROMPkg);
}

void Dynamixel_p2::RAM(unsigned char id) {
    unsigned char RAMPkg[16];
    Dynamixel_p2::ConstructPacket(RAMPkg, id, 0x03, 100, 0x54); // Position gain P = 100, default setup. for ID
    Dynamixel_p2::TransmitPacket(RAMPkg);

    Dynamixel_p2::ConstructPacket(RAMPkg, id, 0x03, 1, 0x40); // Enables torque for ID
    Dynamixel_p2::TransmitPacket(RAMPkg);

}

void Dynamixel_p2::setLedStatus(unsigned char id, unsigned char value) {
    unsigned char Pkg[13];
    Dynamixel_p2::ConstructPacket(Pkg, id, WRITE, value, 0x41);
    Dynamixel_p2::TransmitPacket(Pkg);
}


void Dynamixel_p2::setPositionGainP(unsigned char id, unsigned int value) {
    unsigned char Pkg[14];
    Dynamixel_p2::ConstructPacket(Pkg, id, WRITE, value, 0x54);
    Dynamixel_p2::TransmitPacket(Pkg);
}


void Dynamixel_p2::setGoalPosition(unsigned char id, unsigned long value) {
    unsigned char GoalPkg[16];
    Dynamixel_p2::ConstructPacket(GoalPkg, id, 0x03, value, 0x74);
    Dynamixel_p2::TransmitPacket(GoalPkg);
}

unsigned char Dynamixel_p2::getID(char ID) {

    return genericGet<unsigned char>(ID, 1, 7);
}

unsigned char Dynamixel_p2::getTemperature(unsigned char id){
    return genericGet<unsigned char>(id, 1, 146);
}


// Following method written by github user zcshiner
void Dynamixel_p2::begin(long baud_rate = 57600) {
#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
    Serial1.begin(baud_rate);  // Set up Serial for Leonardo and Mega
    _serialport = &Serial1;
#else
    Serial.begin(baud_rate);   // Set up Serial for all others (Uno, etc)
    _serialport = &Serial;
#endif
}

// PRIVATE METHODS

void Dynamixel_p2::CreateHeader(unsigned char *tx_packet, unsigned char id) {
    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = 0xFD;
    tx_packet[3] = 0x00;
    tx_packet[4] = id;
}

void Dynamixel_p2::CreateInstruction(unsigned char *tx_packet, unsigned char instruction)
{
    tx_packet[7] = instruction;
}

unsigned short Dynamixel_p2::CreateLength(unsigned char *tx_packet, unsigned short blk_size) {
    unsigned short packet_length = blk_size + 3; // Value of length field
    unsigned char length1 = ((unsigned char) packet_length & 0xFF);
    unsigned char length2;
    if (packet_length > 0xFF) {
        length2 = (packet_length >> 8) & 0xFF;
    } else {
        length2 = 0x00;
    }
    tx_packet[5] = length1;
    tx_packet[6] = length2;
    return packet_length;
}

void Dynamixel_p2::ConstructPacket(unsigned char *tx_packet, unsigned char device_id, unsigned char instruction,
                                   unsigned long params, unsigned char address) {
    CreateHeader(tx_packet, device_id);
    CreateInstruction(tx_packet, instruction);
    char param_size;
    if (instruction == READ) {
        Dynamixel_p2::Create4Params((unsigned int) params, tx_packet, address);
        param_size = 4;
    }
    else {
        param_size = ChooseParams(params, address, tx_packet);
    }
    unsigned short packet_length = CreateLength(tx_packet, param_size);
    CreateCRC(tx_packet, packet_length + 5); // The 5 is the size of header.
}

void Dynamixel_p2::TransmitPacket(unsigned char *tx_packet) {
    digitalWrite(_flow_control_pin, HIGH);
    unsigned short bytes_in_packet = (tx_packet[6] << 8) + tx_packet[5] + 7; // +7 is Header + Length field

    for (int i = 0; i < bytes_in_packet; i++) {
        _serialport->write(tx_packet[i]);
        //Serial.print(tx_packet[i], HEX);
        //Serial.print(" ");
    }
    _serialport->flush();
    digitalWrite(_flow_control_pin, LOW);
}

Dynamixel_p2::status_packet_info Dynamixel_p2::ReceiveStatusPacket() {
    unsigned long start_time = micros(); // Init timer for timeout on recieving Status packet. We don't want to get stuck waiting for data.
    status_packet_info status; //Naming a variable "status" of the type struct, containing, ID, error, parameters[], params.
    status.error = 0x00; // Default error. (Nothing wrong)

    while (micros()<start_time+5000){
        if (_serialport->available() >= 7) { //Waits until at least 8 bytes are available allowing to read header, lengths etc.
            //Serial.println("Scanning for header...");
            // Get rid of the header
            for (int i = 0; i < 2; ++i) {
                if (_serialport->peek() == 0xFF) { //Peek allows you to see the next byte without removing it from the buffer.
                    _serialport->read();
                } else {
                    status.error = 0x08; // 0x08 is not defined in the protocol. Consider it an unknown error.
                    return status;
                }
            }

            if (_serialport->peek() == 0xFD) {
                _serialport->read();
            } else {
                status.error = 0x08;
                return status;
            }

            if (_serialport->peek() == 0x00) {
                _serialport->read();
            } else {
                status.error = 0x08;
                return status;
            }

            // Get ID, length
            unsigned char id = _serialport->read();
            unsigned char l1 = _serialport->read();
            unsigned char l2 = _serialport->read();
            unsigned short packet_length = l1 + (l2 << 8);
            status.id = id;

            // Recreate RX-packet
            unsigned char rx_packet[packet_length+7];
            rx_packet[0] = 0xFF;
            rx_packet[1] = 0xFF;
            rx_packet[2] = 0xFD;
            rx_packet[3] = 0x00;
            rx_packet[4] = id;
            rx_packet[5] = l1;
            rx_packet[6] = l2;

            while (_serialport->available() < packet_length) {
                // Serial.println(_serialport->available());
            }

            // Populate the rest of the packet with instr, err, params, crc
            for (int j = 0; j < packet_length; ++j) {
                rx_packet[j + 7] = _serialport->read();
            }

            // Set error in return value
            status.error = rx_packet[8];
            if (status.error != 0x00) return status;

            // Extract parameters
            for (int k = 0; k < packet_length - 4; ++k) {
                status.parameters[k] = rx_packet[9 + k];
            }

            unsigned short calc_crc = update_crc(0, rx_packet, packet_length+5);
            if ((rx_packet[packet_length+5] != (calc_crc & 0xFF)) | (rx_packet[packet_length+6] != ((calc_crc >> 8) & 0xFF))){
                status.error = 0x03; // CRC error
            }
            return status;
        }
    }

    status.error = 0x09; // TIMEOUT ERROR
    return status;
}

unsigned short Dynamixel_p2::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned short i, j;
    unsigned short crc_table[256] = {
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for (j = 0; j < data_blk_size; j++) {
        i = ((unsigned short) (crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}


void Dynamixel_p2::Create6Params(unsigned long value, unsigned char *package,
                                 unsigned char address) { // Function to split 32 bit value into 4x8 bit array.
    package[8] = address; //Adds low order byte address
    package[9] = 0x00;
    for (int i = 0; i < 4; i++) { // Repeats 4 times.
        package[10 + i] = value & 0x000000FF; // Runs bitmask over 32 bit value to 8 bit.
        value = value >> 8; //Bitshift value by 8 bits to the right.
    }
}

void Dynamixel_p2::Create4Params(unsigned int value, unsigned char *package,
                                 unsigned char address) { // Function split 16 bit value into 2x8 bit array.
    package[8] = address; // Adds low order byte address
    package[9] = 0x00;
    for (int i = 0; i < 2; i++) { // Repeats twice.
        package[10 + i] = value & 0x00FF; // Runs bitmask over 16bit value to 8bit.
        value = value >> 8; // Bitshifts value by 8 bits to the right.
    }
}

void Dynamixel_p2::Create3Params(unsigned char value, unsigned char *package,
                                 unsigned char address) { // Function split 8 bit value into 1x8 bit array.
    package[10] = value & 0xFF;// Runs bitmask over 8bit value to 8bit.
    package[8] = address; // Adds low order byte address.
    package[9] = 0x00;
}

char Dynamixel_p2::ChooseParams(unsigned long value, unsigned char address,
                                unsigned char *tx_packet) { // Takes a parameter and an address. Figures out how many bytes is needed.
    for (int i = 0; i < 31; i++) {
        if (addresses[i] == address) {
            switch (prefBytes[i]) {
                case 0:
                    return 0;
                case 3:
                    Create3Params((unsigned char) value, tx_packet, address);
                    return 3;
                case 4:
                    Create4Params((unsigned int) value, tx_packet, address);
                    return 4;
                case 6:
                    Create6Params(value, tx_packet, address);
                    return 6;
            }
        }
    }
}

void Dynamixel_p2::CreateCRC(unsigned char *tx_packet, unsigned short blk_size) {
    unsigned short cal_crc = update_crc(0, tx_packet, blk_size);
    tx_packet[blk_size] = (cal_crc & 0x00FF);
    tx_packet[blk_size + 1] = (cal_crc >> 8) & 0x00FF;
}

template <typename T>
T Dynamixel_p2::charArrayToValue(unsigned char *array) {
    T value = 0;
    for (int i = 0; i < sizeof(T); ++i) {
        value += ((T) array[i] << i*8) & ((T) 0xFF << i*8);
    }
    return value;
}

template <typename T>
T Dynamixel_p2::genericGet(unsigned char id, unsigned short bytes, unsigned short address) {
    unsigned char tx_packet[14];

    Dynamixel_p2::ConstructPacket(tx_packet, id, READ, bytes, address);
    Dynamixel_p2::TransmitPacket(tx_packet);

    status_packet_info status = Dynamixel_p2::ReceiveStatusPacket(); // stores the needed info from return packet.
    //Serial.write(status.error); Is for testing purposes.
    T receivedData = (T) Dynamixel_p2::charArrayToValue<T>(status.parameters); //Turns the char array back into a singular data type. Using template to eliminate need to specify data type.

    return receivedData;
}

