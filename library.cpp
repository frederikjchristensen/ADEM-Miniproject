#include "library.h"
// Contains adresses of the RAM control table
unsigned char addresses[31] = {0, 64, 65, 68, 69, 70, 76, 78, 80, 82, 84, 88, 90, 98, 100, 102, 104, 108, 112, 116, 120,
                               122, 123, 124, 126, 128, 132, 136, 140, 144, 146};
// Contains the expected amount of Bytes to the addresses.
unsigned char prefBytes[31] = {0, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 3, 4, 4, 6, 6, 6, 6, 4, 3, 3, 4, 4, 6, 6, 6, 6, 4,
                               3};



