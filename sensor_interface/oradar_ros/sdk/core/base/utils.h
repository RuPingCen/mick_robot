
#ifndef UTILS_H_
#define UTILS_H_

#include <cinttypes>

namespace ordlidar
{

class Utils
{
public:
    
	static uint8_t xor_check(uint8_t *data, uint16_t len)
	{
		uint8_t check = 0;
		for (int i = 0; i < len; i++)
		{
			check ^= data[i];
		}
		return check;
	}

};

}

#endif
