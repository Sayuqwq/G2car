#include "stm32f10x.h"                  // Device header
#include "Command.h"
#include "Direction.h"

#define TRUE 1
#define FALSE 0

_Bool Dxl_ledon(uint8_t DXL_ID)
{
	Dir_SetSend();
	
	{
		uint8_t packet[32];
		uint8_t data[8];
		*(uint8_t *)data = 1;		// data
		
		dyn2_init_write_packet(packet, DXL_ID, 65, 1, data);
		
		send_dynamixel_packet(packet);
	}
	
	Dir_SetReceive();

	return TRUE;
}
