#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#define MB_PACKET_SIZE				(260)
#define MB_BUF_SIZE_IN				(16)
#define MB_BUF_SIZE_OUT				(16)
#define MB_BUF_SIZE_3x				(256)
#define MB_BUF_SIZE_4x				(256)

#define MB_STATE_START_WAIT			(0)
#define MB_STATE_WAIT				    (1)
#define MB_STATE_READY			   	(4)
#define MB_STATE_SEND				    (5)

extern volatile uint16_t buf_3x[],buf_4x[];

void Modbus_Config(uint32_t baud);
void Modbus_set_addr(uint8_t addr);
void do_modbus(void);

#endif
