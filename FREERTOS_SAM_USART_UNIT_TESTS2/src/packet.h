#ifndef	_PACKET_H
#define	_PACKET_H

/******************************************************************************
* packet.h 
* This header holds info related to the tx and rx of packetized data used to
* send commands and recieve status between the master and slave controllers
* on the robot.
******************************************************************************/
/*
* PACKET format:
*/
#define	TYPE_LEN		2			// 1 byte cmd category, 1 byte sub type
#define MAX_PKT_LEN		86			// Up to 255 bytes in packet so we can use 1 BYTE
//MSG_ID one byte
#define	MAX_DATA_LEN	80			// Max number of bytes of data in pkt.
#define	PKT_NUM_MAX		255			// Sequence # of this pkt in msg.
#define NUM_PACKETS_MAX	255			// Total number of packets possible in 1 msg.
			
/* The size of the buffer used to receive characters from the USART driver.
 * This equals the length of the longest string used in this file. */
#define PKT_BUFFER_SIZE          (MAX_PKT_LEN +1 )

/*
* PACKET format:
* 
*  2 BYTE 1 BYTE	1 BYTE	  	UP TO 80	1 BYTE	1 BYTE
*
* TYPE	PKT_LEN 	MSG_ID		DATA 	PKT_NUM	NUM_PACKETS 
* 
* TYPE is 2 bytes which allows for a CMD catagory and sub-type.
* NUM_PACKETS is the number of pkts. in this message.
* DATA can be any length and extend across multiple packets, but must be < MAX_DATA_LEN in each pkt.
* PKT_LEN is the number of bytes in this pkt. and is one byte in length.
* PKT_NUM If the message has multiple pkts., this will be the numerical sequence of a given pkt.
* MSG_ID is the ID of this message. A message will have its own ID.
* 
* Example: 0x01 0x11 0x01 0x07 0xF0 0x23 0x01 0x01 
* Packet = Servo move CCW, ID = 0x01, 7 bytes in this packet , Data, PKT_NUM, NUM_PACKETS
* 
* NOTE: There is no CRC because the serial port is really USB to RS232 port and USB
* Has CRC.
*
*/

#define	URT_RX_ERR	1	// usart did not read requested amount
#define	COM_QUE_ERR	2	// Queue error in COM manager

/**
* Packet info:
* COMMANDS/STATUS RESPONSES
*/
#define	SERVO_MV	(0x01)	// Command to move motor
#define	DIRECTION	(0x02)	// Direction to move
#define	STAT_RQ		(0x03)	// Status request
#define	STAT_RESP	(0x04)	// Status response
#define	AZMUTH		(0x05)	// Compass reading on turret
#define	HEADING		(0x07)	// Compass reading of robot direction 
#define	ERROR		(0x08)	// An error was encountered.
#define	TEST		(0x09)
#define	MOVE		(0x10)
#define	CLOCKWISE	(0x11)
#define	COUNTER_CW	(0x12)	// Counter clockwise
#define	STOP		(0x13)
#define	PKT_ID		(0x14)
#define	DC_MOTOR	(0x15)
#define	SERVO_TASK	(0x16)
#define	DC_MOTOR_TASK	(0x17)
#define	LIDAR_TASK	(0x18)
#define	DC_MOTOR_TASK	(0x19)
#define	SERVO_TASK	(0x20)


typedef struct 
{
	unsigned char typeh;
	unsigned char typel;
	unsigned char data_len;
	unsigned char msg_ID;
	unsigned char *data;
	unsigned char pkt_num;
	unsigned char num_pkts;	
} packet_t;

// unsigned short crc16(const unsigned char* data_p, unsigned char length);
void make_pkt(unsigned char *buff, char type, int data_len);
void send_packet_to_task(uint8_t  num, uint8_t *buffer);
uint8_t find_task(uint8_t type);
void log_error(uint8_t err_num);
/*
 * Sets up the hardware ready to run this example.
 */
static void prvSetupHardware(void);

#endif