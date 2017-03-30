/**************************************************************
* This file is part of the RoboMower project.
* This is the main.c file as part of a program used to debug
* the communications/messaging interface from the master to
* a micro-controller board through a serial port(USB 2 RS232).
* Messages are packetized and transmitted at 115200 baud.
* See packet.h for format of messages.
**************************************************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "packet.h"
#include "serial.h"

/* NOTE TO SELF: add the line chmod o+rw /dev/ttyS* to /etc/rc.d/rc.local */
#define NUM_COUNT   36
#define MAX_STR_LEN 80  // guess for now
#define MAXLINELENGTH   80 // guess for now

    // globals
const char port_name[] = "/dev/ttyUSB0";

void top_menu(int shandle);
int init_serial_port(void);
int compass_rd_cmd(unsigned char number_of_points, packet_t *pkt, int shandle);
int show_menu(void);
int servo_up_cmd(unsigned char position, packet_t *pkt, int port_handle);
int servo_dn_cmd(unsigned char position, packet_t *pkt, int sp_handle);
int RD_encoders_cmd(packet_t *pkt, int sp_handle);
int RD_lidar_cmd(packet_t *pkt, int sp_handle, int amount);
int turret_cw_cmd(unsigned char position, packet_t *pkt, int sp_handle);
int turret_ccw_cmd( unsigned char position, packet_t *pkt,int sp_handle);
int motors_cmd(unsigned char *params, packet_t *pkt, int sp_handle);
void finished(void);
int RD_encoders_cmd(packet_t *pkt, int sp_handle);
int wait_for_SlaveMessage(int handle, unsigned char *buffer);


packet_t *packet;

int main(void)
{
    int serial_handle;
    int ret;
    unsigned char buff[sizeof(packet_t)];

    serial_handle = init_serial_port();

    while(1)
    {
        top_menu(serial_handle);
        ret = wait_for_SlaveMessage(serial_handle, buff);
    }


}


void top_menu(int shandle)
{
    int val, ret;
    unsigned char number_of_points = 1; // read this many times
    unsigned char position = (unsigned char)NULL;
    static unsigned char buffer[sizeof(packet_t)];
    packet = (packet_t*)buffer;
    unsigned char instructions[4];

    while(1)
    {
    val = show_menu();
    switch(val)
    {
        case 1:
            ret = compass_rd_cmd(number_of_points, packet, shandle);
            break;
        case 2:
            ret = servo_up_cmd(position ,packet, shandle);
            break;
        case 3:
            ret = servo_dn_cmd(position, packet, shandle);
            break;
        case 4:
            ret = RD_encoders_cmd(packet, shandle);
            break;
        case 5:
            ret = RD_lidar_cmd(packet, shandle, 1);
            break;
        case 6:
            ret = turret_cw_cmd(position, packet, shandle);
            break;
        case 7:
            ret = turret_ccw_cmd(position, packet, shandle);
            break;
        case 8:
            instructions[0] = FORWARD; // Left motor direction
            instructions[1] = FORWARD; // Right motor dir
            instructions[2] = 100;      // encoder counts to move left motor
            instructions[3] = 100;      // right motor encoder count to move
            ret = motors_cmd(&instructions[0], packet, shandle);
            break;
        case 9:
            finished();
            break;
        default:
            printf("Un-recognized command!\n");
            break;
    }

    }
}

void finished(void)
{
    exit(0);
}

int show_menu(void)
{
    int response;
    int ret;

        printf("Robomower communications debug menu\n");
        printf("\n");
        printf("1. Read compass \n");
        printf("\n");
        printf("2. Servo up \n");
        printf("\n");
        printf("3. Servo down \n");
        printf("\n");
        printf("4. Read encoders \n");
        printf("\n");
        printf("5. Read LIDAR \n");
        printf("\n");
        printf("6. Turret CW \n");
        printf("\n");
        printf("7. Turret CCW \n");
        printf("\n");
        printf("8. Motors \n");
        printf("\n");
        printf("9. Exit \n");



    ret = scanf("%d", &response); // get users choice
    if(ret < 0)
        {
            printf("Error in response. Try again!\n");
            exit(0);
        }

    return response;
}



int init_serial_port(void)
{
    int port_handle;
    port_handle = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY); /* O_NDELAY = DCD is dont care */
    set_options(port_handle);
    return port_handle;
}

int uartSendByte(int fd, unsigned char byte)
{
  int n;

  n = write(fd, &byte, 1);
  if(n<0)
    return(1);

  return(0);
}

    /********************************************************************
    * Send a serial command to the slave telling it to read the compass.
    * The first argument is how many times to read it.
    ********************************************************************/
int compass_rd_cmd(unsigned char number_of_points, packet_t *pkt, int port_handle)
{
    int ret;
    pkt->typeh = COMPASS;
    pkt->typel = HEADING;
    pkt->data_len = 0x01;
    pkt->data[0] = number_of_points;
    pkt->msg_ID = manage_msg_id(GET_NEXT_ID); // get a new msg number
    pkt->num_pkts = 1;
    pkt->pkt_num = 1;
    ret = send_pkt_to_port(port_handle, pkt);
    return ret;
}

    /********************************************************************
    * Send a serial command to the slave telling it to  move the turret
    * servo up.
    ********************************************************************/
int servo_up_cmd(unsigned char position, packet_t *pkt, int port_handle)
{
    int ret;
    pkt->typeh = SERVO_MV;
    pkt->typel = UP;
    pkt->data_len = 0x01;
    pkt->data[0] = position;
    pkt->msg_ID = manage_msg_id(GET_NEXT_ID);
    pkt->num_pkts = 1;
    pkt->pkt_num = 1;
    ret = send_pkt_to_port(port_handle, pkt);
    return ret;
}

    /********************************************************************
    * Send a serial command to the slave telling it to move the turret
    * servo down.
    ********************************************************************/
int servo_dn_cmd(unsigned char position, packet_t *pkt, int port_handle)
{
    int ret;
    pkt->typeh = SERVO_MV;
    pkt->typel = DOWN;
    pkt->data_len = 0x01;
    pkt->data[0] = position;
    pkt->msg_ID = manage_msg_id(GET_NEXT_ID);
    pkt->num_pkts = 1;
    pkt->pkt_num = 1;
    ret = send_pkt_to_port(port_handle, pkt);
    return ret;
}

    /********************************************************************
    * Send a serial command to the slave telling it to read the wheel
    * encoders.
    ********************************************************************/
int RD_encoders_cmd(packet_t *pkt, int sp_handle)
{
    int ret;
    pkt->typeh = ENCODERS;
    pkt->typel = READ;
    pkt->data_len = 0x01;
    pkt->data[0] = 0x00; // NULL - place holder
    pkt->msg_ID = manage_msg_id(GET_NEXT_ID);
    pkt->num_pkts = 1;
    pkt->pkt_num = 1;
    ret = send_pkt_to_port(sp_handle, pkt);
    return ret;
}

int RD_lidar_cmd(packet_t *pkt, int sp_handle, int num)
{
    int ret;
    pkt->typeh = LIDAR;
    pkt->typel = READ;
    pkt->data_len = 0x01;
    pkt->data[0] = num; // How many points to read
    pkt->msg_ID = manage_msg_id(GET_NEXT_ID);
    pkt->num_pkts = 1;
    pkt->pkt_num = 1;
    ret = send_pkt_to_port(sp_handle, pkt);
    return ret;
}

int turret_cw_cmd(unsigned char position, packet_t *pkt, int sp_handle)
{
    int ret;
    pkt->typeh = SERVO_MV;
    pkt->typel = CLOCKWISE;
    pkt->data_len = 0x01;
    pkt->data[0] = position;
    pkt->msg_ID = manage_msg_id(GET_NEXT_ID);
    pkt->num_pkts = 1;
    pkt->pkt_num = 1;
    ret = send_pkt_to_port(sp_handle, pkt);
    return ret;
}

int turret_ccw_cmd(unsigned char position, packet_t *pkt,int sp_handle)
{
    int ret;
    pkt->typeh = SERVO_MV;
    pkt->typel = COUNTER_CW;
    pkt->data_len = 0x01;
    pkt->data[0] = position;
    pkt->msg_ID = manage_msg_id(GET_NEXT_ID);
    pkt->num_pkts = 1;
    pkt->pkt_num = 1;
    ret = send_pkt_to_port(sp_handle, pkt);
    return ret;
}

int motors_cmd(unsigned char *instructions, packet_t *pkt, int sp_handle)
{
    int ret;
    unsigned char *instr = instructions;
    pkt->typeh = DC_MOTOR;
    pkt->typel = MOVE;
    pkt->data_len = 0x04;
    pkt->data[0] = *instr++; // Left motor direction
    pkt->data[1] = *instr++; // right motor direction
    pkt->data[2] = *instr++;
    pkt->data[3] = *instr;
    pkt->msg_ID = manage_msg_id(GET_NEXT_ID);
    pkt->num_pkts = 1;
    pkt->pkt_num = 1;
    ret = send_pkt_to_port(sp_handle, pkt);
    return ret;
}

    /************************************************************
    * make a packet from the data in the buffer.
    ************************************************************/
void fill_data(unsigned char *buff, packet_t *pkt)
{
    int ctr;

    for(ctr = 0; ctr < pkt->data_len; ctr++)
    {
        pkt->data[ctr] = 0x00;   // zero out
    }

    for(ctr = 0; ctr < pkt->data_len; ctr++)
    {
        pkt->data[ctr] = *buff++;   // load data
    }
}

    /**************************************************************
    * Get a new message ID number. Each message will have an ID
    * number. There may be several packets in a message. Each
    * packet in a message will have the same ID number.
    **************************************************************/
int manage_msg_id(unsigned char type_cmd)
{
    int static new_ID_number;

    if(type_cmd == GET_NEXT_ID)
            new_ID_number++;
    if((type_cmd == RESET_IDS) || (new_ID_number >= MAX_IDS))
        new_ID_number = 0;
    return(new_ID_number);
}

int wait_for_SlaveMessage(int handle, unsigned char *pak)
{
    int val;
    val = wait_reply(handle, pak);
    return val;
}
