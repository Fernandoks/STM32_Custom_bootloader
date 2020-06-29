/*
 * bootloader.h
 *
 *  Created on: Jun 26, 2020
 *      Author: fernandoks
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_


/***********************************************************************************************
 * Defines
 ***********************************************************************************************/
#define FLASH_SECTOR2_BASE_ADDRESS 					0x08008000U

//version 1.0
#define BL_VERSION 0x10

//bootloader commands
//#define  <command name >	<command_code>

//This command is used to read the bootloader version from the MCU
#define BL_GET_VER									0x51
//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP									0x52
//This command is used to read the MCU chip identification number
#define BL_GET_CID									0x53
//This command is used to read the FLASH Read Protection level.
#define BL_GET_RDP_STATUS							0x54
//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR								0x55
//This command is used to mass erase or sector erase of the user flash .
#define BL_FLASH_ERASE          					0x56
//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE								0x57
//This command is used to enable or disable read/write protect on different sectors of the user flash .
#define BL_EN_RW_PROTECT							0x58
//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ									0x59
//This command is used to read all the sector protection status.
#define BL_READ_SECTOR_P_STATUS						0x5A
//This command is used to read the OTP contents.
#define BL_OTP_READ									0x5B
//This command is used disable all sector read/write protection
#define BL_DIS_R_W_PROTECT							0x5C
/* ACK and NACK bytes*/
#define BL_ACK   0XA5
#define BL_NACK  0X7F

/***********************************************************************************************
 * Types
 **********************************************************************************************/

typedef enum
{
	status_ok,
	status_error
} status_t;

typedef enum
{
	CRC_ok,
	CRC_error
} CRC_t;


/***********************************************************************************************
 * Command Function Prototypes
 ***********************************************************************************************/
void printmsg(char *format,...);

void bootloader_jump_to_user_application(void);
void bootloader_uart_read_data(void);
void bootloader_handle_getver_cmd(uint8_t* blRxBuffer);
void bootloader_handle_gethelp_cmd(uint8_t* blRxBuffer);
void bootloader_handle_getcid_cmd(uint8_t* blRxBuffer);
void bootloader_handle_getrdp_cmd(uint8_t* blRxBuffer);
void bootloader_handle_go_cmd(uint8_t *blRxBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t* blRxBuffer);
void bootloader_handle_mem_write_cmd(uint8_t* blRxBuffer);
void bootloader_handle_en_rw_protect(uint8_t* blRxBuffer);
void bootloader_handle_mem_read(uint8_t* blRxBuffer);
void bootloader_handle_read_sector_protection_status(uint8_t* blRxBuffer);
void bootloader_handle_read_otp(uint8_t* blRxBuffer);
void bootloader_handle_dis_rw_protect(uint8_t* blRxBuffer);

/***********************************************************************************************
 * Static Function Prototypes
 ***********************************************************************************************/
void bootloader_send_ack(uint8_t commandCode, uint8_t followLength);
void bootloader_send_nack(void);
CRC_t bootloader_verify_crc(uint8_t* pData, uint32_t length, uint32_t crc_host);
uint8_t get_bootloader_version();
void bootloader_uart_write_data(uint8_t* pData, uint32_t length);

#endif /* INC_BOOTLOADER_H_ */
