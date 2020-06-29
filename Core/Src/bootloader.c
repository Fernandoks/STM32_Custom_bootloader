/*
 * bootloader.c
 *
 *  Created on: Jun 26, 2020
 *      Author: fernandoks
 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include "bootloader.h"
#include "stm32f4xx_hal.h"


extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;



void printmsg(char *format,...)
{
	uint8_t strg[50];

	va_list args;
	va_start(args, format);
	vsprintf(strg, format, args);
	HAL_UART_Transmit(&huart2, strg, strlen(strg), HAL_MAX_DELAY);
	va_end(args);
}



/***********************************************************************************************
 * This functions jumps to the user code application,
 * which should be available in FLASH SECTOR 2
 *
 * The user application need to be redirected by the linker to start
 * in the flash sector 2 0x0800 8000
 * Also you need to redirect the Vector table
 * VECT_TAB_OFFSET to 0x8000
 ***********************************************************************************************/

void bootloader_jump_to_user_application(void)
{

	void (*app_reset_handler)(void);

	printmsg("BL Message: bootloader_jump_to_user_application\r\n");

	//Configure MSP by reading the value from the Base address of the selected sector in Flash
	//This bootloader uses (sector 2)
	uint32_t msp_value = *(volatile uint32_t*) FLASH_SECTOR2_BASE_ADDRESS;
	printmsg("BL Message: MSP Value: %#x\r\n", msp_value);

	//set MSP
	__set_MSP(msp_value);

	//Get the User Application Reset Handler
    uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

    app_reset_handler = (void*) resethandler_address;

    //printmsg("BL_DEBUG_MSG: app reset handler addr : %#x\n",resethandler_address);

    //3. jump to reset handler of the user application
    app_reset_handler();

	//JUMP_TO_APP(userapp_resetHandlerAddress);

}

/***********************************************************************************************
 * HEAD function
 ***********************************************************************************************/

void bootloader_uart_read_data(void)
{
	static uint8_t blRxBuffer[255];

	HAL_UART_Receive(&huart2, &blRxBuffer, 1, HAL_MAX_DELAY);
	uint8_t recLenght = blRxBuffer[0];
	HAL_UART_Receive(&huart2, &blRxBuffer[1], recLenght, HAL_MAX_DELAY);
	switch (blRxBuffer[1])
	{
    	case BL_GET_VER:
		   bootloader_handle_getver_cmd(blRxBuffer);
		   break;
	   case BL_GET_HELP:
		   bootloader_handle_gethelp_cmd(blRxBuffer);
		   break;
	   case BL_GET_CID:
		   bootloader_handle_getcid_cmd(blRxBuffer);
		   break;
	   case BL_GET_RDP_STATUS:
		   bootloader_handle_getrdp_cmd(blRxBuffer);
		   break;
	   case BL_GO_TO_ADDR:
		   bootloader_handle_go_cmd(blRxBuffer);
		   break;
	   case BL_FLASH_ERASE:
		   bootloader_handle_flash_erase_cmd(blRxBuffer);
		   break;
	   case BL_MEM_WRITE:
		   bootloader_handle_mem_write_cmd(blRxBuffer);
		   break;
	   case BL_EN_RW_PROTECT:
		   bootloader_handle_en_rw_protect(blRxBuffer);
		   break;
	   case BL_MEM_READ:
		   bootloader_handle_mem_read(blRxBuffer);
		   break;
	   case BL_READ_SECTOR_P_STATUS:
		   bootloader_handle_read_sector_protection_status(blRxBuffer);
		   break;
	   case BL_OTP_READ:
		   bootloader_handle_read_otp(blRxBuffer);
		   break;
	   case BL_DIS_R_W_PROTECT:
		   bootloader_handle_dis_rw_protect(blRxBuffer);
		   break;
		default:
		   printmsg("BL_DEBUG_MSG:Invalid command code received from host \n");
		   break;
	}
}

/***********************************************************************************************
 * Command Functions
 ***********************************************************************************************/

void bootloader_handle_getver_cmd(uint8_t* blRxBuffer)
{
	printmsg("BL_DEBUG_MSG:bootloader_handle_getver_cmd \n");
	 //Total length of the command packet
	uint32_t command_packet_len = blRxBuffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (blRxBuffer+command_packet_len - 4) ) ;

	if (bootloader_verify_crc(&blRxBuffer[0],command_packet_len,host_crc) == CRC_ok )
	{
		printmsg("BL_DEBUG_MSG: CRC ok");
		bootloader_send_ack(blRxBuffer[0],1);
		uint8_t bl_version = get_bootloader_version();
		printmsg("BL_DEBUG_MSG: Bootloader version %d \n",bl_version);
		bootloader_uart_write_data(&bl_version,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: CRC error");
		bootloader_send_nack();
	}

}

void bootloader_handle_gethelp_cmd(uint8_t* blRxBuffer)
{
	printmsg("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd \n");
}

void bootloader_handle_getcid_cmd(uint8_t* blRxBuffer)
{
	__NOP();
}

void bootloader_handle_getrdp_cmd(uint8_t* blRxBuffer)
{
	__NOP();
}

void bootloader_handle_go_cmd(uint8_t *blRxBuffer)
{
	__NOP();
}

void bootloader_handle_flash_erase_cmd(uint8_t* blRxBuffer)
{
	__NOP();
}

void bootloader_handle_mem_write_cmd(uint8_t* blRxBuffer)
{
	__NOP();
}

void bootloader_handle_en_rw_protect(uint8_t* blRxBuffer)
{
	__NOP();
}

void bootloader_handle_mem_read(uint8_t* blRxBuffer)
{
	__NOP();
}

void bootloader_handle_read_sector_protection_status(uint8_t* blRxBuffer)
{
	__NOP();
}

void bootloader_handle_read_otp(uint8_t* blRxBuffer)
{
	__NOP();
}

void bootloader_handle_dis_rw_protect(uint8_t* blRxBuffer)
{
	__NOP();
}

/***********************************************************************************************
 * Static functions
 ***********************************************************************************************/

void bootloader_send_ack(uint8_t commandCode, uint8_t followLength)
{
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = followLength;
	HAL_UART_Transmit(&huart2, ack_buf, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(&huart2, &nack, 1, HAL_MAX_DELAY);
}

CRC_t bootloader_verify_crc(uint8_t* pData, uint32_t length, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xff;
	for (uint32_t i = 0; i < length; ++i)
	{
		uint32_t iData = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &iData, 1);
	}
	if ( uwCRCValue == crc_host)
	{
		return CRC_ok;
	}
	else return CRC_error;
}

uint8_t get_bootloader_version()
{
	return (uint8_t) BL_VERSION;
}

void bootloader_uart_write_data(uint8_t* pData, uint32_t length)
{
	HAL_UART_Transmit(&huart2, pData, length, HAL_MAX_DELAY);
}



