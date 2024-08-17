/*
 * USB_comms.c
 *
 *  Created on: Aug 8, 2024
 *      Author: jaron
 */
#include "USB_comms.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

extern char usb_send_buffer[32];
extern char usb_receive_buffer[32];

void USB_transmit(char * s){
	sprintf(usb_send_buffer, "%x\r\n", s );
	CDC_Transmit_FS((uint8_t *)usb_send_buffer, strlen(usb_send_buffer));
}

void USB_receive(){
	uint32_t len = 32;
	CDC_Receive_FS((uint8_t *)usb_receive_buffer, &len);

//	if(strlen(usb_receive_buffer)==2){
////	if(usb_receive_buffer[0]!='\0'){
//		USB_transmit(usb_receive_buffer);
//		strcpy(usb_receive_buffer, "\0");
//	}
}
