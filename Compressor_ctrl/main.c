/*
 * Compressor_ctrl.c
 *
 * Created: 12.03.2021 11:10:10
 * Author : Legkiy
 */ 

#include "common.h"

FUSES = 
{
	.low = (FUSE_CKSEL0 & FUSE_CKSEL1 & FUSE_CKSEL3 & FUSE_SUT0 & FUSE_CKDIV8 & FUSE_CKOUT),
	//.low = (FUSE_CKSEL1), //choose external 4mhz crystal 64ms startup
	//.low = (FUSE_SUT0 & FUSE_CKSEL3 & FUSE_CKSEL0 & FUSE_CKOUT), //internal 128khz 64ms startup !!dont DIV that more than 2!!
	.high = HFUSE_DEFAULT,	//SPIEn
	.extended = EFUSE_DEFAULT,
};

LOCKBITS = LOCKBITS_DEFAULT;

int main(void)
{
	hw_init();
    /* Replace with your application code */
    while (1) 
    {
		pin_routine();
		sleep_mode();
    }
}

