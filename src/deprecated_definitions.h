/*
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

	***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
	***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
	the FAQ page "My application does not run, what could be wrong?".  Have you
	defined configASSERT()?

	http://www.FreeRTOS.org/support - In return for receiving this top quality
	embedded software for free we request you assist our global community by
	participating in the support forum.

	http://www.FreeRTOS.org/training - Investing in training allows your team to
	be as productive as possible as early as possible.  Now you can receive
	FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
	Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

#ifndef DEPRECATED_DEFINITIONS_H
#define DEPRECATED_DEFINITIONS_H


/* Each FreeRTOS port has a unique portmacro.h header file.  Originally a
pre-processor definition was used to ensure the pre-processor found the correct
portmacro.h file for the port being used.  That scheme was deprecated in favour
of setting the compiler's include path such that it found the correct
portmacro.h file - removing the need for the constant and allowing the
portmacro.h file to be located anywhere in relation to the port being used.  The
definitions below remain in the code for backward compatibility only.  New
projects should not use them. */

#ifdef OPEN_WATCOM_INDUSTRIAL_PC_PORT
	#include "portmacro.h"
	typedef void ( __interrupt __far *pxISR )();
#endif

#ifdef OPEN_WATCOM_FLASH_LITE_186_PORT
	#include "portmacro.h"
	typedef void ( __interrupt __far *pxISR )();
#endif

#ifdef GCC_MEGA_AVR
	#include "portmacro.h"
#endif

#ifdef IAR_MEGA_AVR
	#include "portmacro.h"
#endif

#ifdef MPLAB_PIC24_PORT
	#include "portmacro.h"
#endif

#ifdef MPLAB_DSPIC_PORT
	#include "portmacro.h"
#endif

#ifdef MPLAB_PIC18F_PORT
	#include "portmacro.h"
#endif

#ifdef MPLAB_PIC32MX_PORT
	#include "portmacro.h"
#endif

#ifdef _FEDPICC
	#include "portmacro.h"
#endif

#ifdef SDCC_CYGNAL
	#include "portmacro.h"
#endif

#ifdef GCC_ARM7
	#include "portmacro.h"
#endif

#ifdef GCC_ARM7_ECLIPSE
	#include "portmacro.h"
#endif

#ifdef ROWLEY_LPC23xx
	#include "portmacro.h"
#endif

#ifdef IAR_MSP430
	#include "portmacro.h"
#endif

#ifdef GCC_MSP430
	#include "portmacro.h"
#endif

#ifdef ROWLEY_MSP430
	#include "portmacro.h"
#endif

#ifdef ARM7_LPC21xx_KEIL_RVDS
	#include "portmacro.h"
#endif

#ifdef SAM7_GCC
	#include "portmacro.h"
#endif

#ifdef SAM7_IAR
	#include "portmacro.h"
#endif

#ifdef SAM9XE_IAR
	#include "portmacro.h"
#endif

#ifdef LPC2000_IAR
	#include "portmacro.h"
#endif

#ifdef STR71X_IAR
	#include "portmacro.h"
#endif

#ifdef STR75X_IAR
	#include "portmacro.h"
#endif

#ifdef STR75X_GCC
	#include "portmacro.h"
#endif

#ifdef STR91X_IAR
	#include "portmacro.h"
#endif

#ifdef GCC_H8S
	#include "portmacro.h"
#endif

#ifdef GCC_AT91FR40008
	#include "portmacro.h"
#endif

#ifdef RVDS_ARMCM3_LM3S102
	#include "portmacro.h"
#endif

#ifdef GCC_ARMCM3_LM3S102
	#include "portmacro.h"
#endif

#ifdef GCC_ARMCM3
	#include "portmacro.h"
#endif

#ifdef IAR_ARM_CM3
	#include "portmacro.h"
#endif

#ifdef IAR_ARMCM3_LM
	#include "portmacro.h"
#endif

#ifdef HCS12_CODE_WARRIOR
	#include "portmacro.h"
#endif

#ifdef MICROBLAZE_GCC
	#include "portmacro.h"
#endif

#ifdef TERN_EE
	#include "portmacro.h"
#endif

#ifdef GCC_HCS12
	#include "portmacro.h"
#endif

#ifdef GCC_MCF5235
    #include "portmacro.h"
#endif

#ifdef COLDFIRE_V2_GCC
	#include "portmacro.h"
#endif

#ifdef COLDFIRE_V2_CODEWARRIOR
	#include "portmacro.h"
#endif

#ifdef GCC_PPC405
	#include "portmacro.h"
#endif

#ifdef GCC_PPC440
	#include "portmacro.h"
#endif

#ifdef _16FX_SOFTUNE
	#include "portmacro.h"
#endif

#ifdef BCC_INDUSTRIAL_PC_PORT
	/* A short file name has to be used in place of the normal
	FreeRTOSConfig.h when using the Borland compiler. */
	#include "frconfig.h"
	#include "prtmacro.h"
    typedef void ( __interrupt __far *pxISR )();
#endif

#ifdef BCC_FLASH_LITE_186_PORT
	/* A short file name has to be used in place of the normal
	FreeRTOSConfig.h when using the Borland compiler. */
	#include "frconfig.h"
	#include "prtmacro.h"
    typedef void ( __interrupt __far *pxISR )();
#endif

#ifdef __GNUC__
   #ifdef __AVR32_AVR32A__
	   #include "portmacro.h"
   #endif
#endif

#ifdef __ICCAVR32__
   #ifdef __CORE__
      #if __CORE__ == __AVR32A__
	      #include "portmacro.h"
      #endif
   #endif
#endif

#ifdef __91467D
	#include "portmacro.h"
#endif

#ifdef __96340
	#include "portmacro.h"
#endif


#ifdef __IAR_V850ES_Fx3__
	#include "portmacro.h"
#endif

#ifdef __IAR_V850ES_Jx3__
	#include "portmacro.h"
#endif

#ifdef __IAR_V850ES_Jx3_L__
	#include "portmacro.h"
#endif

#ifdef __IAR_V850ES_Jx2__
	#include "portmacro.h"
#endif

#ifdef __IAR_V850ES_Hx2__
	#include "portmacro.h"
#endif

#ifdef __IAR_78K0R_Kx3__
	#include "portmacro.h"
#endif

#ifdef __IAR_78K0R_Kx3L__
	#include "portmacro.h"
#endif

#endif /* DEPRECATED_DEFINITIONS_H */

