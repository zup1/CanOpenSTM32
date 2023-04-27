/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Hamed Jafarzadeh 	2022
 * 				Tilen Marjerle		2021
 * 				Janez Paternoster	2020
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include "main.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Determining the CANOpen Driver

#if defined(FDCAN) || defined(FDCAN1) || defined(FDCAN2) || defined(FDCAN3)
#define CO_STM32_FDCAN_Driver 1
#elif defined(CAN) || defined(CAN1) || defined(CAN2) || defined(CAN3)
#define CO_STM32_CAN_Driver 1
#else
#error This STM32 Do not support CAN or FDCAN
#endif

#undef CO_CONFIG_STORAGE_ENABLE // We don't need Storage option, implement based on your use case and remove this line from here

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/**
 * \brief           CAN RX message for platform
 *
 * This is platform specific one
 */
typedef struct {
    uint32_t ident;  /*!< Standard identifier */
    uint8_t dlc;     /*!< Data length */
    uint8_t data[8]; /*!< Received data */
} CO_CANrxMsg_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t*)(msg)))->ident)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t*)(msg)))->dlc)
#define CO_CANrxMsg_readData(msg)  ((uint8_t*)(((CO_CANrxMsg_t*)(msg)))->data)

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void* object;
    void (*CANrx_callback)(void* object, void* message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void* CANptr;
    CO_CANrx_t* rxArray;
    uint16_t rxSize;
    CO_CANtx_t* txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;

    /* STM32 specific features */
    uint32_t primask_send; /* Primask register for interrupts for send operation */
    uint32_t primask_emcy; /* Primask register for interrupts for emergency operation */
    uint32_t primask_od;   /* Primask register for interrupts for send operation */

} CO_CANmodule_t;

/* Data storage object for one entry */
typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    /* Additional variables (target specific) */
    void* addrNV;
} CO_storage_entry_t;

#ifndef portFORCE_INLINE
	#define portFORCE_INLINE inline __attribute__(( always_inline))
#endif

#ifdef __NVIC_PRIO_BITS
	/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		4        /* 15 priority levels */
#endif
/* THE CANBUS AND TIMER INTERRUPT MUST HAS A LOWER PRIORITY THAN THIS!
(lower priorities are higher numeric values.) */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	13

#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#define portSET_INTERRUPT_MASK_FROM_ISR()		ulPortRaiseBASEPRI()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	vPortSetBASEPRI(x)

/*-----------------------------------------------------------*/

portFORCE_INLINE static uint32_t ulPortRaiseBASEPRI( void )
{
uint32_t ulOriginalBASEPRI, ulNewBASEPRI;

	__asm volatile
	(
		"	mrs %0, basepri											\n" \
		"	mov %1, %2												\n"	\
		"	msr basepri, %1											\n" \
		"	isb														\n" \
		"	dsb														\n" \
		:"=r" (ulOriginalBASEPRI), "=r" (ulNewBASEPRI) : "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY )
	);

	/* This return will not be reached but is necessary to prevent compiler
	warnings. */
	return ulOriginalBASEPRI;
}
/*-----------------------------------------------------------*/

portFORCE_INLINE static void vPortSetBASEPRI( uint32_t ulNewMaskValue )
{
	__asm volatile
	(
		"	msr basepri, %0	" :: "r" ( ulNewMaskValue )
	);
}


/* (un)lock critical section in CO_CANsend() */
// Why disabling the whole Interrupt
#define CO_LOCK_CAN_SEND(CAN_MODULE)                                                                                   \
	do {                                                                                                               \
	    CAN_MODULE->primask_send = portSET_INTERRUPT_MASK_FROM_ISR();                                                  \
    } while (0)

#define CO_UNLOCK_CAN_SEND(CAN_MODULE)                                                                                 \
    do {                                                                                                               \
        portCLEAR_INTERRUPT_MASK_FROM_ISR(CAN_MODULE->primask_send);                                                   \
    } while (0)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)                                                                                       \
    do {                                                                                                               \
        CAN_MODULE->primask_emcy = portSET_INTERRUPT_MASK_FROM_ISR();                                                  \
    } while (0)

#define CO_UNLOCK_EMCY(CAN_MODULE)                                                                                     \
    do {                                                                                                               \
        portCLEAR_INTERRUPT_MASK_FROM_ISR(CAN_MODULE->primask_emcy);                                                   \
    } while (0)


/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)                                                                                         \
    do {                                                                                                               \
        CAN_MODULE->primask_od = portSET_INTERRUPT_MASK_FROM_ISR();                                                    \
    } while (0)

#define CO_UNLOCK_OD(CAN_MODULE)                                                                                       \
    do {                                                                                                               \
        portCLEAR_INTERRUPT_MASK_FROM_ISR(CAN_MODULE->primask_od);                                                     \
    } while (0)


/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                                             \
    do {                                                                                                               \
        CO_MemoryBarrier();                                                                                            \
        rxNew = (void*)1L;                                                                                             \
    } while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                                           \
    do {                                                                                                               \
        CO_MemoryBarrier();                                                                                            \
        rxNew = NULL;                                                                                                  \
    } while (0)


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
