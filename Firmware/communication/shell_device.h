/**
 * @file shell_device.h
 *
 */

#ifndef __SHELL_DEVICE_H__
#define __SHELL_DEVICE_H__

#ifdef __cplusplus
    extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include <stdint.h>
#include "shell.h"

/**********************
 *      TYPEDEFS
 **********************/

/**Provides USB CDC-based serial communication interface for shell,
 *including read/write operations and thread synchronization using FreeRTOS semaphores.*/

/**********************
 *  STATIC VARIABLES
 **********************/

/**
 * Write data to shell output (USB CDC IN endpoint)
 * Sends data through USB CDC interface (Host receive direction).
 * Implements shell device write interface required by shell.h
 * @param data_p Pointer to data buffer to transmit
 * @param len Number of bytes to transmit
 * @return int16_t Number of bytes transmitted (or 0 on failure)
 */
int16_t shell_device_write(int8_t * data_p, 
    uint16_t len);

/**
 * Read data from shell input (USB CDC OUT endpoint)
 * Receives data through USB CDC interface (Host transmit direction).
 * Implements shell device read interface required by shell.h
 * @param data_p Buffer to store received data
 * @param len Maximum bytes to read
 * @return int16_t Number of bytes received (to be implemented)
 */
int16_t shell_device_read(
    int8_t * data_p, uint16_t len);

/**
 * Read data from shell input (USB CDC OUT endpoint)
 * Receives data through USB CDC interface (Host transmit direction).
 * Implements shell device read interface required by shell.h
 * @param data_p Buffer to store received data
 * @param len Maximum bytes to read
 * @return int16_t Number of bytes received (to be implemented)
 */
int16_t shell_qq_readusb(
    int8_t * data_p, uint16_t len);

/**
 * Acquire exclusive access to shell device
 * Implements recursive mutex locking for thread-safe shell operations.
 * Required for concurrent access protection in multi-tasking environment.
 * @param shell Pointer to shell instance (unused)
 * @return int32_t 0 on success (FreeRTOS semaphore never times out)
 */
int32_t shell_device_lock(Shell * shell);

/**
 * Release exclusive access to shell device
 * Implements recursive mutex unlocking for thread-safe shell operations.
 * Must be called after corresponding shell_device_lock()
 * @param shell Pointer to shell instance (unused)
 * @return int32_t 0 on success
 */
int32_t shell_device_unlock(Shell *shell);

/**
 * Retrieves the initialized shell device instanceReturns the pre-initialized 
 * global shell singleton that was set up during system startup.
 * @return Shell* Pointer to the global shell instance (never NULL)
 */
Shell * shell_device_get_object();

/**
 * Initialize shell device and communication interface
 * Creates synchronization primitives, initializes USB CDC communication,
 * and binds device operations to shell instance. Call once at system startup.
 */
void _shell_device_init(void);

#ifdef __cplusplus
    }
#endif /*__cplusplus*/

#endif /*__SHELL_DEVICE_H__*/
