/**
 * @file shell_device.c
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "shell_device.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "shell.h"
#include "shell_qq.h"
#include "log.h"

/*********************
 *      DEFINES
 *********************/

#define TRANSMIT_MAX 48U

/**********************
 *      TYPEDEFS
 **********************/

/**Provides USB CDC-based serial communication interface for shell,
 *including read/write operations and thread synchronization using FreeRTOS semaphores.*/

/**********************
 *  STATIC VARIABLES
 **********************/

static osMutexDef_t shellmutex = {0};
static osMutexId shellmutexid; /* Mutex for shell operation synchronization */
static shell_qq_t usbshellqq = {0}; /*Shell data retrieval queue*/
static int8_t buffer[512] = {0}; /* Shell command processing buffer */
static Shell shell = {0};        /* Shell instance structure */
static Log _log = {0}; /* Shell log structure */

/**
 * Write data to shell output (USB CDC IN endpoint)
 * Sends data through USB CDC interface (Host receive direction).
 * Implements shell device write interface required by shell.h
 * @param data_p Pointer to data buffer to transmit
 * @param len Number of bytes to transmit
 * @return int16_t Number of bytes transmitted (or 0 on failure)
 */
int16_t shell_device_write(int8_t * data_p, uint16_t len)
{
    uint8_t result = USBD_OK;
    uint16_t remain = len;
    uint16_t block = 0;
    uint16_t sent = 0;

    while (remain > 0) { /*剩余长度*/
        block = (remain > TRANSMIT_MAX) ? TRANSMIT_MAX : remain;
        result = CDC_Transmit_FS((uint8_t*)data_p + sent, 
            block, CDC_IN_EP);

        /**The amount of data transferred in a single 
        USB transaction is limited (usually only 
        handling 64-byte RRB packets). */
        sent += block;
        remain -= block;

        if (result != USBD_OK) return sent;

        osDelay(1);
    }

    /* USB IN endpoint: Device-to-Host transmission (Host receives data) */
    /*USB 的 EP_IN，EP_OUT 都是基于 HOST 设备作为参考的的，
    对于 Device 而言 IN 指的是发送*/
    /*result = CDC_Transmit_FS(data_p, 
        len, CDC_IN_EP);*/

    /*Wait for delivery, otherwise the 
    content will not be delivered*/
    /*osDelay(1);*/

    /*Alternative serial debug output (commented out)*/
    /**serialTransmit(&debugSerial, (uint8_t *)data, len, 0x1FF);*/
    if (result != USBD_OK) {
        osDelay(1); return 0;}
    return len;
}

bool data_pending = false;

/**
 * Read data from shell input (USB CDC OUT endpoint)
 * Receives data through USB CDC interface (Host transmit direction).
 * Implements shell device read interface required by shell.h
 * @param data_p Buffer to store received data
 * @param len Maximum bytes to read
 * @return int16_t Number of bytes received (to be implemented)
 */
int16_t shell_device_read(
    int8_t * data_p, uint16_t len)
{
    bool res = false;

    res = _shell_qq_remove(&usbshellqq, 
        (int8_t *)data_p);
    /*TODO: Implement actual receive logic using CDC_Receive_FS callback*/
    /**serialReceive(&debugSerial, (uint8_t *)data_p, len, 0);*/
    if (res) return 1;
    else data_pending = false;
    return 0;
}

/**
 * Read data from shell input (USB CDC OUT endpoint)
 * Receives data through USB CDC interface (Host transmit direction).
 * Implements shell device read interface required by shell.h
 * @param data_p Buffer to store received data
 * @param len Maximum bytes to read
 * @return int16_t Number of bytes received (to be implemented)
 */
int16_t shell_qq_readusb(
    int8_t * data_p, uint16_t len)
{
    uint16_t i = 0;
    for (i = 0; i < len; i++) {
        _shell_qq_ins(&usbshellqq, 
            (int8_t *)&data_p[i]);
    }

    data_pending = true;
    return len;
}

/**
 * Acquire exclusive access to shell device
 * Implements recursive mutex locking for thread-safe shell operations.
 * Required for concurrent access protection in multi-tasking environment.
 * @param shell Pointer to shell instance (unused)
 * @return int32_t 0 on success (FreeRTOS semaphore never times out)
 */
int32_t shell_device_lock(Shell * shell)
{
    osRecursiveMutexWait(
        shellmutexid, portMAX_DELAY);
    return 0;
}

/**
 * Release exclusive access to shell device
 * Implements recursive mutex unlocking for thread-safe shell operations.
 * Must be called after corresponding shell_device_lock()
 * @param shell Pointer to shell instance (unused)
 * @return int32_t 0 on success
 */
int32_t shell_device_unlock(Shell *shell)
{
    osRecursiveMutexRelease(shellmutexid);
    return 0;
}

/**
 * Retrieves the initialized shell device instanceReturns the pre-initialized 
 * global shell singleton that was set up during system startup.
 * @return Shell* Pointer to the global shell instance (never NULL)
 */
Shell * shell_device_get_object()
{
    return &shell;
}

/**
 * Initialize shell device and communication interface
 * Creates synchronization primitives, initializes USB CDC communication,
 * and binds device operations to shell instance. Call once at system startup.
 */
void _shell_device_init(void)
{
    shellmutexid = osMutexCreate(&shellmutex); /* Create synchronization mutex */
    _shell_qq_init(&usbshellqq); /* Create Read the command queue */

    /**Bind device operations to shell instance*/
    shell.write = shell_device_write;
    shell.read = shell_device_read;
    shell.lock = shell_device_lock;
    shell.unlock = shell_device_unlock;
    /**Initialize shell with 512-byte command buffer*/
    shellInit(&shell, buffer, 512);

    _log.write = shell_device_write; /*Because log and the shell share a single serial port*/
    _log.active = true;
    _log.level = LOG_DEBUG;
    logRegister(&_log, &shell);
}
