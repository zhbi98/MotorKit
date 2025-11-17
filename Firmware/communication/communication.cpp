
/* Includes ------------------------------------------------------------------*/

#include "communication.h"
#include "odrive_main.h"
#include "freertos_vars.h"
#include "utils.hpp"

#include <cmsis_os.h>
#include <memory>
//#include <usbd_cdc_if.h>
//#include <usb_device.h>
//#include <usart.h>
#include "shell_device.h"
#include <gpio.h>

#include <type_traits>

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

uint64_t serial_number;
char serial_number_str[13]; // 12 digits + null termination

/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

void init_communication(void) {
    //printf("hi!\r\n");

    // Dual UART operation not supported yet
    /*if (odrv.config_.enable_uart_a && odrv.config_.enable_uart_b) {
        odrv.misconfigured_ = true;
    }*/

    /*if (odrv.config_.enable_uart_a && uart_a) {
        start_uart_server(uart_a);
    } else if (odrv.config_.enable_uart_b && uart_b) {
        start_uart_server(uart_b);
    }*/

    /*start_usb_server();*/
    _shell_device_init();

    /*if (odrv.config_.enable_i2c_a) {
        start_i2c_server();
    }

    if (odrv.config_.enable_can_a) {
        odrv.can_.start_server(&hcan1);
    }*/
}

extern "C" {
int _write(int file, const char* data, int len) __attribute__((used));
}

// @brief This is what printf calls internally
int _write(int file, const char* data, int len) {
    return len; // Always pretend that we processed everything
}

ODrive& ep_root = odrv;
