#include "odrive_can.hpp"

#include <can.h>
#include <cmsis_os.h>

#include "freertos_vars.h"
#include "utils.hpp"

// Safer context handling via maps instead of arrays
// #include <unordered_map>
// std::unordered_map<CAN_HandleTypeDef *, ODriveCAN *> ctxMap;


bool ODriveCAN::apply_config()
{
    return true;
}

bool ODriveCAN::reinit() 
{
    return true;
}

bool ODriveCAN::start_server(CAN_HandleTypeDef* handle) 
{
    return true;
}

void ODriveCAN::can_server_thread() 
{
    for (;;) {

    }
}

// Set one of only a few common baud rates.  CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
// 21 TQ allows for easy sampling at exactly 80% (recommended by Vector Informatik GmbH for high reliability systems)
// Conveniently, the CAN peripheral's 42MHz clock lets us easily create 21TQs for all common baud rates
bool ODriveCAN::set_baud_rate(uint32_t baud_rate) 
{
    return true;
}

void ODriveCAN::process_rx_fifo(uint32_t fifo) 
{
}

// Send a CAN message on the bus
bool ODriveCAN::send_message() 
{
    return true;
}

//void ODriveCAN::set_error(Error error) {
//    error_ |= error;
//}

bool ODriveCAN::subscribe() 
{
    return true;
}

bool ODriveCAN::unsubscribe()
{
    return true;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    //HAL_CAN_ResetError(hcan);
}
