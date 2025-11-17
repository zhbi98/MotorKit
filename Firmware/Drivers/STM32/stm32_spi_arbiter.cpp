
#include "stm32_spi_arbiter.hpp"
#include "stm32_system.h"
#include "utils.hpp"
#include <cmsis_os.h>

bool Stm32SpiArbiter::acquire_task(SpiTask* task) {
    return !__atomic_exchange_n(&task->is_in_use, 
        true, __ATOMIC_SEQ_CST);
}

void Stm32SpiArbiter::release_task(SpiTask* task) {
    task->is_in_use = false;
}

bool Stm32SpiArbiter::start() {
    if (!task_list_) return false;

    SpiTask& task = *task_list_;
    task.ncs_gpio.write(false);
    
    HAL_StatusTypeDef status = HAL_ERROR;

    if (hspi_->hdmatx->State != HAL_DMA_STATE_READY || 
        hspi_->hdmarx->State != HAL_DMA_STATE_READY) {
        /*This can happen if the DMA or interrupt priorities 
        are not configured properly.*/
        status = HAL_BUSY;
    } else if (task.tx_buf && task.rx_buf) {
        status = HAL_SPI_TransmitReceive_DMA(hspi_, 
            (uint8_t*)task.tx_buf, task.rx_buf, task.length);
    } else if (task.tx_buf) {
        status = HAL_SPI_Transmit_DMA(hspi_, 
            (uint8_t*)task.tx_buf, task.length);
    } else if (task.rx_buf) {
        status = HAL_SPI_Receive_DMA(hspi_, 
            task.rx_buf, task.length);
    }

    if (status != HAL_OK) {
        task.ncs_gpio.write(true);
    }

    return status == HAL_OK;
}

/**异步执行，其他线程/上下文可以调用 transfer_async() 把更多任务追加到链表*/
void Stm32SpiArbiter::transfer_async(SpiTask* task) {
    /**当前任务还未添加到任务链表所以是处于独立的，
    所以任务的下一个节点是空的，下一个新的任务才会接入到该指针中*/
    task->next = nullptr;
    
    // Append new task to task list.
    // We could try to do this lock free but we could 
    // also use our time for useful things.
    
    /**将 ptr 指针指向任务链表的首地址*/
    SpiTask** ptr = &task_list_;
    CRITICAL_SECTION() {
        
        /**遍历任务链表，如果某个节点的下一个节点为 nllptr 
        则说明找到上一个任务预留的 next 接入点，当前的任务就连接到该节点中*/
        while (*ptr)
            ptr = &(*ptr)->next;

        /*将当前任务连接到上一个任务节点的 next 中*/
        *ptr = task;
    }

    // If the list was empty before, kick off the SPI arbiter now

    /**如果 ptr 指针指向任务链表的首地址，说明任务链表是空的，
     *不需要排队可以直接启动发送操作，如果任务链表非空，
     *则只把待发送数据以任务节点的方式放入链表中，等待排队处理。*/
    if (ptr == &task_list_) {
        if (!start()) {
            if (task->end_callback) {
                (*task->end_callback)(task->parm, false);
            }
        }
    }
}

/**在内部自行创建一个临时 task 对象，并且增加状态同步等待，封装异步发送来实现同步发送*/
// TODO: this currently only works when called in a CMSIS thread.
bool Stm32SpiArbiter::transfer(Stm32Gpio ncs_gpio, const uint8_t* tx_buf, 
    uint8_t* rx_buf, size_t length, uint32_t timeout_ms) {
    volatile uint8_t result = 0xff;

    SpiTask task = {
        .ncs_gpio = ncs_gpio,
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .length = length,
        .end_callback = [](void* ctx, bool success) {
            *(volatile uint8_t*)ctx = success ? 1 : 0;
        },
        .parm = (void*)&result,
        .is_in_use = false,
        .next = nullptr
    };

    /**要求：传入的 SpiTask 实例在该传输完成（回调执行）之前必须保持有效。
    因为 transfer_async 是异步的，若调用者把临时栈对象入队但在完成前销毁，
    会造成未定义行为。*/

    transfer_async(&task);

    /**局部 SpiTask 是安全的，因为 while(result == 0xFF)
    会阻塞等待完成（所以 task 在等待期间有效，不会提前销毁）。*/

    /**result 正常情况下表示发送成功或失败，所以值一般为 false/true，
    如果依旧为初始值 0xFF 则说明当前发送任务还没有完成，在此等待完成*/
    while (result == 0xff) {
        osDelay(1); // TODO: honor timeout
    }

    /**
     * 每次发送都要等待 result != 0xff 那链表中是否永远只有一个 task 对象?
     * (1) transfer 函数会把栈上任务入队并阻塞等待完成 (result != 0xFF)，
     * 在此期间其他线程可以调用 transfer_async() 把更多任务追加到链表，
     * 因此链表可以有多个节点。
     * (2) 如果只一个线程且只用 transfer 同步调用，没有其它并发的 transfer
     * 或 transfer_async 调用，那么链表在任意时刻确实只会有那个一个栈上任务。
     */

    return result;
}

/**数据发送完成时调用，可以将该函数放置到发送完成中断中调用*/
void Stm32SpiArbiter::on_complete() {
    if (!task_list_) {
        return; // this should not happen
    }

    // Wrap up transfer
    task_list_->ncs_gpio.write(true);
    if (task_list_->end_callback) {
        (*task_list_->end_callback)(task_list_->parm, true);
    }

    // Start next task if any
    /*获取下一个新的发送任务*/
    SpiTask* next = nullptr;
    CRITICAL_SECTION() {
        next = task_list_ = task_list_->next;
    }
    /*当前任务完成，启动一个新的发送任务*/
    if (next) {
        start();
    }
}
