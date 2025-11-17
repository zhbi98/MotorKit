/**
 * @file shell_qq.h
 *
 */

#ifndef __SHELL_QQ_H__
#define __SHELL_QQ_H__

#ifdef __cplusplus
    extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

#define NODE_SIZE 512
#define NODE_NUM  256

/*Dummy type to make handling easier*/
typedef uint8_t shell_qq_node_t;

typedef struct {
    uint8_t phead;
    uint8_t ptail;
    shell_qq_node_t nodes[NODE_SIZE];
} shell_qq_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void _shell_qq_init(shell_qq_t * qq_p);
uint8_t _shell_qq_is_empty(shell_qq_t * qq_p);
uint8_t _shell_qq_is_full(shell_qq_t * qq_p);
uint8_t _shell_qq_ins(shell_qq_t * qq_p, shell_qq_node_t * node_p);
uint8_t _shell_qq_remove(shell_qq_t * qq_p, shell_qq_node_t * node_p);
uint8_t _shell_qq_clear(shell_qq_t * qq_p);
uint32_t _shell_qq_get_len(shell_qq_t * qq_p);

#ifdef __cplusplus
    }
#endif /*__cplusplus*/

#endif /*NT_QQ_H*/
