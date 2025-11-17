/**
 * @file shell_qq.c
 *
 */

#include "shell_qq.h"


/**
 * Initialize qq
 * @param qq_p pointer to shell_qq_t variable
 * @param node_size the size of 1 node in bytes
 */
void _shell_qq_init(shell_qq_t * qq_p)
{
    uint8_t node_size = sizeof(shell_qq_node_t);
    memset(qq_p->nodes, 0, node_size * NODE_NUM);
    qq_p->phead = 0;
    qq_p->ptail = 0;
}

/**
 * Check qq if the contents are empty
 * @param qq_p pointer to shell_qq_t variable
 * @param node_size the size of 1 node in bytes
 */
uint8_t _shell_qq_is_empty(shell_qq_t * qq_p)
{
    if (qq_p->phead == qq_p->ptail) return true;
    else return false;
}

/**
 * Check qq if the contents are full
 * @param qq_p pointer to shell_qq_t variable
 * @param node_size the size of 1 node in bytes
 */
uint8_t _shell_qq_is_full(shell_qq_t * qq_p)
{
    if (((qq_p->ptail + 1) % NODE_NUM) == qq_p->phead)
        return true;
    else return false;
}

/**
 * Add a new head to a qq
 * @param qq_p pointer to qq
 * @return pointer to the new head
 */
uint8_t _shell_qq_ins(shell_qq_t * qq_p, 
    shell_qq_node_t * node_p)
{
    uint8_t node_size = sizeof(shell_qq_node_t);
    if (_shell_qq_is_full(qq_p)) return false;

    memcpy(&qq_p->nodes[qq_p->ptail], node_p, 
        node_size);

    qq_p->ptail += 1;
    qq_p->ptail = qq_p->ptail % NODE_NUM;
    return true;
}

/**
 * Remove the node 'node_p' from 'qq_p' qq list.
 * It does not free the memory of node.
 * @param qq_p pointer to the qq of 'node_p'
 * @param node_p pointer to node in 'qq_p' qq
 */
uint8_t _shell_qq_remove(shell_qq_t * qq_p, 
    shell_qq_node_t * node_p)
{
    uint8_t node_size = sizeof(shell_qq_node_t);
    if (_shell_qq_is_empty(qq_p)) return false;

    memcpy(node_p, &qq_p->nodes[qq_p->phead], node_size);

    qq_p->phead += 1;
    qq_p->phead = qq_p->phead % NODE_NUM;
    return true;
}

/**
 * Remove and free all elements from a qq. The qq remain valid but become empty.
 * @param qq_p pointer to qq
 */
uint8_t _shell_qq_clear(shell_qq_t * qq_p)
{
    uint8_t node_size = sizeof(shell_qq_node_t);
    if (_shell_qq_is_empty(qq_p)) return true;
    memset(qq_p->nodes, 0, NODE_NUM * node_size);
    qq_p->phead = 0;
    qq_p->ptail = 0;
    return true;
}

/**
 * Return the length of the qq.
 * @param qq_p pointer to qq
 * @return length of the qq
 */
uint32_t _shell_qq_get_len(shell_qq_t * qq_p)
{
    if (_shell_qq_is_empty(qq_p)) return 0;
    return (qq_p->ptail - qq_p->phead + NODE_NUM) % NODE_NUM;
}
