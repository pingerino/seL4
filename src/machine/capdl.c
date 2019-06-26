/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(GD_GPL)
 */


#include <config.h>
#include <object/structures.h>
#include <object/tcb.h>
#include <model/statedata.h>
#include <machine/capdl.h>
#include <arch/machine/capdl.h>
#include <machine/io.h>
#include <plat/machine/hardware.h>

#ifdef CONFIG_DEBUG_BUILD

static int getDecodedChar(unsigned char *result)
{
    unsigned char c;
    c = getDebugChar();
    if (c == START) {
        return 1;
    }
    if (c == ESCAPE) {
        c = getDebugChar();
        if (c == START) {
            return 1;
        }
        switch (c) {
        case ESCAPE_ESCAPE:
            *result = ESCAPE;
            break;
        case START_ESCAPE:
            *result = START;
            break;
        case END_ESCAPE:
            *result = END;
            break;
        default:
            if (c >= 20 && c < 40) {
                *result = c - 20;
            }
        }
        return 0;
    } else {
        *result = c;
        return 0;
    }
}

static void putEncodedChar(unsigned char c)
{
    switch (c) {
    case ESCAPE:
        putDebugChar(ESCAPE);
        putDebugChar(ESCAPE_ESCAPE);
        break;
    case START:
        putDebugChar(ESCAPE);
        putDebugChar(START_ESCAPE);
        break;
    case END:
        putDebugChar(ESCAPE);
        putDebugChar(END_ESCAPE);
        break;
    default:
        if (c < 20) {
            putDebugChar(ESCAPE);
            putDebugChar(c + 20);
        } else {
            putDebugChar(c);
        }
    }
}

int getWord(word_t *res)
{
    for (word_t i = 0; i < seL4_WordBits; i+=8) {
        unsigned char byte;
        if (getDecodedChar(&byte)) {
            return 1;
        }
        *res = ((*res) << 8u) + byte;
    }
    return 0;
}

void sendWord(word_t word)
{
    for (word_t i = 0; i < seL4_WordBits; i += 8) {
        putEncodedChar(word & 0xff);
        word >>= 8u;
    }
}

static cte_t *getMDBParent(cte_t *slot)
{
    cte_t *oldSlot = CTE_PTR(mdb_node_get_mdbPrev(slot->cteMDBNode));

    while (oldSlot != 0 && !isMDBParentOf(oldSlot, slot)) {
        oldSlot = CTE_PTR(mdb_node_get_mdbPrev(oldSlot->cteMDBNode));
    }

    return oldSlot;
}

static void sendRunqueues(void)
{
    for (word_t i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        for (tcb_t *curr = NODE_STATE_ON_CORE(ksDebugTCBs, i); curr != NULL; curr = curr->tcbDebugNext) {
            thread_state_t *state = &curr->tcbState;
            if (thread_state_ptr_get_tsType(state) != ThreadState_IdleThreadState &&
                thread_state_ptr_get_tsType(state) != ThreadState_Inactive) {
                sendWord((word_t)curr);
            }
        }
    }
}

static void sendEPQueue(word_t epptr)
{
    tcb_t *current = (tcb_t *)endpoint_ptr_get_epQueue_head((endpoint_t *)epptr);
    for (; current != NULL; current = current->tcbEPNext) {
        sendWord((word_t)current);
    }
}

static void sendCNode(word_t address, word_t sizebits)
{
    word_t i;
    cte_t *start = (cte_t *)address;
    for (i = 0; i < (1 << sizebits); i++) {
        cap_t cap = start[i].cap;
        if (cap_get_capType(cap) != cap_null_cap) {
            cte_t *parent = getMDBParent(&start[i]);
            sendWord(i);
            for (word_t i = 0; i < ARRAY_SIZE(cap.words); i++) {
                sendWord(cap.words[i]);
            }
            sendWord((word_t)parent);
        }
    }
}

static void sendIRQNode(void)
{
    sendCNode((word_t)intStateIRQNode, IRQ_CNODE_SLOT_BITS);
}

static void sendVersion(void)
{
    sendWord(ARCH);
    sendWord(CAPDL_VERSION);
}

void capDL(void)
{
    int result;
    int done = 0;
    while (done == 0) {
        unsigned char c;
        do {
            c = getDebugChar();
        } while (c != START);
        do {
            result = getDecodedChar(&c);
            if (result) {
                continue;
            }
            switch (c) {
            case RQ_COMMAND: {
                /*runqueues */
                sendRunqueues();
                putDebugChar(END);
                result = 0;
            }
            break;
            case EP_COMMAND: {
                /*endpoint waiters */
                word_t arg;
                result = getWord(&arg);
                if (result) {
                    continue;
                }
                sendEPQueue(arg);
                putDebugChar(END);
            }
            break;
            case CN_COMMAND: {
                /*cnode */
                word_t address, sizebits;
                result = getWord(&address);
                if (result) {
                    continue;
                }
                result = getWord(&sizebits);
                if (result) {
                    continue;
                }

                sendCNode(address, sizebits);
                putDebugChar(END);
            }
            case TCB_COMMAND: {
                /*cnode */
                word_t address, sizebits;
                result = getWord(&address);
                if (result) {
                    continue;
                }
                result = getWord(&sizebits);
                if (result) {
                    continue;
                }

                sendCNode((word_t)TCB_PTR_CTE_PTR(address, 0), sizebits);
                putDebugChar(END);
            }
            break;
            case IRQ_COMMAND: {
                sendIRQNode();
                putDebugChar(END);
                result = 0;
            }
            break;
            case VERSION_COMMAND: {
                sendVersion();
                putDebugChar(END);
            }
            break;
            case DONE: {
                done = 1;
                putDebugChar(END);
            }
            default: {
                word_t arg;
                result = getWord(&arg);
                if (result) {
                    continue;
                }
                result = doArchCommand(c, arg);
                break;
            }
            }
        } while (result);
    }
}

#endif
