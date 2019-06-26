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

static void sendASIDPool(word_t address)
{
    word_t i;
    pde_t **start = (pde_t **)address;
    for (i = 0; i < BIT(ASID_POOL_INDEX_BITS); i++) {
        pde_t *pde = start[i];
        if (pde != 0) {
            sendWord(i);
            sendWord((word_t)pde);
        }
    }
}

int doArchCommand(unsigned char c, word_t arg)
{
    switch (c) {
    case ASID_POOL_COMMAND:
        /*asid pool */
        sendASIDPool(arg);
        putDebugChar(END);
        break;
    default:
        return doModeCommand(c, arg);
    }

    return 1;
}

#endif
