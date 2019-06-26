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

static void sendPD(unsigned int address)
{
    word_t i, exists;
    pde_t *start = (pde_t *)address;
    for (i = 0; i < BIT(PD_INDEX_BITS); i++) {
        pde_t pde = start[i];
        exists = 0;
        if (pde_get_pdeType(pde) == pde_pde_coarse && pde_pde_coarse_get_address(pde) != 0) {
            exists = 1;
        } else if (pde_get_pdeType(pde) == pde_pde_section && (pde_pde_section_get_address(pde) != 0 ||
#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT
                                                               pde_pde_section_get_HAP(pde))) {
#else
                                                               pde_pde_section_get_AP(pde))) {
#endif
            exists = 1;
        }
        if (exists != 0 && i < kernelBase >> pageBitsForSize(ARMSection)) {
            sendWord(i);
            sendWord(pde.words[0]);
        }
    }
}

static void sendPT(unsigned int address)
{
    word_t i, exists;
    pte_t *start = (pte_t *)address;
    for (i = 0; i < BIT(PT_INDEX_BITS); i++) {
        pte_t pte = start[i];
        exists = 0;
#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT
        if (pte_get_pteType(pte) == pte_pte_small && (pte_pte_small_get_address(pte) != 0 ||
                                                      pte_pte_small_get_HAP(pte))) {
            exists = 1;
        }
#else
        if (pte_get_pteType(pte) == pte_pte_large && (pte_pte_large_get_address(pte) != 0 ||
                                                      pte_pte_large_get_AP(pte))) {
            exists = 1;
        } else if (pte_get_pteType(pte) == pte_pte_small && (pte_pte_small_get_address(pte) != 0 ||
                                                             pte_pte_small_get_AP(pte))) {
            exists = 1;
        }
#endif
        if (exists != 0) {
            sendWord(i);
            sendWord(pte.words[0]);
        }
    }
}

static void sendASIDPool(unsigned int address)
{
    word_t i;
    pde_t **start = (pde_t **)address;
    for (i = 0; i < BIT(ASID_POOL_INDEX_BITS); i++) {
        pde_t *pde = start[i];
        if (pde != 0) {
            sendWord(i);
            sendWord((unsigned int)pde);
        }
    }
}

int doArchCommand(unsigned char c, word_t arg)
{
    switch (c) {
    case PD_COMMAND:
        /*pgdir */
        sendPD(arg);
        putDebugChar(END);
        break;
    case PT_COMMAND:
        /*pg table */
        sendPT(arg);
        putDebugChar(END);
        break;
    case ASID_POOL_COMMAND:
        /*asid pool */
        sendASIDPool(arg);
        putDebugChar(END);
        break;
    default:
        return 0;
    }

    return 1;
}

#endif
