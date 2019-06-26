/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DATA61_GPL)
 */

#include <config.h>
#include <machine/capdl.h>

#ifdef CONFIG_DEBUG_BUILD

static void sendPD(word_t address)
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

static void sendPT(word_t address)
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

void doModeCommand(unsigned char c, word_t arg)
{
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

}

#endif
