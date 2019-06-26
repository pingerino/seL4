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

#ifdef CONFIG_DEBUG_BUILD

static void sendPD(unsigned long address)
{
    unsigned long i;
    unsigned int exists;
    pde_t *start = (pde_t *)address;
    for (i = 0; i < BIT(PD_INDEX_BITS); i++) {
        pde_t pde = start[i];
        exists = 1;
        if (pde_get_page_size(pde) == pde_pde_pt && (pde_pde_pt_get_pt_base_address(pde) == 0 ||
                                                     !pde_pde_pt_get_present(pde) || !pde_pde_pt_get_super_user(pde))) {
            exists = 0;
        } else if (pde_get_page_size(pde) == pde_pde_large && (pde_pde_large_get_page_base_address(pde) == 0 ||
                                                               !pde_pde_large_get_present(pde) || !pde_pde_large_get_super_user(pde))) {
            exists = 0;
        }
        if (exists != 0 && i < PPTR_BASE >> pageBitsForSize(X86_LargePage)) {
            sendWord(i);
            sendWord(pde.words[0]);
        }
    }
}

static void sendPT(unsigned long address)
{
    unsigned long i;
    pte_t *start = (pte_t *)address;
    for (i = 0; i < BIT(PT_INDEX_BITS); i++) {
        pte_t pte = start[i];
        if (pte_get_page_base_address(pte) != 0 && pte_get_present(pte) && pte_get_super_user(pte)) {
            sendWord(i);
            sendWord(pte.words[0]);
        }
    }
}

static void sendASIDPool(unsigned long address)
{
    unsigned long i;
    pde_t **start = (pde_t **)address;
    for (i = 0; i < BIT(ASID_POOL_INDEX_BITS); i++) {
        pde_t *pde = start[i];
        if (pde != 0) {
            sendWord(i);
            sendWord((unsigned long)pde);
        }
    }
}

#ifdef CONFIG_IOMMU
static void sendIOPT(unsigned long address, unsigned int level)
{
    unsigned long i;
    vtd_pte_t *start = (vtd_pte_t *)address;
    for (i = 0; i < BIT(VTD_PT_INDEX_BITS); i++) {
        vtd_pte_t vtd_pte = start[i];
        if (vtd_pte_get_addr(vtd_pte) != 0) {
            sendWord(i);
            sendWord(vtd_pte.words[0]);
#ifdef CONFIG_ARCH_IA32
            sendWord(vtd_pte.words[1]);
#endif
            if (level == x86KSnumIOPTLevels) {
                sendWord(1);
            } else {
                sendWord(0);
            }
        }
    }
}

static void sendIOSpace(uint32_t pci_request_id)
{
    uint32_t   vtd_root_index;
    uint32_t   vtd_context_index;
    vtd_rte_t *vtd_root_slot;
    vtd_cte_t *vtd_context;
    vtd_cte_t *vtd_context_slot;

    vtd_root_index = get_pci_bus(pci_request_id);
    vtd_root_slot = x86KSvtdRootTable + vtd_root_index;

    vtd_context = (vtd_cte_t *)paddr_to_pptr(vtd_rte_ptr_get_ctp(vtd_root_slot));
    vtd_context_index = (get_pci_dev(pci_request_id) << 3) | get_pci_fun(pci_request_id);
    vtd_context_slot = &vtd_context[vtd_context_index];

    if (vtd_cte_ptr_get_present(vtd_context_slot)) {
        sendWord(vtd_cte_ptr_get_asr(vtd_context_slot));
    } else {
        sendWord(0);
    }
}
#endif

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
#ifdef CONFIG_IOMMU
    case IO_PT_COMMAND: {
        /*io pt table */
        unsigned long address = arg;
        word_t result = getWord(&level);
        if (result) {
            return result;
            }
            sendIOPT(address, level);
            putDebugChar(END);
        }
        break;
    case IO_SPACE_COMMAND:
        /*io space */
        sendIOSpace(arg);
        putDebugChar(END);
        break;
    default:
        return 0;
#endif
    }
    return 1;
}

#endif
