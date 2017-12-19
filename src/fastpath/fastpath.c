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
#include <fastpath/fastpath.h>
#include <object/notification.h>

#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
#include <benchmark/benchmark_track.h>
#endif
#include <benchmark/benchmark_utilisation.h>

void
#ifdef ARCH_X86
NORETURN
#endif
fastpath_call(word_t cptr, word_t msgInfo)
{
    seL4_MessageInfo_t info;
    cap_t ep_cap;
    endpoint_t *ep_ptr;
    word_t length;
    tcb_t *dest;
    word_t badge;
    cte_t *replySlot, *callerSlot;
    cap_t newVTable;
    vspace_root_t *cap_pd;
    pde_t stored_hw_asid;
    word_t fault_type;
    dom_t dom;

    /* Get message info, length, and fault type. */
    info = messageInfoFromWord_raw(msgInfo);
    length = seL4_MessageInfo_get_length(info);
    fault_type = seL4_Fault_get_seL4_FaultType(NODE_STATE(ksCurThread)->tcbFault);

    /* Check there's no extra caps, the length is ok and there's no
     * saved fault. */
    if (unlikely(fastpath_mi_check(msgInfo) ||
                 fault_type != seL4_Fault_NullFault)) {
        slowpath(SysCall);
    }

    /* Lookup the cap */
    ep_cap = lookup_fp(TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCTable)->cap, cptr);

    /* Check it's an endpoint */
    if (unlikely(!cap_capType_equals(ep_cap, cap_endpoint_cap) ||
                 !cap_endpoint_cap_get_capCanSend(ep_cap))) {
        slowpath(SysCall);
    }

    /* Get the endpoint address */
    ep_ptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(ep_cap));

    /* Get the destination thread, which is only going to be valid
     * if the endpoint is valid. */
    dest = TCB_PTR(endpoint_ptr_get_epQueue_head(ep_ptr));

    /* Check that there's a thread waiting to receive */
    if (unlikely(endpoint_ptr_get_state(ep_ptr) != EPState_Recv)) {
        slowpath(SysCall);
    }

    /* ensure we are not single stepping the destination in ia32 */
#if defined(CONFIG_HARDWARE_DEBUG_API) && defined(CONFIG_ARCH_IA32)
    if (dest->tcbArch.tcbContext.breakpointState.single_step_enabled) {
        slowpath(SysCall);
    }
#endif

    /* Get destination thread.*/
    newVTable = TCB_PTR_CTE_PTR(dest, tcbVTable)->cap;

    /* Get vspace root. */
    cap_pd = cap_vtable_cap_get_vspace_root_fp(newVTable);

    /* Ensure that the destination has a valid VTable. */
    if (unlikely(! isValidVTableRoot_fp(newVTable))) {
        slowpath(SysCall);
    }

#ifdef CONFIG_ARCH_AARCH32
    /* Get HW ASID */
    stored_hw_asid = cap_pd[PD_ASID_SLOT];
#endif

#ifdef CONFIG_ARCH_X86_64
    /* borrow the stored_hw_asid for PCID */
    stored_hw_asid.words[0] = cap_pml4_cap_get_capPML4MappedASID_fp(newVTable);
#endif

#ifdef CONFIG_ARCH_AARCH64
    stored_hw_asid.words[0] = cap_page_global_directory_cap_get_capPGDMappedASID(newVTable);
#endif

    /* let gcc optimise this out for 1 domain */
    dom = maxDom ? ksCurDomain : 0;
    /* ensure only the idle thread or lower prio threads are present in the scheduler */
    if (likely(dest->tcbPriority < NODE_STATE(ksCurThread->tcbPriority)) &&
            !isHighestPrio(dom, dest->tcbPriority)) {
        slowpath(SysCall);
    }

    /* Ensure that the endpoint has has grant rights so that we can
     * create the reply cap */
    if (unlikely(!cap_endpoint_cap_get_capCanGrant(ep_cap))) {
        slowpath(SysCall);
    }

#ifdef CONFIG_ARCH_AARCH32
    if (unlikely(!pde_pde_invalid_get_stored_asid_valid(stored_hw_asid))) {
        slowpath(SysCall);
    }
#endif

    /* Ensure the original caller is in the current domain and can be scheduled directly. */
    if (unlikely(dest->tcbDomain != ksCurDomain && maxDom)) {
        slowpath(SysCall);
    }

#ifdef ENABLE_SMP_SUPPORT
    /* Ensure both threads have the same affinity */
    if (unlikely(NODE_STATE(ksCurThread)->tcbAffinity != dest->tcbAffinity)) {
        slowpath(SysCall);
    }
#endif /* ENABLE_SMP_SUPPORT */

    /*
     * --- POINT OF NO RETURN ---
     *
     * At this stage, we have committed to performing the IPC.
     */

#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
    ksKernelEntry.is_fastpath = true;
#endif

    /* Dequeue the destination. */
    endpoint_ptr_set_epQueue_head_np(ep_ptr, TCB_REF(dest->tcbEPNext));
    if (unlikely(dest->tcbEPNext)) {
        dest->tcbEPNext->tcbEPPrev = NULL;
    } else {
        endpoint_ptr_mset_epQueue_tail_state(ep_ptr, 0, EPState_Idle);
    }

    badge = cap_endpoint_cap_get_capEPBadge(ep_cap);

    /* Block sender */
    thread_state_ptr_set_tsType_np(&NODE_STATE(ksCurThread)->tcbState,
                                   ThreadState_BlockedOnReply);

    /* Get sender reply slot */
    replySlot = TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbReply);

    /* Get dest caller slot */
    callerSlot = TCB_PTR_CTE_PTR(dest, tcbCaller);

    /* Insert reply cap */
    cap_reply_cap_ptr_new_np(&callerSlot->cap, 0, TCB_REF(NODE_STATE(ksCurThread)));
    mdb_node_ptr_set_mdbPrev_np(&callerSlot->cteMDBNode, CTE_REF(replySlot));
    mdb_node_ptr_mset_mdbNext_mdbRevocable_mdbFirstBadged(
        &replySlot->cteMDBNode, CTE_REF(callerSlot), 1, 1);

    fastpath_copy_mrs (length, NODE_STATE(ksCurThread), dest);

    /* Dest thread is set Running, but not queued. */
    thread_state_ptr_set_tsType_np(&dest->tcbState,
                                   ThreadState_Running);
    switchToThread_fp(dest, cap_pd, stored_hw_asid);

    msgInfo = wordFromMessageInfo(seL4_MessageInfo_set_capsUnwrapped(info, 0));

    fastpath_restore(badge, msgInfo, NODE_STATE(ksCurThread));
}

void
fastpath_reply_recv(word_t cptr, word_t msgInfo)
{
    seL4_MessageInfo_t info;
    cap_t ep_cap;
    endpoint_t *ep_ptr;
    word_t length;
    cte_t *callerSlot;
    cap_t callerCap;
    tcb_t *caller;
    word_t badge;
    tcb_t *endpointTail;
    word_t fault_type;

    cap_t newVTable;
    vspace_root_t *cap_pd;
    pde_t stored_hw_asid;
    dom_t dom;

    /* Get message info and length */
    info = messageInfoFromWord_raw(msgInfo);
    length = seL4_MessageInfo_get_length(info);
    fault_type = seL4_Fault_get_seL4_FaultType(NODE_STATE(ksCurThread)->tcbFault);

    /* Check there's no extra caps, the length is ok and there's no
     * saved fault. */
    if (unlikely(fastpath_mi_check(msgInfo) ||
                 fault_type != seL4_Fault_NullFault)) {
        slowpath(SysReplyRecv);
    }

    /* Lookup the cap */
    ep_cap = lookup_fp(TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCTable)->cap,
                       cptr);

    /* Check it's an endpoint */
    if (unlikely(!cap_capType_equals(ep_cap, cap_endpoint_cap) ||
                 !cap_endpoint_cap_get_capCanReceive(ep_cap))) {
        slowpath(SysReplyRecv);
    }

    /* Check there is nothing waiting on the notification */
    if (NODE_STATE(ksCurThread)->tcbBoundNotification &&
            notification_ptr_get_state(NODE_STATE(ksCurThread)->tcbBoundNotification) == NtfnState_Active) {
        slowpath(SysReplyRecv);
    }

    /* Get the endpoint address */
    ep_ptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(ep_cap));

    /* Check that there's not a thread waiting to send */
    if (unlikely(endpoint_ptr_get_state(ep_ptr) == EPState_Send)) {
        slowpath(SysReplyRecv);
    }

    /* Only reply if the reply cap is valid. */
    callerSlot = TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCaller);
    callerCap = callerSlot->cap;
    if (unlikely(!fastpath_reply_cap_check(callerCap))) {
        slowpath(SysReplyRecv);
    }

    /* Determine who the caller is. */
    caller = TCB_PTR(cap_reply_cap_get_capTCBPtr(callerCap));

    /* ensure we are not single stepping the caller in ia32 */
#if defined(CONFIG_HARDWARE_DEBUG_API) && defined(CONFIG_ARCH_IA32)
    if (caller->tcbArch.tcbContext.breakpointState.single_step_enabled) {
        slowpath(SysReplyRecv);
    }
#endif

    /* Check that the caller has not faulted, in which case a fault
       reply is generated instead. */
    fault_type = seL4_Fault_get_seL4_FaultType(caller->tcbFault);
    if (unlikely(fault_type != seL4_Fault_NullFault)) {
        slowpath(SysReplyRecv);
    }

    /* Get destination thread.*/
    newVTable = TCB_PTR_CTE_PTR(caller, tcbVTable)->cap;

    /* Get vspace root. */
    cap_pd = cap_vtable_cap_get_vspace_root_fp(newVTable);

    /* Ensure that the destination has a valid MMU. */
    if (unlikely(! isValidVTableRoot_fp (newVTable))) {
        slowpath(SysReplyRecv);
    }

#ifdef CONFIG_ARCH_AARCH32
    /* Get HWASID. */
    stored_hw_asid = cap_pd[PD_ASID_SLOT];
#endif

#ifdef CONFIG_ARCH_X86_64
    stored_hw_asid.words[0] = cap_pml4_cap_get_capPML4MappedASID(newVTable);
#endif

#ifdef CONFIG_ARCH_AARCH64
    stored_hw_asid.words[0] = cap_page_global_directory_cap_get_capPGDMappedASID(newVTable);
#endif

    /* Ensure the original caller can be scheduled directly. */
    dom = maxDom ? ksCurDomain : 0;
    if (unlikely(!isHighestPrio(dom, caller->tcbPriority))) {
        slowpath(SysReplyRecv);
    }

#ifdef CONFIG_ARCH_AARCH32
    /* Ensure the HWASID is valid. */
    if (unlikely(!pde_pde_invalid_get_stored_asid_valid(stored_hw_asid))) {
        slowpath(SysReplyRecv);
    }
#endif

    /* Ensure the original caller is in the current domain and can be scheduled directly. */
    if (unlikely(caller->tcbDomain != ksCurDomain && maxDom)) {
        slowpath(SysReplyRecv);
    }

#ifdef ENABLE_SMP_SUPPORT
    /* Ensure both threads have the same affinity */
    if (unlikely(NODE_STATE(ksCurThread)->tcbAffinity != caller->tcbAffinity)) {
        slowpath(SysReplyRecv);
    }
#endif /* ENABLE_SMP_SUPPORT */

    /*
     * --- POINT OF NO RETURN ---
     *
     * At this stage, we have committed to performing the IPC.
     */

#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
    ksKernelEntry.is_fastpath = true;
#endif

    /* Set thread state to BlockedOnReceive */
    thread_state_ptr_mset_blockingObject_tsType(
        &NODE_STATE(ksCurThread)->tcbState, (word_t)ep_ptr, ThreadState_BlockedOnReceive);

    /* Place the thread in the endpoint queue */
    endpointTail = endpoint_ptr_get_epQueue_tail_fp(ep_ptr);
    if (likely(!endpointTail)) {
        NODE_STATE(ksCurThread)->tcbEPPrev = NULL;
        NODE_STATE(ksCurThread)->tcbEPNext = NULL;

        /* Set head/tail of queue and endpoint state. */
        endpoint_ptr_set_epQueue_head_np(ep_ptr, TCB_REF(NODE_STATE(ksCurThread)));
        endpoint_ptr_mset_epQueue_tail_state(ep_ptr, TCB_REF(NODE_STATE(ksCurThread)),
                                             EPState_Recv);
    } else {
        /* Append current thread onto the queue. */
        endpointTail->tcbEPNext = NODE_STATE(ksCurThread);
        NODE_STATE(ksCurThread)->tcbEPPrev = endpointTail;
        NODE_STATE(ksCurThread)->tcbEPNext = NULL;

        /* Update tail of queue. */
        endpoint_ptr_mset_epQueue_tail_state(ep_ptr, TCB_REF(NODE_STATE(ksCurThread)),
                                             EPState_Recv);
    }

    /* Delete the reply cap. */
    mdb_node_ptr_mset_mdbNext_mdbRevocable_mdbFirstBadged(
        &CTE_PTR(mdb_node_get_mdbPrev(callerSlot->cteMDBNode))->cteMDBNode,
        0, 1, 1);
    callerSlot->cap = cap_null_cap_new();
    callerSlot->cteMDBNode = nullMDBNode;

    /* I know there's no fault, so straight to the transfer. */

    /* Replies don't have a badge. */
    badge = 0;

    fastpath_copy_mrs (length, NODE_STATE(ksCurThread), caller);

    /* Dest thread is set Running, but not queued. */
    thread_state_ptr_set_tsType_np(&caller->tcbState,
                                   ThreadState_Running);
    switchToThread_fp(caller, cap_pd, stored_hw_asid);

    msgInfo = wordFromMessageInfo(seL4_MessageInfo_set_capsUnwrapped(info, 0));

    fastpath_restore(badge, msgInfo, NODE_STATE(ksCurThread));
}

static inline void
mask_ack_bail(irq_t irq)
{
    maskInterrupt(true, irq);
    ackInterrupt(irq);
    restore_user_context();
}

void
fastpath_irq(irq_t irq)
{
    /* check the irq is valid */
    if (unlikely(irq == irqInvalid)) {
        if (config_set(CONFIG_IRQ_REPORTING)) {
            printf("Spurious interrupt\n");
        }
        handleSpuriousIRQ();
        restore_user_context();
        UNREACHABLE();
    }

    if (unlikely(irq > maxIRQ)) {
        mask_ack_bail(irq);
        UNREACHABLE();
    }

    if (unlikely(intStateIRQTable[irq] != IRQSignal)) {
        slowpath_irq(irq);
        UNREACHABLE();
    }

    cap_t ntfn_cap = intStateIRQNode[irq].cap;
    if (unlikely(cap_get_capType(ntfn_cap) != cap_notification_cap ||
                !cap_notification_cap_get_capNtfnCanSend(ntfn_cap))) {
        if (config_set(CONFIG_IRQ_REPORTING)) {
            printf("Undelivered irq: %d\n", (int) irq);
        }
        mask_ack_bail(irq);
        UNREACHABLE();
    }

    notification_t *ntfn_ptr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(ntfn_cap));
    notification_state_t ntfn_state = notification_ptr_get_state(ntfn_ptr);
    word_t badge = cap_notification_cap_get_capNtfnBadge(ntfn_cap);

    tcb_t *dest = NULL;
    if (ntfn_state == NtfnState_Waiting) {
        dest = TCB_PTR(notification_ptr_get_ntfnQueue_head(ntfn_ptr));
    } else if (notification_ptr_get_ntfnBoundTCB(ntfn_ptr)) {
        dest = TCB_PTR(notification_ptr_get_ntfnBoundTCB(ntfn_ptr));
#ifdef CONFIG_VTX
        if (unlikely(thread_state_ptr_get_tsType(dest->tcbState) == ThreadState_RunningVM)) {
            slowpath_irq(irq);
            UNREACHABLE();
        }
#endif
    } else {
        ntfn_set_active(ntfn_ptr, badge | notification_ptr_get_ntfnMsgIdentifier(ntfn_ptr));
        mask_ack_bail(irq);
        UNREACHABLE();
    }
    assert(dest != NULL);

#if CONFIG_MAX_NUM_NODES > 1
    if (dest->tcbAffinity != getCurrentCPUIndex()) {
        slowpath_irq(irq);
        UNREACHABLE();
    }
#endif

    cap_t newVTable = TCB_PTR_CTE_PTR(dest, tcbVTable)->cap;
#ifndef CONFIG_ARCH_AARCH32
    vspace_root_t *cap_pd;
#else
    pde_t *cap_pd;
#endif
    cap_pd = cap_vtable_cap_get_vspace_root_fp(newVTable);

    if (unlikely(!isValidVTableRoot_fp(newVTable))) {
        slowpath_irq(irq);
        UNREACHABLE();
    }

    pde_t stored_hw_asid;

#ifdef CONFIG_ARCH_X86_64
    stored_hw_asid.words[0] = cap_pml4_cap_get_capPML4MappedASID(newVTable);
#endif

#ifdef CONFIG_ARCH_AARCH64
    stored_hw_asid.words[0] = cap_page_global_directory_cap_get_capPGDMappedASID(newVTable);
#endif

#ifdef CONFIG_ARCH_AARCH32
    /* Get HW ASID */
    stored_hw_asid = cap_pd[PD_ASID_SLOT];
    if (unlikely(!pde_pde_invalid_get_stored_asid_valid(stored_hw_asid))) {
        slowpath_irq(irq);
        UNREACHABLE();
    }
#endif

    /* --- POINT OF NO RETURN -- */
    switch (ntfn_state) {
    case NtfnState_Idle:
    case NtfnState_Active:
        if (thread_state_get_tsType(dest->tcbState) == ThreadState_BlockedOnReceive) {
            endpoint_t *ep_ptr = EP_PTR(thread_state_get_blockingObject(dest->tcbState));
            endpoint_ptr_set_epQueue_head_np(ep_ptr, TCB_REF(dest->tcbEPNext));
            if (unlikely(dest->tcbEPNext)) {
                dest->tcbEPNext->tcbEPPrev = NULL;
            } else {
                endpoint_ptr_mset_epQueue_tail_state(ep_ptr, 0, EPState_Idle);
            }

            setRegister(dest, badgeRegister, badge);
            thread_state_ptr_set_tsType_np(&dest->tcbState, ThreadState_Running);
        } else {
            ntfn_set_active(ntfn_ptr, badge);
            mask_ack_bail(irq);
            UNREACHABLE();
        }
        break;
    case NtfnState_Waiting:
        notification_ptr_set_ntfnQueue_head_np(ntfn_ptr, TCB_REF(dest->tcbEPNext));
        if (unlikely(dest->tcbEPNext)) {
            dest->tcbEPNext->tcbEPPrev = NULL;
        } else {
            notification_ptr_mset_ntfnQueue_tail_state(ntfn_ptr, 0, NtfnState_Idle);
        }
        setRegister(dest, badgeRegister, badge);
        thread_state_ptr_set_tsType_np(&dest->tcbState, ThreadState_Running);
        break;
    }

    if (dest->tcbPriority > NODE_STATE(ksCurThread)->tcbPriority) {
        if (thread_state_get_tsType(NODE_STATE(ksCurThread->tcbState)) == ThreadState_Running) {
            SCHED_ENQUEUE_CURRENT_TCB;
        }
        switchToThread_fp(dest, cap_pd, stored_hw_asid);
    } else {
        tcbSchedEnqueue(dest);
    }

    mask_ack_bail(irq);
    UNREACHABLE();
}


void
#ifdef ARCH_X86
NORETURN
#endif
fastpath_signal(word_t cptr)
{
    word_t fault_type;
    cap_t ntfn_cap;
    notification_t *ntfn_ptr;
    tcb_t *dest = NULL;
    word_t badge;
    notification_state_t ntfn_state;

    /* get message info and fault type */
    fault_type = seL4_Fault_get_seL4_FaultType(ksCurThread->tcbFault);

    /* check there's no saved fault */
    if (unlikely(fault_type != seL4_Fault_NullFault)) {
        slowpath(SysSend);
    }

    /* lookup the cap */
    ntfn_cap = lookup_fp(TCB_PTR_CTE_PTR(ksCurThread, tcbCTable)->cap, cptr);

    /* check it's a notification object */
    if (unlikely(!cap_capType_equals(ntfn_cap, cap_notification_cap))) {
        slowpath(SysSend);
    }

    /* get the address */
    ntfn_ptr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(ntfn_cap));

    /* get the state */
    ntfn_state = notification_ptr_get_state(ntfn_ptr);

    if (ntfn_state == NtfnState_Waiting) {
        /* get the destination thread */
        dest = TCB_PTR(notification_ptr_get_ntfnQueue_head(ntfn_ptr));
    } else {
        /* get the bound tcb */
        dest = (tcb_t *) notification_ptr_get_ntfnBoundTCB(ntfn_ptr);
    }

    /* if the target is higher prio we'll need to invoke the scheduler,
     * this fastpath only fastpaths signal where we don't change threads */
    if (unlikely(dest && dest->tcbPriority > ksCurThread->tcbPriority)) {
        slowpath(SysSend);
    }
    /* --- POINT OF NO RETURN -- */

#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
    ksKernelEntry.is_fastpath = true;
#endif

    badge = cap_notification_cap_get_capNtfnBadge(ntfn_cap);

    switch (ntfn_state) {
    case NtfnState_Idle:
        if (dest && thread_state_get_tsType(dest->tcbState) == ThreadState_BlockedOnReceive) {
            endpoint_t *ep_ptr;
            ep_ptr = EP_PTR(thread_state_get_blockingObject(dest->tcbState));
            endpoint_ptr_set_epQueue_head_np(ep_ptr, TCB_REF(dest->tcbEPNext));
            if (unlikely(dest->tcbEPNext)) {
                dest->tcbEPNext->tcbEPPrev = NULL;
            } else {
                endpoint_ptr_mset_epQueue_tail_state(ep_ptr, 0, EPState_Idle);
            }

            setRegister(dest, badgeRegister, badge);
            thread_state_ptr_set_tsType_np(&dest->tcbState, ThreadState_Running);
            tcbSchedEnqueue(dest);
        } else {
            ntfn_set_active(ntfn_ptr, badge);
        }
        break;
    case NtfnState_Waiting:
        /* dequeue the destination */
        notification_ptr_set_ntfnQueue_head_np(ntfn_ptr, TCB_REF(dest->tcbEPNext));
        if (unlikely(dest->tcbEPNext)) {
            dest->tcbEPNext->tcbEPPrev = NULL;
        } else {
            notification_ptr_mset_ntfnQueue_tail_state(ntfn_ptr, 0, NtfnState_Idle);
        }
        setRegister(dest, badgeRegister, badge);
        thread_state_ptr_set_tsType_np(&dest->tcbState, ThreadState_Running);
        tcbSchedEnqueue(dest);
        break;
    case NtfnState_Active:
        ntfn_set_active(ntfn_ptr, badge | notification_ptr_get_ntfnMsgIdentifier(ntfn_ptr));
        break;
    }

    fastpath_restore(getRegister(ksCurThread, badgeRegister),
                     getRegister(ksCurThread, msgInfoRegister),
                     ksCurThread);
}


