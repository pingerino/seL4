/*
 * Copyright 2016, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(D61_GPL)
 */
#include <object/reply.h>

void
reply_push(tcb_t *tcb_caller, tcb_t *tcb_callee, reply_t *reply, bool_t canDonate)
{
    sched_context_t *sc_donated = tcb_caller->tcbSchedContext;

    assert(tcb_caller != NULL);
    assert(reply != NULL);
    assert(reply->replyTCB == NULL);

    if (tcb_callee->tcbSchedContext) {
        /* receiver already has sc */
        canDonate = false;
    }

    assert(call_stack_get_callStackPtr(reply->replyPrev) == 0);
    assert(call_stack_get_callStackPtr(reply->replyNext) == 0);

    /* tcb caller should not be in a existing call stack */
    assert(thread_state_get_replyObject(tcb_caller->tcbState) == 0);

    /* unlink callee and reply - they may not have been linked already,
     * if this rendesvous is occuring when seL4_Recv is called,
     * however, no harm in overring 0 with 0 */
    thread_state_ptr_set_replyObject(&tcb_callee->tcbState, 0);

    /* link caller and reply */
    reply->replyTCB = tcb_caller;
    thread_state_ptr_set_replyObject(&tcb_caller->tcbState, REPLY_REF(reply));
    setThreadState(tcb_caller, ThreadState_BlockedOnReply);

    if (sc_donated != NULL && canDonate) {
        assert(tcb_callee->tcbSchedContext == NULL);

        reply_t *old_caller = sc_donated->scReply;

        /* check stack integrity */
        assert(old_caller == NULL ||
               SC_PTR(call_stack_get_callStackPtr(old_caller->replyNext)) == sc_donated);

        /* push on to stack */
        reply->replyPrev = call_stack_new(REPLY_REF(old_caller), false);
        if (old_caller) {
            old_caller->replyNext = call_stack_new(REPLY_REF(reply), false);
        }
        reply->replyNext = call_stack_new(SC_REF(sc_donated), true);
        sc_donated->scReply = reply;

        /* now do the actual donation */
        schedContext_donate(sc_donated, tcb_callee);
    }
}

/* Pop the head reply from the call stack */
void
reply_pop(reply_t *reply)
{
    assert(reply != NULL);
    assert(reply->replyTCB != NULL);
    assert(thread_state_get_tsType(reply->replyTCB->tcbState) == ThreadState_BlockedOnReply);
    /* unlink tcb and reply */
    tcb_t *tcb = reply->replyTCB;
    reply_unlink(reply);

    word_t next_ptr = call_stack_get_callStackPtr(reply->replyNext);
    word_t prev_ptr = call_stack_get_callStackPtr(reply->replyPrev);

    if (likely(next_ptr != 0)) {
        assert(call_stack_get_isHead(reply->replyNext));

        /* give it back */
        schedContext_donate(SC_PTR(next_ptr), tcb);

        SC_PTR(next_ptr)->scReply = REPLY_PTR(prev_ptr);
        if (prev_ptr != 0) {
            REPLY_PTR(prev_ptr)->replyNext = reply->replyNext;
            assert(call_stack_get_isHead(REPLY_PTR(prev_ptr)->replyNext));
        }

        reply->replyPrev = call_stack_new(0, false);
        reply->replyNext = call_stack_new(0, false);
    }
}

/* Remove a reply from the middle of the call stack */
void
reply_remove(reply_t *reply)
{
    assert(thread_state_get_tsType(reply->replyTCB->tcbState) == ThreadState_BlockedOnReply);

    word_t next_ptr = call_stack_get_callStackPtr(reply->replyNext);
    word_t prev_ptr = call_stack_get_callStackPtr(reply->replyPrev);

    if (likely(next_ptr)) {
        if (likely(call_stack_get_isHead(reply->replyNext))) {
            /* head of the call stack -> just pop */
            reply_pop(reply);
            return;
        }
        /* not the head, remove from middle */
        REPLY_PTR(next_ptr)->replyPrev = reply->replyPrev;
        if (REPLY_PTR(next_ptr)->replyTCB) {
            reply_unlink(REPLY_PTR(next_ptr));
        }
        tcb_t *tcb = reply->replyTCB;
        assert(tcb);
        /* to maintain the call chain, we remove this caller and
         * replaced them with the next - so the TCB that the reply object
         * we are passing in is linked to is linked to the next reply object, and the
         * tcb of the next_ptr is dropped */
        REPLY_PTR(next_ptr)->replyTCB = tcb;
        reply->replyTCB = NULL;
        thread_state_ptr_set_replyObject(&tcb->tcbState, REPLY_REF(next_ptr));
    } else if (reply->replyTCB) {
        /* removing start of call chain */
        reply_unlink(reply);
    }

    if (prev_ptr) {
        REPLY_PTR(prev_ptr)->replyNext = reply->replyNext;
    }

    reply->replyPrev = call_stack_new(0, false);
    reply->replyNext = call_stack_new(0, false);
}

void reply_remove_tcb(tcb_t *tcb)
{
    assert(thread_state_get_tsType(tcb->tcbState) == ThreadState_BlockedOnReply);
    reply_t *reply = REPLY_PTR(thread_state_get_replyObject(tcb->tcbState));
    word_t next_ptr = call_stack_get_callStackPtr(reply->replyNext);
    word_t prev_ptr = call_stack_get_callStackPtr(reply->replyPrev);


    if (call_stack_get_isHead(reply->replyNext)) {
        reply_pop(reply);
        return;
    }

    if (next_ptr) {
        REPLY_PTR(next_ptr)->replyPrev = reply->replyPrev;
    }

    if (prev_ptr) {
        REPLY_PTR(prev_ptr)->replyNext = reply->replyNext;
    }

    reply->replyPrev = call_stack_new(0, false);
    reply->replyNext = call_stack_new(0, false);
    reply_unlink(reply);
}

void reply_clear(reply_t *reply) {
    assert(reply && reply->replyTCB);

    switch (thread_state_get_tsType(reply->replyTCB->tcbState)) {
    case ThreadState_BlockedOnReply:
        reply_remove(reply);
        break;
    case ThreadState_BlockedOnReceive:
        reply_unlink(reply);
        break;
    default:
        fail("Invalid state of replyTCB");
    }
}
