--
-- Copyright 2017, Data61
-- Commonwealth Scientific and Industrial Research Organisation (CSIRO)
-- ABN 41 687 119 230.
--
-- This software may be distributed and modified according to the terms of
-- the BSD 2-Clause license. Note that NO WARRANTY is provided.
-- See "LICENSE_BSD2.txt" for details.
--
-- @TAG(DATA61_BSD)
--

-- this file contains types shared between libsel4 and the kernel
#include <autoconf.h>

base 64

block seL4_MessageInfo {
    padding 32
    field label 19
    field capsUnwrapped 3
    field extraCaps 3
    field length 7
}

block seL4_PrioProps {
    field mcp  8
    field prio 8
#if CONFIG_NUM_CRITICALITIES > 1
    field mcc  8
    field crit 8
#else
    padding 16
#endif
    padding 32
}

block seL4_CapRights {
    padding 32

    padding 29
    field capAllowGrant 1
    field capAllowRead 1
    field capAllowWrite 1
}

-- CNode cap data
block seL4_CNode_CapData {
    field guard 58
    field guardSize 6
}
