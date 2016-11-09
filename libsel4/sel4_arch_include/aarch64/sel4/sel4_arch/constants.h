/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */

#ifndef __LIBSEL4_SEL4_ARCH_CONSTANTS_H
#define __LIBSEL4_SEL4_ARCH_CONSTANTS_H

#ifdef HAVE_AUTOCONF
#include <autoconf.h>
#endif

#ifndef __ASSEMBLER__
/* format of an unknown syscall message */
enum {
    seL4_UnknownSyscall_X0,
    seL4_UnknownSyscall_X1,
    seL4_UnknownSyscall_X2,
    seL4_UnknownSyscall_X3,
    seL4_UnknownSyscall_X4,
    seL4_UnknownSyscall_X5,
    seL4_UnknownSyscall_X6,
    seL4_UnknownSyscall_X7,
    seL4_UnknownSyscall_FaultIP,
    seL4_UnknownSyscall_SP,
    seL4_UnknownSyscall_LR,
    seL4_UnknownSyscall_SPSR,
    seL4_UnknownSyscall_Syscall,
    /* length of an unknown syscall message */
    seL4_UnknownSyscall_Length,
    SEL4_FORCE_LONG_ENUM(seL4_UnknownSyscall_Msg),
} seL4_UnknownSyscall_Msg;

/* format of a user exception message */
enum {
    seL4_UserException_FaultIP,
    seL4_UserException_SP,
    seL4_UserException_SPSR,
    seL4_UserException_Number,
    seL4_UserException_Code,
    /* length of a user exception */
    seL4_UserException_Length,
    SEL4_FORCE_LONG_ENUM(seL4_UserException_Msg),
} seL4_UserException_Msg;

/* format of a vm fault message */
enum {
    seL4_VMFault_IP,
    seL4_VMFault_Addr,
    seL4_VMFault_PrefetchFault,
    seL4_VMFault_FSR,
    seL4_VMFault_Length,
    SEL4_FORCE_LONG_ENUM(seL4_VMFault_Msg),
} seL4_VMFault_Msg;

#endif /* !__ASSEMBLER__ */

#define seL4_DataFault 0
#define seL4_InstructionFault 1
/* object sizes - 2^n */
#define seL4_PageBits 12
#define seL4_LargePageBits 21
#define seL4_HugePageBits 30
#define seL4_SlotBits 5
#define seL4_TCBBits 11
#define seL4_EndpointBits 4
#define seL4_NotificationBits 5
#define seL4_SchedContextBits 8
#define seL4_ReplyBits           5

#define seL4_PageTableBits 12
#define seL4_PageTableEntryBits 3
#define seL4_PageTableIndexBits 9

#define seL4_PageDirBits 12
#define seL4_PageDirEntryBits 3
#define seL4_PageDirIndexBits 9

#define seL4_ASIDPoolBits 12
#define seL4_ASIDPoolIndexBits 9
#define seL4_IOPageTableBits 12
#define seL4_WordSizeBits 3

#define seL4_PGDBits 12
#define seL4_PGDEntryBits 3
#define seL4_PGDIndexBits    9

#define seL4_PUDBits 12
#define seL4_PUDEntryBits 3
#define seL4_PUDIndexBits 9

/* word size */
#define seL4_WordBits (sizeof(seL4_Word) * 8)

/* Untyped size limits */
#define seL4_MinUntypedBits 4
#define seL4_MaxUntypedBits 47

#ifndef __ASSEMBLER__
SEL4_SIZE_SANITY(seL4_PageTableEntryBits, seL4_PageTableIndexBits, seL4_PageTableBits);
SEL4_SIZE_SANITY(seL4_PageDirEntryBits, seL4_PageDirIndexBits, seL4_PageDirBits);
SEL4_SIZE_SANITY(seL4_WordSizeBits, seL4_ASIDPoolIndexBits, seL4_ASIDPoolBits);
SEL4_SIZE_SANITY(seL4_PGDEntryBits, seL4_PGDIndexBits, seL4_PGDBits);
SEL4_SIZE_SANITY(seL4_PUDEntryBits, seL4_PUDIndexBits, seL4_PUDBits);
#endif

#ifdef CONFIG_ENABLE_BENCHMARKS
/* size of kernel log buffer in bytes */
#define seL4_LogBufferSize (LIBSEL4_BIT(20))
#endif /* CONFIG_ENABLE_BENCHMARKS */

#ifdef CONFIG_HARDWARE_DEBUG_API
#define seL4_FirstBreakpoint (0)
#define seL4_FirstDualFunctionMonitor (-1)
#define seL4_NumDualFunctionMonitors (0)
#endif

#define seL4_FastMessageRegisters 4

/* IPC buffer is 1024 bytes, giving size bits of 10 */
#define seL4_IPCBufferSizeBits 10

#endif
