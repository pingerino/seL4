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

#ifndef ARMV_BENCHMARK_H
#define ARMV_BENCHMARK_H

#define PMCR_CYCLE_COUNT_RESET BIT(2)
#define PMCR_EVENT_COUNT_RESET BIT(1)
#define PMCR_ENABLE BIT(0)
/* 0 - cycle counter overflow irq on increment that changes bit 31 from 1 to 0
 * 1 - cycle counter overflow irq on increment that change bit 63 from 1 to 0.
 */
#define PMCR_LONG_CYCLE_COUNT BIT(6)

#define PMCNTENSET_CYCLE_COUNT_ENABLE BIT(31)

#define PMUSERENR_EL0_EN BIT(0)

#define CCNT "PMCCNTR_EL0"
/* clear the cycle count overflow irq */
#define PMOVSCLR "PMOVSCLR_EL0"
#define CCNT_OVERFLOW BIT(31)

#endif /* ARMV_BENCHMARK_H */
