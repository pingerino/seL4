/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(GD_GPL)
 */

#ifndef __MACHINE_CAPDL_H
#define __MACHINE_CAPDL_H

#define ESCAPE               0xaa
#define START                0xff
#define END                  0xbb

#define START_ESCAPE         0xa0
#define ESCAPE_ESCAPE        0xa1
#define END_ESCAPE           0xa2

#define RQ_COMMAND           0xf1
#define EP_COMMAND           0xf2
#define CN_COMMAND           0xf3
#define TCB_COMMAND          0xfb
#define IRQ_COMMAND          0xf4
#define ASID_POOL_COMMAND    0xf6
#define VERSION_COMMAND      0xf9
#define DONE                 0xfa

#define CAPDL_VERSION        0

void capDL(void);
int getWord(word_t *res);
void sendWord(word_t word);
int doArchCommand(unsigned char c, word_t arg);
int doModeCommand(unsigned char c, word_t arg);

#endif
