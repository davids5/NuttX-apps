/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#if !defined(arraySize)
#define arraySize(a) (sizeof((a))/sizeof((a[0])))
#endif


#define HARDFAULT_REBOOT_FILENO 0
#define HARDFAULT_REBOOT_PATH BBSRAM_PATH "" STRINGIFY(HARDFAULT_REBOOT_FILENO)
#define HARDFAULT_FILENO 3
#define HARDFAULT_PATH BBSRAM_PATH "" STRINGIFY(HARDFAULT_FILENO)

#define BBSRAM_SIZE_FN0 (sizeof(int))
#define BBSRAM_SIZE_FN1 384     /* greater then 2.5 times the size of vehicle_status_s */
#define BBSRAM_SIZE_FN2 384     /* greater then 2.5 times the size of vehicle_status_s */
#define BBSRAM_SIZE_FN3 -1

/* The following guides in the amount of the user and interrupt stack
 * data we can save. The amount of storage left will dictate the actual
 * number of entries of the user stack data saved. If it is too big
 * It will be truncated by the call to stm32_bbsram_savepanic
 */
#define BBSRAM_HEADER_SIZE 20 /* This is an assumption */
#define BBSRAM_USED ((5*BBSRAM_HEADER_SIZE)+(BBSRAM_SIZE_FN0+ \
                                             BBSRAM_SIZE_FN1+ \
                                             BBSRAM_SIZE_FN2+ \
                                             BBSRAM_SIZE_FN3))
#define BBSRAM_REAMINING (4096-BBSRAM_USED)
#if CONFIG_ARCH_INTERRUPTSTACK <= 3
#  define BBSRAM_NUMBER_STACKS 1
#else
#  define BBSRAM_NUMBER_STACKS 2
#endif
#define BBSRAM_FIXED_ELEMENTS_SIZE (sizeof(struct info_t))
#define BBSRAM_LEFTOVER (BBSRAM_REAMINING-BBSRAM_FIXED_ELEMENTS_SIZE)

#define CONFIG_ISTACK_SIZE (BBSRAM_LEFTOVER/BBSRAM_NUMBER_STACKS/sizeof(stack_word_t))
#define CONFIG_USTACK_SIZE (BBSRAM_LEFTOVER/BBSRAM_NUMBER_STACKS/sizeof(stack_word_t))

/* The path to the Battery Backed up SRAM */
#define BBSRAM_PATH "/fs/bbr"

/* For Assert keep this much of the file name*/
#define MAX_FILE_PATH_LENGTH 40

/* Fixed size strings
 * To change a format add the number of chars not represented by the format
 * Specifier to the xxxx_NUM definei.e %Y is YYYY so add 2 and %s is -2
 * Also xxxxTIME_FMT need to match in size. See CCASERT in hardfault_log.c
 */
#define LOG_PATH_BASE       "/fs/microsd/"
#define LOG_PATH_BASE_LEN    ((arraySize(LOG_PATH_BASE))-1)

#define LOG_NAME_FMT        "fault_%s.log"
#define LOG_NAME_NUM         (     -2    )
#define LOG_NAME_LEN         ((arraySize(LOG_NAME_FMT)-1) + LOG_NAME_NUM)

#define TIME_FMT             "%Y_%m_%d_%H_%M_%S"
#define TIME_FMT_NUM         (2+ 0+ 0+ 0+ 0+ 0)
#define TIME_FMT_LEN         (((arraySize(TIME_FMT)-1) + TIME_FMT_NUM))

#define LOG_PATH_LEN         ((LOG_PATH_BASE_LEN + LOG_NAME_LEN + TIME_FMT_LEN))

#define HEADER_TIME_FMT      "%Y-%m-%d-%H:%M:%S"
#define HEADER_TIME_FMT_NUM  (2+ 0+ 0+ 0+ 0+ 0)
#define HEADER_TIME_FMT_LEN  (((arraySize(HEADER_TIME_FMT)-1) + HEADER_TIME_FMT_NUM))

/* Select which format to use. On a terminal the details are at the bottom
 * and in a file they are at the top
 */
#define HARDFAULT_DISPLAY_FORMAT 1
#define HARDFAULT_FILE_FORMAT    0

enum bbsramdf_e
{
  BBSRAM_CRC_VALID = 1,        /* The crc is valid */
  BBSRAM_DIRTY     = 2,        /* The file was closed */
};

struct bbsramd_s
{
  uint8_t flags;               /* The crc is valid and the file was closed */
  uint8_t fileno;              /* The minor number */
  uint16_t len;                /* Total Bytes in this file*/
  struct timespec lastwrite;   /* Last write time */
};

#define BBSRAM_GETDESC_IOCTL _DIOC(0x0010) /* Returns a bbsramd_s */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Used for stack frame storage */

typedef uint32_t stack_word_t;

/* Stack related data */

struct stack_t
{
  uint32_t sp;
  uint32_t top;
  uint32_t size;
};

struct stacks_t
{
   struct stack_t user;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
   struct stack_t interrupt;
#endif
};

/* Flags to identify what is in the dump */

enum fault_flags_t
{
  REGS_PRESENT          = 0x01,
  USERSTACK_PRESENT     = 0x02,
  INTSTACK_PRESENT      = 0x04,
  INVALID_USERSTACK_PTR = 0x20,
  INVALID_INTSTACK_PTR  = 0x40,
};

struct info_t
{
  enum fault_flags_t flags;                           /* What is in the dump */
  uintptr_t          current_regs;                    /* Used to validate the dump */
  int                lineno;                          /* __LINE__ to up_assert */
  int                pid;                             /* Process ID */
  uint32_t           regs[XCPTCONTEXT_REGS];          /* Interrupt register save area */
  struct stacks_t    stacks;                          /* Stack info */
#if CONFIG_TASK_NAME_SIZE > 0
  char               name[CONFIG_TASK_NAME_SIZE + 1]; /* Task name (with NULL terminator) */
#endif
  char               filename[MAX_FILE_PATH_LENGTH];  /* the Last of chars in __FILE__
                                                       *  to up_assert */
};

struct fullcontext_t
{
  struct info_t      info;              /* The info */
#if CONFIG_ARCH_INTERRUPTSTACK > 3 /* The amount of stack data is compile time
                                    * sized backed on what is left after the
                                    * other BBSRAM files are defined
                                    * The order is such that only the
                                    * ustack should be truncated
                                    */
  stack_word_t       istack[CONFIG_USTACK_SIZE];
#endif
  stack_word_t       ustack[CONFIG_ISTACK_SIZE];
};
