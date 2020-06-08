/**
 * file:   platform.c
 * author: li xianjing <xianjimli@hotmail.com>
 * brief:  platform dependent function of stm32
 *
 * copyright (c) 2018 - 2018 Guangzhou ZHIYUAN Electronics Co.,Ltd. 
 *
 * this program is distributed in the hope that it will be useful,
 * but without any warranty; without even the implied warranty of
 * merchantability or fitness for a particular purpose.  see the
 * license file for more details.
 *
 */

/**
 * history:
 * ================================================================
 * 2018-05-12 li xianjing <xianjimli@hotmail.com> created
 *
 */

#include "fsl_common.h"
#include "fsl_elcdif.h"
#include "fsl_debug_console.h"

#include "tkc/mem.h"
#include "base/timer.h"
#include "tkc/platform.h"

extern uint64_t get_time_ms64(void);

SDK_ALIGN(uint8_t awtk_mem[4 * 1024 * 1024], 64);
//uint8_t awtk_mem[4 * 1024 * 1024];

#define MEM2_MAX_SIZE		4 * 1024 * 1024 
#define MEM2_ADDR           awtk_mem

ret_t platform_prepare(void) {
	timer_prepare(get_time_ms64);
	tk_mem_init(MEM2_ADDR, MEM2_MAX_SIZE);
//    TKMEM_INIT(MEM2_MAX_SIZE);
	
	return RET_OK;
}


