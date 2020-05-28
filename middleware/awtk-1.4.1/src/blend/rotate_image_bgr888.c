﻿/**
 * File:   rotate_image_bgr888.c
 * Author: AWTK Develop Team
 * Brief:  rotate on bgr888
 *
 * Copyright (c) 2018 - 2020  Guangzhou ZHIYUAN Electronics Co.,Ltd.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * License file for more details.
 *
 */

/**
 * History:
 * ================================================================
 * 2018-10-10 Generated by gen.sh(DONT MODIFY IT)
 *
 */
#include "tkc/rect.h"
#include "base/pixel.h"
#include "base/bitmap.h"
#include "base/pixel_pack_unpack.h"

#define pixel_dst_t pixel_bgr888_t
#define pixel_dst_format pixel_bgr888_format
#define pixel_dst_to_rgba pixel_bgr888_to_rgba
#define pixel_dst_from_rgb pixel_bgr888_from_rgb

#include "rotate_image.inc"

ret_t rotate_bgr888_image(bitmap_t* fb, bitmap_t* img, rect_t* src, lcd_orientation_t o) {
  return rotate_image(fb, img, src, o);
}
