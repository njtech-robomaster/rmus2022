/* Copyright 2021 Haowei Wen <yushijinhun@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdlib.h>
#include "tagRobomaster2021.h"

static uint64_t codedata[44] = {
   0x0000000000c81ec9UL,
   0x0000000000e23ec8UL,
   0x0000000000e2ae29UL,
   0x000000000028e9a6UL,
   0x0000000001e4ae4bUL,
   0x0000000000e4ae3bUL,
   0x0000000001e22280UL,
   0x0000000000e2ae2bUL,
   0x0000000000e3ae0bUL,
   0x0000000000f38f39UL,
   0x000000000051f073UL,
   0x0000000001e2ae7bUL,
   0x0000000000e28e38UL,
   0x0000000001e38e78UL,
   0x0000000001e43e7bUL,
   0x0000000001e4207bUL,
   0x0000000000e1ae39UL,
   0x000000000107b07bUL,
   0x0000000001ec1ec1UL,
   0x0000000000078e30UL,
   0x000000000104517bUL,
   0x0000000001001e78UL,
   0x000000000117d079UL,
   0x0000000001179179UL,
   0x0000000000e38e38UL,
   0x0000000001e2207bUL,
   0x0000000000e31738UL,
   0x0000000001e2317bUL,
   0x0000000000e4ae4bUL,
   0x0000000001ec0481UL,
   0x0000000001078e38UL,
   0x000000000107051cUL,
   0x0000000001078ab9UL,
   0x0000000001145145UL,
   0x0000000001144481UL,
   0x0000000001e45e45UL,
   0x0000000000e22409UL,
   0x0000000000bb659fUL,
   0x0000000000596f97UL,
   0x00000000005bf913UL,
   0x000000000059627fUL,
   0x0000000001e79e79UL,
   0x0000000000186187UL,
   0x0000000001efbefbUL,
};
apriltag_family_t *tagRobomaster2021_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagRobomaster2021");
   tf->h = 1;
   tf->ncodes = 44;
   tf->codes = codedata;
   tf->nbits = 25;
   tf->bit_x = calloc(25, sizeof(uint32_t));
   tf->bit_y = calloc(25, sizeof(uint32_t));
   tf->bit_x[0] = 1;
   tf->bit_y[0] = 1;
   tf->bit_x[1] = 2;
   tf->bit_y[1] = 1;
   tf->bit_x[2] = 3;
   tf->bit_y[2] = 1;
   tf->bit_x[3] = 4;
   tf->bit_y[3] = 1;
   tf->bit_x[4] = 2;
   tf->bit_y[4] = 2;
   tf->bit_x[5] = 3;
   tf->bit_y[5] = 2;
   tf->bit_x[6] = 5;
   tf->bit_y[6] = 1;
   tf->bit_x[7] = 5;
   tf->bit_y[7] = 2;
   tf->bit_x[8] = 5;
   tf->bit_y[8] = 3;
   tf->bit_x[9] = 5;
   tf->bit_y[9] = 4;
   tf->bit_x[10] = 4;
   tf->bit_y[10] = 2;
   tf->bit_x[11] = 4;
   tf->bit_y[11] = 3;
   tf->bit_x[12] = 5;
   tf->bit_y[12] = 5;
   tf->bit_x[13] = 4;
   tf->bit_y[13] = 5;
   tf->bit_x[14] = 3;
   tf->bit_y[14] = 5;
   tf->bit_x[15] = 2;
   tf->bit_y[15] = 5;
   tf->bit_x[16] = 4;
   tf->bit_y[16] = 4;
   tf->bit_x[17] = 3;
   tf->bit_y[17] = 4;
   tf->bit_x[18] = 1;
   tf->bit_y[18] = 5;
   tf->bit_x[19] = 1;
   tf->bit_y[19] = 4;
   tf->bit_x[20] = 1;
   tf->bit_y[20] = 3;
   tf->bit_x[21] = 1;
   tf->bit_y[21] = 2;
   tf->bit_x[22] = 2;
   tf->bit_y[22] = 4;
   tf->bit_x[23] = 2;
   tf->bit_y[23] = 3;
   tf->bit_x[24] = 3;
   tf->bit_y[24] = 3;
   tf->width_at_border = 7;
   tf->total_width = 9;
   tf->reversed_border = false;
   return tf;
}

void tagRobomaster2021_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
