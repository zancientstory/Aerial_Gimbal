#ifndef LOOP_FIFO_H
#define LOOP_FIFO_H

#include "struct_typedef.h"

extern void LoopFifoFp32_init(LoopFifofloat_t *fifo, float *address, uint32_t size);
extern void LoopFifoFp32_push(LoopFifofloat_t *fifo, float data);
extern float LoopFifoFp32_read(LoopFifofloat_t *fifo, uint32_t delta);
extern void LoopFifoFp32_clear(LoopFifofloat_t *fifo);

#endif
