#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include "main.h"

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef double fp64;

typedef struct
{
    float YawMotorAngle;
    float YawMotorSpeed;
    float PitchMotorAngle;
    float PitchMotorSpeed;
    float DaMiaoPitchMotorAngle;
    float DaMiaoPitchMotorSpeed;
} GimbalMotorMeasure_t;

typedef struct
{
    float RotorMotorSpeed;
    float AmmoLeftMotorSpeed;
    float AmmoRightMotorSpeed;
} ShootMotorMeasure_t;

typedef struct
{
    float YawAngle;
    float PitchAngle;
    float RollAngle;
    float YawSpeed;
    float PitchSpeed;
    float RollSpeed;
} EulerSystemMeasure_t;

typedef struct
{
    float *ptr;
    uint32_t offset;
    uint32_t size;
} LoopFifofloat_t;

#endif
