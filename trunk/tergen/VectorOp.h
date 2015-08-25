#ifndef __VECTOR_OP_H__
#define __VECTOR_OP_H__


// This file contains some simple macros for vector operations.  These
// are currently only used by the routines to calculate the distance
// from a point to a triangle.


#define MIN(a,b) (a < b ? a : b)

#define VECTOR_OP(a,op,b,c) \
(a)[0] = (b)[0] op (c)[0]; \
(a)[1] = (b)[1] op (c)[1]; \
(a)[2] = (b)[2] op (c)[2]

#define VECTOR_DOT(a,b) ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2])

#define VECTOR_CROSS(a,b,c) \
(a)[0] = ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
(a)[1] = ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
(a)[2] = ((b)[0]*(c)[1] - (b)[1]*(c)[0])

#define VECTOR_LENGTH(a) (sqrt((a)[0]*(a)[0] + (a)[1]*(a)[1] + (a)[2]*(a)[2]))

#define VECTOR_NORMALIZE(a) \
do { \
double temp = sqrt((a)[0]*(a)[0] + (a)[1]*(a)[1] + (a)[2]*(a)[2]); \
(a)[0] = (a)[0] / temp; \
(a)[1] = (a)[1] / temp; \
(a)[2] = (a)[2] / temp; \
} while (0)

#endif

