#ifndef __SD_VECTOR_OP_H__
#define __SD_VECTOR_OP_H__

#define SD_VEC_SET(v, x, y, z) \
do { \
(v).n[0] = (x); \
(v).n[1] = (y); \
(v).n[2] = (z); \
} while (0)

#define SD_VEC_COPY(v, u) \
do { \
(v).n[0] = (u).n[0]; \
(v).n[1] = (u).n[1]; \
(v).n[2] = (u).n[2]; \
} while (0)

#define SD_VEC_ZERO(v) \
do { \
(v).n[0] = 0.0; \
(v).n[1] = 0.0; \
(v).n[2] = 0.0; \
} while (0)


#define SD_VEC_ADD(v, u, w) \
do { \
(v).n[0] = (u).n[0] + (w).n[0]; \
(v).n[1] = (u).n[1] + (w).n[1]; \
(v).n[2] = (u).n[2] + (w).n[2]; \
} while (0)

#define SD_VEC_SUB(v, u, w) \
do { \
(v).n[0] = (u).n[0] - (w).n[0]; \
(v).n[1] = (u).n[1] - (w).n[1]; \
(v).n[2] = (u).n[2] - (w).n[2]; \
} while (0)

#define SD_VEC_DOT(v, u) \
((v).n[0] * (u).n[0] + (v).n[1] * (u).n[1] + (v).n[2] * (u).n[2])

#define SD_VEC_CROSS(v, u, w) \
do { \
(v).n[0] = (u).n[1] * (w).n[2] - (u).n[2] * (w).n[1]; \
(v).n[1] = (u).n[2] * (w).n[0] - (u).n[0] * (w).n[2]; \
(v).n[2] = (u).n[0] * (w).n[1] - (u).n[1] * (w).n[0]; \
} while (0)

#define SD_VEC_LENGTH(v) \
sqrt((v).n[0] * (v).n[0] + (v).n[1] * (v).n[1] + (v).n[2] * (v).n[2])

#define SD_VEC_DIST(u,v) \
sqrt(((v).n[0] - (u).n[0]) * ((v).n[0] - (u).n[0]) + \
     ((v).n[1] - (u).n[1]) * ((v).n[1] - (u).n[1]) + \
     ((v).n[2] - (u).n[2]) * ((v).n[2] - (u).n[2]))

#define SD_VEC_NORMALIZE(v) \
do { \
double len = sqrt((v).n[0]*(v).n[0] + (v).n[1]*(v).n[1] + (v).n[2]*(v).n[2]); \
(v).n[0] = (v).n[0] / len; \
(v).n[1] = (v).n[1] / len; \
(v).n[2] = (v).n[2] / len; \
} while (0)

     
#define SD_VEC_INTERP(v, u, w, t) \
do { \
(v).n[0] = (1-(t))*(u).n[0] + (t)*(w).n[0]; \
(v).n[1] = (1-(t))*(u).n[1] + (t)*(w).n[1]; \
(v).n[2] = (1-(t))*(u).n[2] + (t)*(w).n[2]; \
} while (0)

#define SD_VEC_PRINT(v) \
do { \
cout << "(" << (v).n[0] << " " << (v).n[1] << " " << (v).n[2] << ")" << endl; \
} while (0)


#endif




