#ifndef VAS_COMMON_STRUCT_ENDIAN_H_
#define VAS_COMMON_STRUCT_ENDIAN_H_

#define STRUCT_ENDIAN_NOT_SET   0
#define STRUCT_ENDIAN_BIG       1
#define STRUCT_ENDIAN_LITTLE    2

extern int struct_get_endian(void);

#endif /* !VAS_COMMON_STRUCT_ENDIAN_H_ */
