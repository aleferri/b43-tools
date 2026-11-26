#ifndef B43_MASM_OUTPUT_H_
#define B43_MASM_OUTPUT_H_

#include "asm.h"

enum outfmt {
	FMT_B43,
	FMT_RAW_LE32,
	FMT_RAW_BE32,
};

int write_ucode(const struct asm_result *r, const char *path, enum outfmt fmt,
		int print_size);
int write_initvals(const struct asm_result *r, const char *dir, const char *ext,
		   int print_size);
int write_map(const struct asm_result *r, const char *path);
int write_listing(const struct asm_result *r, const char *path);

#endif /* B43_MASM_OUTPUT_H_ */
