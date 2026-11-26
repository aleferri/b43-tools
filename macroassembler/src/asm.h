#ifndef B43_MASM_ASM_H_
#define B43_MASM_ASM_H_

#include "masm.h"

struct asm_opts {
	int arch;
	int corerev;
	strvec_t defines;	/* NAME or NAME=VALUE */
	strvec_t incdirs;
	int listing;
};

enum ivkind {
	IV_MMIO16,
	IV_MMIO32,
	IV_PHY,
	IV_RADIO,
	IV_SHM16,
	IV_SHM32,
	IV_TRAM,
};

struct ivop {
	enum ivkind kind;
	uint32_t value, routing, offset;
};

struct ivsect {
	char *name;
	VEC(struct ivop) ops;
};

struct listent {
	const struct srcloc *loc;
	char *text;
	struct segment *seg;
	unsigned int from, to;
};

struct asm_result {
	int arch;
	int corerev;
	VEC(struct segment *) segs;
	VEC(struct ivsect *) ivs;
	VEC(struct proc *) procs;
	struct symtab *globals;
	VEC(struct listent) listing;
	unsigned int passes;
};

int asm_run(const char *file, const struct asm_opts *opts, struct asm_result *res);

#endif /* B43_MASM_ASM_H_ */
