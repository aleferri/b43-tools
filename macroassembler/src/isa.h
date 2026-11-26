#ifndef B43_MASM_ISA_H_
#define B43_MASM_ISA_H_

#include "masm.h"

enum fmt {
	F_ALU,		/* A, B, D */
	F_EXT,		/* M, S, A, B, D */
	F_MOV,		/* A, D */
	F_JMP,		/* L */
	F_JCMP,		/* A, B, L */
	F_JAND,		/* A, B, L; long masks become jzx/jnzx */
	F_JX,		/* M, S, A, B, L */
	F_JEXT,		/* C, L */
	F_CALL,		/* lrX, L */
	F_CALLS,	/* L */
	F_RET,		/* lrX, lrY */
	F_RETS,
	F_TKIP,		/* A, D */
	F_NAP,
	F_INSN,		/* opcode, A, B, D */
};

#define IF_ARCH5	0x1
#define IF_ARCH15	0x2
#define IF_REV11	0x4

struct isa_op {
	const char *mn;
	enum fmt fmt;
	unsigned int opcode;
	unsigned int flags;
	unsigned int aux;
};

enum opkind {
	OK_REG,
	OK_IMM,
	OK_MEM,
	OK_RAW,
};

struct operand {
	enum opkind kind;
	enum regclass rc;
	int64_t n;
	int base;		/* OK_MEM: offset register, -1 for a direct address */
};

const struct isa_op *isa_find(const char *mn);
unsigned int isa_nr_operands(enum fmt f);
int isa_target_index(enum fmt f);
int isa_dest_index(enum fmt f);
int isa_is_jump(enum fmt f);
unsigned int isa_code_limit(int arch);
int isa_encode(int arch, const struct isa_op *op, const struct operand *ops,
	       uint64_t *out, const struct srcloc *loc);

#endif /* B43_MASM_ISA_H_ */
