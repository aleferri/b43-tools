#include "isa.h"

#include <string.h>

static const struct isa_op ops[] = {
	{ "mul",    F_ALU,  0x101, IF_REV11, 0 },
	{ "add",    F_ALU,  0x1C0, 0, 0 },
	{ "add.",   F_ALU,  0x1C2, 0, 0 },
	{ "addc",   F_ALU,  0x1C1, 0, 0 },
	{ "addc.",  F_ALU,  0x1C3, 0, 0 },
	{ "sub",    F_ALU,  0x1D0, 0, 0 },
	{ "sub.",   F_ALU,  0x1D2, 0, 0 },
	{ "subc",   F_ALU,  0x1D1, 0, 0 },
	{ "subc.",  F_ALU,  0x1D3, 0, 0 },
	{ "sra",    F_ALU,  0x130, 0, 0 },
	{ "or",     F_ALU,  0x160, 0, 0 },
	{ "and",    F_ALU,  0x140, 0, 0 },
	{ "xor",    F_ALU,  0x170, 0, 0 },
	{ "sr",     F_ALU,  0x120, 0, 0 },
	{ "sl",     F_ALU,  0x110, 0, 0 },
	{ "rl",     F_ALU,  0x1A0, 0, 0 },
	{ "rr",     F_ALU,  0x1B0, 0, 0 },
	{ "nand",   F_ALU,  0x150, 0, 0 },
	{ "srx",    F_EXT,  0x200, 0, 0 },
	{ "orx",    F_EXT,  0x300, 0, 0 },
	{ "mov",    F_MOV,  0x160, 0, 0 },
	{ "jmp",    F_JMP,  0x77F, 0, 0 },
	{ "jand",   F_JAND, 0x041, 0, 0 },
	{ "jnand",  F_JAND, 0x040, 0, 0 },
	{ "js",     F_JCMP, 0x050, 0, 0 },
	{ "jns",    F_JCMP, 0x051, 0, 0 },
	{ "je",     F_JCMP, 0x0D0, 0, 0 },
	{ "jne",    F_JCMP, 0x0D1, 0, 0 },
	{ "jls",    F_JCMP, 0x0D2, 0, 0 },
	{ "jges",   F_JCMP, 0x0D3, 0, 0 },
	{ "jgs",    F_JCMP, 0x0D4, 0, 0 },
	{ "jles",   F_JCMP, 0x0D5, 0, 0 },
	{ "jdn",    F_JCMP, 0x0D6, 0, 0 },
	{ "jdpz",   F_JCMP, 0x0D7, 0, 0 },
	{ "jdp",    F_JCMP, 0x0D8, 0, 0 },
	{ "jdnz",   F_JCMP, 0x0D9, 0, 0 },
	{ "jl",     F_JCMP, 0x0DA, 0, 0 },
	{ "jge",    F_JCMP, 0x0DB, 0, 0 },
	{ "jg",     F_JCMP, 0x0DC, 0, 0 },
	{ "jle",    F_JCMP, 0x0DD, 0, 0 },
	{ "jzx",    F_JX,   0x400, 0, 0 },
	{ "jnzx",   F_JX,   0x500, 0, 0 },
	{ "jext",   F_JEXT, 0x700, 0, 0 },
	{ "jnext",  F_JEXT, 0x600, 0, 0 },
	{ "call",   F_CALL, 0x002, IF_ARCH5, 0 },
	{ "calls",  F_CALLS, 0x004, IF_ARCH15, 0 },
	{ "ret",    F_RET,  0x003, IF_ARCH5, 0 },
	{ "rets",   F_RETS, 0x005, IF_ARCH15, 0 },
	{ "tkiph",  F_TKIP, 0x1E0, 0, 0x1 },
	{ "tkiphs", F_TKIP, 0x1E0, 0, 0x3 },
	{ "tkipl",  F_TKIP, 0x1E0, 0, 0x0 },
	{ "tkipls", F_TKIP, 0x1E0, 0, 0x2 },
	{ "nap",    F_NAP,  0x001, 0, 0 },
	{ "insn",   F_INSN, 0x000, 0, 0 },
};

const struct isa_op *isa_find(const char *mn)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ops); i++)
		if (strieq(ops[i].mn, mn))
			return &ops[i];
	return NULL;
}

unsigned int isa_nr_operands(enum fmt f)
{
	static const unsigned int n[] = {
		[F_ALU] = 3, [F_EXT] = 5, [F_MOV] = 2, [F_JMP] = 1, [F_JCMP] = 3,
		[F_JAND] = 3, [F_JX] = 5, [F_JEXT] = 2, [F_CALL] = 2, [F_CALLS] = 1,
		[F_RET] = 2, [F_RETS] = 0, [F_TKIP] = 2, [F_NAP] = 0, [F_INSN] = 4,
	};
	return n[f];
}

int isa_target_index(enum fmt f)
{
	switch (f) {
	case F_JMP: case F_CALLS: return 0;
	case F_JEXT: case F_CALL: return 1;
	case F_JCMP: case F_JAND: return 2;
	case F_JX: return 4;
	default: return -1;
	}
}

int isa_dest_index(enum fmt f)
{
	switch (f) {
	case F_ALU: return 2;
	case F_EXT: return 4;
	case F_MOV: case F_TKIP: return 1;
	case F_INSN: return 3;
	default: return -1;
	}
}

int isa_is_jump(enum fmt f)
{
	return f == F_JMP || f == F_JCMP || f == F_JAND || f == F_JX || f == F_JEXT;
}

static unsigned int field_bits(int arch)
{
	return arch == 5 ? 12 : 13;
}

unsigned int isa_code_limit(int arch)
{
	return 1u << field_bits(arch);
}

static unsigned int imm_bits(int arch)
{
	return arch == 5 ? 10 : 11;
}

/* Immediates are sign-extended from 10 (arch 5) or 11 (arch 15) bits to 16. */
static int imm_valid(int arch, int64_t v)
{
	unsigned int bits = imm_bits(arch);
	uint32_t u, mask;

	if (v < -32768 || v > 0xFFFF)
		return 0;
	u = v & 0xFFFF;
	mask = (~0u << bits) & 0xFFFF;
	if (u & (1u << (bits - 1)))
		return (u & mask) == mask;
	return (u & mask) == 0;
}

static uint32_t imm_encode(int arch, int64_t v)
{
	uint32_t base = arch == 5 ? 0xC00 : 0x1800;

	return base | (uint32_t)(v & ((1u << imm_bits(arch)) - 1));
}

static uint32_t sprbase(int arch)
{
	return arch == 5 ? 0x800 : 0x1000;
}

static int enc_reg(int arch, const struct operand *op, uint32_t *bits,
		   const struct srcloc *loc)
{
	switch (op->rc) {
	case RC_GPR:
		*bits = (arch == 5 ? 0xBC0 : 0x1780) | (uint32_t)op->n;
		return 0;
	case RC_SPR:
		if (op->n < 0 || op->n > (arch == 5 ? 0x1FF : 0x3FF)) {
			diag_error(loc, "spr%03llX does not exist on arch %d",
				   (unsigned long long)op->n, arch);
			return -1;
		}
		*bits = sprbase(arch) | (uint32_t)op->n;
		return 0;
	case RC_OFF:
		*bits = sprbase(arch) | (0x060 + (uint32_t)op->n);
		return 0;
	case RC_LR:
		break;
	}
	diag_error(loc, "link register not allowed here");
	return -1;
}

static int enc_mem(int arch, const struct operand *op, uint32_t *bits,
		   const struct srcloc *loc)
{
	int64_t lim;

	if (op->base < 0) {
		lim = arch == 5 ? 0x7FF : 0xFFF;
		if (op->n < 0 || op->n > lim) {
			diag_error(loc, "direct SHM address 0x%llX out of range (max 0x%llX on arch %d)",
				   (unsigned long long)op->n, (unsigned long long)lim, arch);
			return -1;
		}
		*bits = (uint32_t)op->n;
		return 0;
	}
	lim = arch == 5 ? 0x3F : 0x7F;
	if (op->n < 0 || op->n > lim) {
		diag_error(loc, "offset 0x%llX from off%d out of range (max 0x%llX on arch %d)",
			   (unsigned long long)op->n, op->base, (unsigned long long)lim, arch);
		return -1;
	}
	if (op->base == 6)
		diag_warn(loc, "off6 is broken on some devices");
	if (arch == 5)
		*bits = 0xA00 | ((uint32_t)op->base << 6) | (uint32_t)op->n;
	else
		*bits = 0x1400 | ((uint32_t)op->base << 7) | (uint32_t)op->n;
	return 0;
}

static int enc_raw(int arch, const struct operand *op, uint32_t *bits,
		   const struct srcloc *loc)
{
	if (op->n < 0 || op->n >= (int64_t)1 << field_bits(arch)) {
		diag_error(loc, "raw operand 0x%llX does not fit %u bits",
			   (unsigned long long)op->n, field_bits(arch));
		return -1;
	}
	*bits = (uint32_t)op->n;
	return 0;
}

static int enc_src(int arch, const struct operand *op, uint32_t *bits,
		   const struct srcloc *loc)
{
	switch (op->kind) {
	case OK_REG:
		return enc_reg(arch, op, bits, loc);
	case OK_MEM:
		return enc_mem(arch, op, bits, loc);
	case OK_RAW:
		return enc_raw(arch, op, bits, loc);
	case OK_IMM:
		if (!imm_valid(arch, op->n)) {
			diag_error(loc, "immediate 0x%llX not representable (%u bits, sign-extended) on arch %d",
				   (unsigned long long)op->n, imm_bits(arch), arch);
			return -1;
		}
		*bits = imm_encode(arch, op->n);
		return 0;
	}
	return -1;
}

static int enc_dst(int arch, const struct operand *op, uint32_t *bits,
		   const struct srcloc *loc)
{
	if (op->kind == OK_IMM && op->n != 0) {
		diag_error(loc, "destination must be a register or memory (0 discards the result)");
		return -1;
	}
	return enc_src(arch, op, bits, loc);
}

static int enc_target(int arch, const struct operand *op, uint32_t *bits,
		      const struct srcloc *loc)
{
	if (op->kind == OK_RAW)
		return enc_raw(arch, op, bits, loc);
	if (op->kind != OK_IMM) {
		diag_error(loc, "jump target must be a label or an address");
		return -1;
	}
	if (op->n < 0 || op->n >= (int64_t)isa_code_limit(arch)) {
		diag_error(loc, "jump target 0x%llX out of range on arch %d",
			   (unsigned long long)op->n, arch);
		return -1;
	}
	*bits = (uint32_t)op->n;
	return 0;
}

static int enc_lr(const struct operand *op, uint32_t *bits,
		  const struct srcloc *loc)
{
	if (op->kind != OK_REG || op->rc != RC_LR) {
		diag_error(loc, "link register expected");
		return -1;
	}
	*bits = (uint32_t)op->n;
	return 0;
}

static int enc_small(const struct operand *op, unsigned int max, const char *what,
		     unsigned int *out, const struct srcloc *loc)
{
	if (op->kind != OK_IMM || op->n < 0 || op->n > max) {
		diag_error(loc, "%s must be a number in 0..%u", what, max);
		return -1;
	}
	*out = (unsigned int)op->n;
	return 0;
}

static int check_inputs(const struct operand *a, const struct operand *b,
			const struct srcloc *loc)
{
	if (a->kind == OK_REG && a->rc == RC_SPR && b->kind == OK_REG && b->rc == RC_SPR) {
		diag_error(loc, "only one SPR source operand per instruction");
		return -1;
	}
	if (a->kind == OK_MEM && b->kind == OK_MEM) {
		diag_error(loc, "only one memory source operand per instruction");
		return -1;
	}
	return 0;
}

static uint64_t word(int arch, unsigned int opcode, uint32_t a, uint32_t b, uint32_t d)
{
	unsigned int s = field_bits(arch);

	return (uint64_t)d | (uint64_t)b << s | (uint64_t)a << (2 * s) |
	       (uint64_t)opcode << (3 * s);
}

static int encode_3(int arch, unsigned int opcode, const struct operand *a,
		    const struct operand *b, const struct operand *d, int d_is_target,
		    uint64_t *out, const struct srcloc *loc)
{
	uint32_t ea, eb, ed;

	if (check_inputs(a, b, loc) || enc_src(arch, a, &ea, loc) ||
	    enc_src(arch, b, &eb, loc))
		return -1;
	if (d_is_target ? enc_target(arch, d, &ed, loc) : enc_dst(arch, d, &ed, loc))
		return -1;
	*out = word(arch, opcode, ea, eb, ed);
	return 0;
}

static const struct operand r0 = { OK_REG, RC_GPR, 0, -1 };

static int encode_ext(int arch, unsigned int base, const struct operand *o,
		      int is_jump, uint64_t *out, const struct srcloc *loc)
{
	unsigned int m, s;

	if (enc_small(&o[0], 15, "mask width-1", &m, loc) ||
	    enc_small(&o[1], 15, "shift", &s, loc))
		return -1;
	return encode_3(arch, base | m << 4 | s, &o[2], &o[3], &o[4], is_jump, out, loc);
}

static int encode_mov(int arch, const struct operand *o, uint64_t *out,
		      const struct srcloc *loc)
{
	struct operand x[5];

	if (o[0].kind != OK_IMM || imm_valid(arch, o[0].n)) {
		struct operand zero = { OK_IMM, RC_GPR, 0, -1 };

		return encode_3(arch, 0x160, &o[0], &zero, &o[1], 0, out, loc);
	}
	if (o[0].n < -32768 || o[0].n > 0xFFFF) {
		diag_error(loc, "mov immediate 0x%llX wider than 16 bits",
			   (unsigned long long)o[0].n);
		return -1;
	}
	memset(x, 0, sizeof(x));
	x[0].kind = x[1].kind = x[2].kind = x[3].kind = OK_IMM;
	x[0].n = 7;
	x[1].n = 8;
	x[2].n = (o[0].n >> 8) & 0xFF;
	x[3].n = o[0].n & 0xFF;
	x[4] = o[1];
	return encode_ext(arch, 0x300, x, 0, out, loc);
}

static int encode_jand(int arch, const struct isa_op *op, const struct operand *o,
		       uint64_t *out, const struct srcloc *loc)
{
	const struct operand *imm = NULL, *reg = NULL;
	struct operand x[5];
	uint32_t mask;
	unsigned int lo, width;

	if (o[0].kind == OK_IMM && o[1].kind != OK_IMM) {
		imm = &o[0];
		reg = &o[1];
	} else if (o[1].kind == OK_IMM && o[0].kind != OK_IMM) {
		imm = &o[1];
		reg = &o[0];
	}
	if (!imm || imm_valid(arch, imm->n))
		return encode_3(arch, op->opcode, &o[0], &o[1], &o[2], 1, out, loc);
	if (imm->n < 0 || imm->n > 0xFFFF) {
		diag_error(loc, "bit mask 0x%llX wider than 16 bits", (unsigned long long)imm->n);
		return -1;
	}
	mask = (uint32_t)imm->n;
	for (lo = 0; !(mask & (1u << lo)); lo++)
		;
	for (width = 0; lo + width < 16 && (mask & (1u << (lo + width))); width++)
		;
	if (mask != ((1u << width) - 1) << lo) {
		diag_error(loc, "long bit mask 0x%X is not contiguous", mask);
		return -1;
	}
	memset(x, 0, sizeof(x));
	x[0].kind = x[1].kind = x[3].kind = OK_IMM;
	x[0].n = width - 1;
	x[1].n = lo;
	x[2] = *reg;
	x[3].n = 0;
	x[4] = o[2];
	return encode_ext(arch, op->opcode & 1 ? 0x500 : 0x400, x, 1, out, loc);
}

int isa_encode(int arch, const struct isa_op *op, const struct operand *o,
	       uint64_t *out, const struct srcloc *loc)
{
	struct operand imm;
	uint32_t a, d;
	unsigned int v;

	switch (op->fmt) {
	case F_ALU:
		return encode_3(arch, op->opcode, &o[0], &o[1], &o[2], 0, out, loc);
	case F_EXT:
		return encode_ext(arch, op->opcode, o, 0, out, loc);
	case F_MOV:
		return encode_mov(arch, o, out, loc);
	case F_JMP:
		return encode_3(arch, op->opcode, &r0, &r0, &o[0], 1, out, loc);
	case F_JCMP:
		return encode_3(arch, op->opcode, &o[0], &o[1], &o[2], 1, out, loc);
	case F_JAND:
		return encode_jand(arch, op, o, out, loc);
	case F_JX:
		return encode_ext(arch, op->opcode, o, 1, out, loc);
	case F_JEXT:
		if (enc_small(&o[0], 0xFF, "external condition", &v, loc))
			return -1;
		return encode_3(arch, op->opcode | v, &r0, &r0, &o[1], 1, out, loc);
	case F_CALL:
		if (enc_lr(&o[0], &a, loc) || enc_target(arch, &o[1], &d, loc))
			return -1;
		*out = word(arch, op->opcode, a, 0, d);
		return 0;
	case F_CALLS:
		return encode_3(arch, op->opcode, &r0, &r0, &o[0], 1, out, loc);
	case F_RET:
		if (enc_lr(&o[0], &a, loc) || enc_lr(&o[1], &d, loc))
			return -1;
		*out = word(arch, op->opcode, a, 0, d);
		return 0;
	case F_RETS:
	case F_NAP: {
		uint32_t g0;

		enc_reg(arch, &r0, &g0, loc);
		*out = word(arch, op->opcode, g0, g0, 0);
		return 0;
	}
	case F_TKIP:
		memset(&imm, 0, sizeof(imm));
		imm.kind = OK_IMM;
		imm.n = op->aux;
		return encode_3(arch, op->opcode, &o[0], &imm, &o[1], 0, out, loc);
	case F_INSN:
		if (enc_small(&o[0], 0xFFF, "opcode", &v, loc))
			return -1;
		{
			uint32_t ea, eb, ed;

			if (enc_src(arch, &o[1], &ea, loc) || enc_src(arch, &o[2], &eb, loc) ||
			    enc_src(arch, &o[3], &ed, loc))
				return -1;
			*out = word(arch, v, ea, eb, ed);
		}
		return 0;
	}
	return -1;
}
