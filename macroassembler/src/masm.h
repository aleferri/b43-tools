#ifndef B43_MASM_H_
#define B43_MASM_H_

#include "util.h"
#include "lex.h"
#include "source.h"

#define NR_GPR		64
#define NR_OFFR		7
#define NR_LR		4

enum space {
	SP_CODE,
	SP_SHM,
	SP_GPR,
	NR_SPACES,
};

enum regclass {
	RC_GPR,
	RC_SPR,
	RC_OFF,
	RC_LR,
};

#define VF_UNDEF	0x01	/* not known in this pass */
#define VF_CODE		0x02	/* code address */
#define VF_MEM		0x04	/* SHM data: as an operand it is a memory access */
#define VF_TYPE		0x08	/* a type, or an offset inside one */
#define VF_RFIELD	0x10	/* a record field: n is its shift */

struct type;
struct rfield;
struct sym;

struct value {
	enum { V_NUM, V_REG } kind;
	int64_t n;
	enum regclass rc;
	int base;			/* offset register of an address, -1 if none */
	unsigned int flags;
	const struct type *type;
	unsigned int count;		/* array elements, 0 for a scalar */
	const struct rfield *rf;
	const struct sym *sym;		/* set when the value is a bare symbol */
};

struct iword {
	uint16_t v;
	uint8_t set;
};

typedef VEC(struct iword) image_t;

enum type_kind {
	TY_WORD,
	TY_DWORD,
	TY_STRUCT,
	TY_UNION,
	TY_RECORD,
};

struct field {
	char *name;
	unsigned int offset;
	const struct type *type;
	unsigned int count;
	int alt;			/* union member after the first */
	image_t def;
};

struct rfield {
	char *name;
	unsigned int shift, width;
	uint16_t def;
	const struct type *rec;
};

struct type {
	enum type_kind kind;
	char *name;
	unsigned int size;		/* words */
	unsigned int width;		/* records: bits */
	VEC(struct field) fields;
	VEC(struct rfield) rfields;
	image_t def;
};

extern const struct type type_word, type_dword;

struct cell {
	uint64_t v;
	uint8_t set;
};

struct segment {
	char *name;
	enum space space;
	const struct srcloc *loc;
	int has_at;
	int64_t at;
	int64_t base, base_prev;
	int placed;
	unsigned int align;
	unsigned int cur, size, size_prev;
	int seen_pass;
	char *ivname;
	VEC(struct cell) cells;
};

struct regset {
	uint64_t gpr;
	uint8_t off;
};

enum pdir {
	PD_IN,
	PD_OUT,
	PD_INOUT,
};

struct param {
	char *name;
	enum pdir dir;
	struct value reg;
};

typedef VEC(struct param) paramvec_t;

struct contract {
	int link;			/* arch 5 link register */
	int uses_all;
	struct regset regs;		/* registers the proc may write */
	paramvec_t params;
	paramvec_t locals;
};

struct callsite {
	char *target;
	int tail;
	struct proc *callee;
};

struct proc {
	char *name;
	const struct srcloc *loc;
	int is_top;
	char *header;
	const struct srcloc *header_loc;
	body_t locals;			/* LOCAL lines, as written */
	struct contract c;		/* in force during the current pass */
	struct contract next;		/* resolved at the end of the pass */
	VEC(struct callsite) calls;
	unsigned int linkset;		/* arch 5: link registers live while it runs */
	int depth;			/* arch 15: return stack entries it needs */
	int walk;
	int seen_pass;
	struct symtab *scope, *scope_prev;
};

enum sym_kind {
	S_CONST,
	S_REDEF,
	S_TEXT,
	S_LABEL,
	S_DATA,
	S_TYPE,
	S_MACRO,
	S_PROC,
	S_SEGMENT,
};

struct macro;

struct sym {
	char *name;
	enum sym_kind kind;
	const struct srcloc *loc;
	struct sym *next;
	struct value val;
	char *text;
	struct segment *seg;
	unsigned int off;
	const struct type *type;
	unsigned int count;
	struct macro *macro;
	struct proc *proc;
};

/* sym.c */
struct symtab *symtab_new(void);
struct sym *sym_get(struct symtab *t, const char *name);
struct sym *sym_put(struct symtab *t, const char *name, enum sym_kind kind,
		    const struct srcloc *loc);
void sym_remove(struct symtab *t, const char *name);
size_t symtab_list(struct symtab *t, struct sym ***out);

/* expr.c */
struct exprctx {
	const struct toklist *tl;
	size_t i;
	const struct srcloc *loc;
	int addr_mode;
};

void value_num(struct value *v, int64_t n);
void value_reg(struct value *v, enum regclass rc, int64_t n);
int value_known(const struct value *v);
int parse_register(const char *name, struct value *v);
int expr_parse(struct exprctx *x, struct value *v);
const struct token *ex_peek(struct exprctx *x);
const struct token *ex_next(struct exprctx *x);
int ex_accept(struct exprctx *x, enum punct p);
int ex_at_end(struct exprctx *x);

/* Services the expression evaluator needs from the assembler. */
int asm_lookup(const char *name, struct value *v, const struct srcloc *loc);
int asm_defined(const char *name);
int asm_here(struct value *v, const struct srcloc *loc);
int asm_anon_label(int forward, struct value *v, const struct srcloc *loc);
int asm_segment_size(const char *name, struct value *v);
int asm_pass(void);

/* types.c */
struct type *type_new(enum type_kind kind, const char *name);
const struct field *type_field(const struct type *t, const char *name);
const struct rfield *type_rfield(const struct type *t, const char *name);
int build_items(const struct type *t, const char *text, image_t *img,
		unsigned int *count, const struct srcloc *loc);
int build_aggregate(const struct type *t, const char *text, image_t *img,
		    const struct srcloc *loc);
void image_append(image_t *dst, const image_t *src);
void image_fill(image_t *img, unsigned int n);
int eval_text(const char *text, struct value *v, const struct srcloc *loc);
int split_list(const char *s, strvec_t *out);
char *strip_angle(const char *s);

/* proc.c */
int contract_eval(struct proc *p, struct contract *c, int strict);
int contract_locals_line(const struct bodyline *bl, paramvec_t *out, int strict);
int contract_equal(const struct contract *a, const struct contract *b);
void contract_free(struct contract *c);
int regset_has(const struct regset *rs, const struct value *reg);
void proc_analyze(struct proc **procs, size_t n, int arch);
void proc_check_write(struct proc *p, const struct value *dst,
		      const struct srcloc *loc);
void proc_check_call(struct proc *caller, struct proc *callee, int tail,
		     int raw_link, int arch, const struct srcloc *loc);
void regset_format(const struct regset *rs, struct strbuf *sb);

#endif /* B43_MASM_H_ */
