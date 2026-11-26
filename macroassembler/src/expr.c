#include "masm.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

enum binop {
	B_LOR, B_LAND, B_OR, B_XOR, B_AND, B_EQ, B_NE, B_LT, B_LE, B_GT, B_GE,
	B_SHL, B_SHR, B_ADD, B_SUB, B_MUL, B_DIV, B_MOD,
};

static const struct {
	enum punct p;
	const char *kw;
	enum binop op;
	int prec;
} binops[] = {
	{ P_LOR, NULL, B_LOR, 1 },
	{ P_LAND, NULL, B_LAND, 2 },
	{ P_PIPE, "OR", B_OR, 3 },
	{ P_CARET, "XOR", B_XOR, 4 },
	{ P_AMP, "AND", B_AND, 5 },
	{ P_EQ, "EQ", B_EQ, 6 }, { P_NE, "NE", B_NE, 6 },
	{ P_LT, "LT", B_LT, 7 }, { P_LE, "LE", B_LE, 7 },
	{ P_GT, "GT", B_GT, 7 }, { P_GE, "GE", B_GE, 7 },
	{ P_SHL, "SHL", B_SHL, 8 }, { P_SHR, "SHR", B_SHR, 8 },
	{ P_PLUS, NULL, B_ADD, 9 }, { P_MINUS, NULL, B_SUB, 9 },
	{ P_STAR, NULL, B_MUL, 10 }, { P_SLASH, NULL, B_DIV, 10 },
	{ P_PERCENT, "MOD", B_MOD, 10 },
};

static int parse_binary(struct exprctx *x, int minprec, struct value *v);

void value_num(struct value *v, int64_t n)
{
	memset(v, 0, sizeof(*v));
	v->kind = V_NUM;
	v->n = n;
	v->base = -1;
}

void value_reg(struct value *v, enum regclass rc, int64_t n)
{
	value_num(v, n);
	v->kind = V_REG;
	v->rc = rc;
}

int value_known(const struct value *v)
{
	return !(v->flags & VF_UNDEF);
}

static int all_digits(const char *s, int base)
{
	if (!*s)
		return 0;
	for (; *s; s++)
		if (base == 10 ? !isdigit((unsigned char)*s) : !isxdigit((unsigned char)*s))
			return 0;
	return 1;
}

/* 1: register, 0: not a register name, -1: register name out of range. */
int parse_register(const char *name, struct value *v)
{
	char low[16];
	size_t i, n = strlen(name);
	long nr;

	if (n >= sizeof(low))
		return 0;
	for (i = 0; i <= n; i++)
		low[i] = tolower((unsigned char)name[i]);
	if (low[0] == 'r' && all_digits(low + 1, 10)) {
		nr = strtol(low + 1, NULL, 10);
		value_reg(v, RC_GPR, nr);
		return nr < NR_GPR ? 1 : -1;
	}
	if (strncmp(low, "spr", 3) == 0 && n <= 7 && all_digits(low + 3, 16)) {
		value_reg(v, RC_SPR, strtol(low + 3, NULL, 16));
		return 1;
	}
	if (strncmp(low, "off", 3) == 0 && all_digits(low + 3, 10)) {
		nr = strtol(low + 3, NULL, 10);
		value_reg(v, RC_OFF, nr);
		return nr < NR_OFFR ? 1 : -1;
	}
	if (strncmp(low, "lr", 2) == 0 && all_digits(low + 2, 10)) {
		nr = strtol(low + 2, NULL, 10);
		value_reg(v, RC_LR, nr);
		return nr < NR_LR ? 1 : -1;
	}
	return 0;
}

const struct token *ex_peek(struct exprctx *x)
{
	return &x->tl->t.v[x->i];
}

const struct token *ex_next(struct exprctx *x)
{
	const struct token *t = &x->tl->t.v[x->i];

	if (t->kind != T_EOL)
		x->i++;
	return t;
}

int ex_accept(struct exprctx *x, enum punct p)
{
	if (tok_is_punct(ex_peek(x), p)) {
		x->i++;
		return 1;
	}
	return 0;
}

int ex_at_end(struct exprctx *x)
{
	return ex_peek(x)->kind == T_EOL;
}

static int expect(struct exprctx *x, enum punct p, const char *what)
{
	if (ex_accept(x, p))
		return 0;
	diag_error(x->loc, "expected %s", what);
	return -1;
}

static int find_binop(const struct token *t, enum binop *op, int *prec)
{
	size_t k;

	for (k = 0; k < ARRAY_SIZE(binops); k++) {
		if ((t->kind == T_PUNCT && t->p == binops[k].p) ||
		    (binops[k].kw && tok_is_kw(t, binops[k].kw))) {
			*op = binops[k].op;
			*prec = binops[k].prec;
			return 1;
		}
	}
	return 0;
}

static void strip_meta(struct value *v)
{
	v->type = NULL;
	v->count = 0;
	v->rf = NULL;
	v->sym = NULL;
}

static int reg_arith(struct exprctx *x, enum binop op, struct value *a,
		     const struct value *b)
{
	const struct value *reg = a->kind == V_REG ? a : b;
	const struct value *num = a->kind == V_REG ? b : a;
	int64_t delta;
	unsigned int undef = (a->flags | b->flags) & VF_UNDEF;

	if (a->kind == V_REG && b->kind == V_REG) {
		if (op == B_EQ || op == B_NE) {
			int same = a->rc == b->rc && a->n == b->n;

			value_num(a, op == B_EQ ? same : !same);
			a->flags |= undef;
			return 0;
		}
		goto bad;
	}
	if (op != B_ADD && !(op == B_SUB && a->kind == V_REG))
		goto bad;
	if (num->base >= 0)
		goto bad;
	delta = op == B_SUB ? -num->n : num->n;
	if (reg->rc == RC_OFF) {
		struct value r;

		if (!x->addr_mode)
			goto bad;
		value_num(&r, delta);
		r.base = reg->n;
		r.flags = (num->flags & ~VF_MEM) | undef;
		r.type = num->type;
		*a = r;
		return 0;
	}
	if (reg->rc == RC_LR)
		goto bad;
	{
		struct value r = *reg;

		r.n += delta;
		r.flags |= undef;
		r.sym = NULL;
		*a = r;
	}
	return 0;
bad:
	diag_error(x->loc, "invalid arithmetic on a register");
	return -1;
}

static int apply_binop(struct exprctx *x, enum binop op, struct value *a,
		       const struct value *b)
{
	unsigned int undef = (a->flags | b->flags) & VF_UNDEF;
	unsigned int la = a->flags & (VF_MEM | VF_CODE);
	unsigned int lb = b->flags & (VF_MEM | VF_CODE);
	int64_t l = a->n, r = b->n, res = 0;
	struct value out;

	if (a->kind == V_REG || b->kind == V_REG)
		return reg_arith(x, op, a, b);
	if ((a->base >= 0 || b->base >= 0) &&
	    !(op == B_ADD && (a->base < 0 || b->base < 0)) &&
	    !(op == B_SUB && b->base < 0)) {
		diag_error(x->loc, "invalid arithmetic on an indexed address");
		return -1;
	}
	switch (op) {
	case B_LOR: res = l || r; break;
	case B_LAND: res = l && r; break;
	case B_OR: res = l | r; break;
	case B_XOR: res = l ^ r; break;
	case B_AND: res = l & r; break;
	case B_EQ: res = l == r; break;
	case B_NE: res = l != r; break;
	case B_LT: res = l < r; break;
	case B_LE: res = l <= r; break;
	case B_GT: res = l > r; break;
	case B_GE: res = l >= r; break;
	case B_SHL:
	case B_SHR:
		if (r < 0 || r > 63) {
			if (!undef) {
				diag_error(x->loc, "shift count %lld out of range", (long long)r);
				return -1;
			}
			r = 0;
		}
		res = op == B_SHL ? (int64_t)((uint64_t)l << r) : l >> r;
		break;
	case B_ADD: res = l + r; break;
	case B_SUB: res = l - r; break;
	case B_MUL: res = l * r; break;
	case B_DIV:
	case B_MOD:
		if (r == 0) {
			if (!undef) {
				diag_error(x->loc, "division by zero");
				return -1;
			}
			break;
		}
		res = op == B_DIV ? l / r : l % r;
		break;
	}
	value_num(&out, res);
	out.flags = undef;
	if (op == B_ADD || op == B_SUB) {
		const struct value *typed = a->type ? a : (op == B_ADD ? b : NULL);

		out.base = a->base >= 0 ? a->base : b->base;
		if (!(op == B_SUB && la && la == lb)) {
			out.flags |= la | (op == B_ADD ? lb : 0);
			if (typed && (typed->flags & (VF_MEM | VF_TYPE))) {
				out.type = typed->type;
				out.flags |= typed->flags & VF_TYPE;
			}
		}
	}
	*a = out;
	return 0;
}

static int member(struct exprctx *x, struct value *v, const char *name)
{
	const struct type *t = v->type;

	if (!t || (t->kind != TY_STRUCT && t->kind != TY_UNION && t->kind != TY_RECORD)) {
		diag_error(x->loc, "'.%s' applied to a value without a STRUC or RECORD type", name);
		return -1;
	}
	if (t->kind == TY_RECORD) {
		const struct rfield *rf = type_rfield(t, name);
		unsigned int undef = v->flags & VF_UNDEF;

		if (!rf) {
			diag_error(x->loc, "record %s has no field '%s'", t->name, name);
			return -1;
		}
		value_num(v, rf->shift);
		v->flags = VF_RFIELD | undef;
		v->rf = rf;
		return 0;
	} else {
		const struct field *f = type_field(t, name);

		if (!f) {
			diag_error(x->loc, "%s has no field '%s'", t->name, name);
			return -1;
		}
		v->n += f->offset;
		v->type = f->type;
		v->count = f->count;
		v->sym = NULL;
	}
	return 0;
}

static int parse_postfix(struct exprctx *x, struct value *v);

static int parse_reg_ctor(struct exprctx *x, enum regclass rc, struct value *v)
{
	struct value arg;
	static const int limit[] = { NR_GPR, 0x400, NR_OFFR, NR_LR };

	if (expect(x, P_LPAREN, "'('") || parse_binary(x, 1, &arg) ||
	    expect(x, P_RPAREN, "')'"))
		return -1;
	if (arg.kind != V_NUM) {
		diag_error(x->loc, "register number must be a number");
		return -1;
	}
	value_reg(v, rc, arg.n);
	v->flags = arg.flags & VF_UNDEF;
	if (value_known(&arg) && (arg.n < 0 || arg.n >= limit[rc])) {
		diag_error(x->loc, "register number %lld out of range", (long long)arg.n);
		return -1;
	}
	return 0;
}

static int parse_primary(struct exprctx *x, struct value *v)
{
	const struct token *t = ex_next(x);
	int r;

	switch (t->kind) {
	case T_NUM:
		value_num(v, t->num);
		return 0;
	case T_STR: {
		size_t i, n = strlen(t->s);
		int64_t acc = 0;

		if (n == 0 || n > 8) {
			diag_error(x->loc, "character constant must be 1..8 characters");
			return -1;
		}
		for (i = 0; i < n; i++)
			acc = (acc << 8) | (unsigned char)t->s[i];
		value_num(v, acc);
		return 0;
	}
	case T_PUNCT:
		if (t->p == P_LPAREN) {
			if (parse_binary(x, 1, v))
				return -1;
			return expect(x, P_RPAREN, "')'");
		}
		break;
	case T_IDENT:
		if (strcmp(t->s, "$") == 0)
			return asm_here(v, x->loc);
		if (strieq(t->s, "@B") || strieq(t->s, "@F"))
			return asm_anon_label(strieq(t->s, "@F"), v, x->loc);
		if (strieq(t->s, "DEFINED") && tok_is_punct(ex_peek(x), P_LPAREN)) {
			const struct token *id;

			x->i++;
			id = ex_next(x);
			if (id->kind != T_IDENT) {
				diag_error(x->loc, "DEFINED needs a symbol name");
				return -1;
			}
			value_num(v, asm_defined(id->s));
			return expect(x, P_RPAREN, "')'");
		}
		if (tok_is_punct(ex_peek(x), P_LPAREN)) {
			if (strieq(t->s, "GPR"))
				return parse_reg_ctor(x, RC_GPR, v);
			if (strieq(t->s, "SPR"))
				return parse_reg_ctor(x, RC_SPR, v);
			if (strieq(t->s, "OFFR"))
				return parse_reg_ctor(x, RC_OFF, v);
		}
		r = parse_register(t->s, v);
		if (r < 0) {
			diag_error(x->loc, "register %s out of range", t->s);
			return -1;
		}
		if (r > 0)
			return 0;
		return asm_lookup(t->s, v, x->loc);
	default:
		break;
	}
	diag_error(x->loc, t->kind == T_EOL ? "expression expected" :
		   "unexpected token in expression");
	return -1;
}

static int parse_postfix(struct exprctx *x, struct value *v)
{
	if (parse_primary(x, v))
		return -1;
	while (tok_is_punct(ex_peek(x), P_DOT)) {
		const struct token *name;

		x->i++;
		name = ex_next(x);
		if (name->kind != T_IDENT) {
			diag_error(x->loc, "field name expected after '.'");
			return -1;
		}
		if (member(x, v, name->s))
			return -1;
	}
	return 0;
}

static int need_num(struct exprctx *x, const struct value *v, const char *op)
{
	if (v->kind == V_NUM && v->base < 0)
		return 0;
	diag_error(x->loc, "%s needs a numeric operand", op);
	return -1;
}

static int type_operator(struct exprctx *x, const char *op, struct value *v)
{
	const struct token *t = ex_peek(x);
	unsigned int undef;
	unsigned int n;

	if (strieq(op, "SIZEOF") && t->kind == T_IDENT &&
	    !tok_is_punct(&x->tl->t.v[x->i + 1], P_DOT) &&
	    asm_segment_size(t->s, v) == 0) {
		x->i++;
		return 0;
	}
	if (parse_postfix(x, v))
		return -1;
	undef = v->flags & VF_UNDEF;
	if (strieq(op, "WIDTH") || strieq(op, "MASK")) {
		unsigned int width, shift;

		if (v->flags & VF_RFIELD) {
			width = v->rf->width;
			shift = v->rf->shift;
		} else if (v->type && v->type->kind == TY_RECORD) {
			width = v->type->width;
			shift = 0;
		} else {
			diag_error(x->loc, "%s needs a RECORD or a record field", op);
			return -1;
		}
		value_num(v, strieq(op, "WIDTH") ? width :
			  (int64_t)(((1u << width) - 1) << shift));
		v->flags = undef;
		return 0;
	}
	if (!v->type) {
		diag_error(x->loc, "%s needs a typed operand", op);
		return -1;
	}
	n = v->count ? v->count : 1;
	if (strieq(op, "LENGTHOF") || strieq(op, "LENGTH"))
		value_num(v, n);
	else if (strieq(op, "TYPE"))
		value_num(v, v->type->size);
	else
		value_num(v, (int64_t)n * v->type->size);
	v->flags = undef;
	return 0;
}

static int parse_unary(struct exprctx *x, struct value *v)
{
	const struct token *t = ex_peek(x);
	static const char *const typeops[] = {
		"SIZEOF", "SIZE", "LENGTHOF", "LENGTH", "TYPE", "WIDTH", "MASK",
	};
	size_t k;

	if (t->kind == T_PUNCT && (t->p == P_MINUS || t->p == P_PLUS ||
				   t->p == P_TILDE || t->p == P_BANG)) {
		enum punct p = t->p;

		x->i++;
		if (parse_unary(x, v) || need_num(x, v, "unary operator"))
			return -1;
		if (p == P_MINUS)
			v->n = -v->n;
		else if (p == P_TILDE)
			v->n = ~v->n;
		else if (p == P_BANG)
			v->n = !v->n;
		v->flags &= VF_UNDEF;
		strip_meta(v);
		return 0;
	}
	if (t->kind != T_IDENT)
		return parse_postfix(x, v);
	for (k = 0; k < ARRAY_SIZE(typeops); k++) {
		if (strieq(t->s, typeops[k])) {
			x->i++;
			return type_operator(x, typeops[k], v);
		}
	}
	if (strieq(t->s, "NOT") || strieq(t->s, "LOW") || strieq(t->s, "HIGH") ||
	    strieq(t->s, "LOWWORD") || strieq(t->s, "HIGHWORD")) {
		const char *op = t->s;

		x->i++;
		if (parse_unary(x, v) || need_num(x, v, op))
			return -1;
		if (strieq(op, "NOT"))
			v->n = ~v->n;
		else if (strieq(op, "LOW"))
			v->n &= 0xFF;
		else if (strieq(op, "HIGH"))
			v->n = (v->n >> 8) & 0xFF;
		else if (strieq(op, "LOWWORD"))
			v->n &= 0xFFFF;
		else
			v->n = (v->n >> 16) & 0xFFFF;
		v->flags &= VF_UNDEF;
		strip_meta(v);
		return 0;
	}
	if (strieq(t->s, "OFFSET")) {
		x->i++;
		if (parse_unary(x, v) || need_num(x, v, "OFFSET"))
			return -1;
		v->flags &= VF_UNDEF;
		v->sym = NULL;
		return 0;
	}
	return parse_postfix(x, v);
}

static int parse_binary(struct exprctx *x, int minprec, struct value *v)
{
	if (parse_unary(x, v))
		return -1;
	for (;;) {
		enum binop op;
		int prec;
		struct value rhs;

		if (!find_binop(ex_peek(x), &op, &prec) || prec < minprec)
			return 0;
		x->i++;
		if (parse_binary(x, prec + 1, &rhs))
			return -1;
		if (apply_binop(x, op, v, &rhs))
			return -1;
		v->sym = NULL;
	}
}

int expr_parse(struct exprctx *x, struct value *v)
{
	return parse_binary(x, 1, v);
}
