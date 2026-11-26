#include "masm.h"

#include <stdlib.h>
#include <string.h>

static void regset_add(struct regset *rs, const struct value *reg)
{
	if (reg->rc == RC_GPR)
		rs->gpr |= (uint64_t)1 << reg->n;
	else if (reg->rc == RC_OFF)
		rs->off |= 1u << reg->n;
}

int regset_has(const struct regset *rs, const struct value *reg)
{
	if (reg->kind != V_REG)
		return 1;
	if (reg->rc == RC_GPR)
		return !!(rs->gpr & ((uint64_t)1 << reg->n));
	if (reg->rc == RC_OFF)
		return !!(rs->off & (1u << reg->n));
	return 1;
}

void regset_format(const struct regset *rs, struct strbuf *sb)
{
	int i = 0, j;

	while (i < NR_GPR) {
		if (!(rs->gpr & ((uint64_t)1 << i))) {
			i++;
			continue;
		}
		for (j = i; j + 1 < NR_GPR && (rs->gpr & ((uint64_t)1 << (j + 1))); j++)
			;
		if (sb->len)
			sb_putc(sb, ' ');
		if (j > i)
			sb_printf(sb, "r%d..r%d", i, j);
		else
			sb_printf(sb, "r%d", i);
		i = j + 1;
	}
	for (i = 0; i < NR_OFFR; i++) {
		if (rs->off & (1u << i)) {
			if (sb->len)
				sb_putc(sb, ' ');
			sb_printf(sb, "off%d", i);
		}
	}
}

static int tracked_reg(struct exprctx *x, struct value *v, int strict,
		       const char *what)
{
	if (expr_parse(x, v))
		return -1;
	if (!value_known(v)) {
		if (strict) {
			diag_error(x->loc, "%s register is not known", what);
			return -1;
		}
		return 1;
	}
	if (v->kind != V_REG || (v->rc != RC_GPR && v->rc != RC_OFF)) {
		diag_error(x->loc, "%s must be a GPR or an offset register", what);
		return -1;
	}
	if (v->n < 0 || (v->rc == RC_GPR ? v->n >= NR_GPR : v->n >= NR_OFFR)) {
		diag_error(x->loc, "%s register out of range", what);
		return -1;
	}
	return 0;
}

static int parse_uses(struct exprctx *x, struct contract *c, int strict)
{
	if (tok_is_kw(ex_peek(x), "ALL")) {
		x->i++;
		c->uses_all = 1;
		return 0;
	}
	while (!ex_at_end(x) && !tok_is_punct(ex_peek(x), P_COMMA) &&
	       !tok_is_kw(ex_peek(x), "LINK")) {
		struct value a, b;
		int r = tracked_reg(x, &a, strict, "USES");

		if (r < 0)
			return -1;
		if (ex_accept(x, P_DOTDOT)) {
			int r2 = tracked_reg(x, &b, strict, "USES");
			int64_t i;

			if (r2 < 0)
				return -1;
			if (r || r2)
				continue;
			if (a.rc != b.rc || b.n < a.n) {
				diag_error(x->loc, "invalid register range");
				return -1;
			}
			for (i = a.n; i <= b.n; i++) {
				a.n = i;
				regset_add(&c->regs, &a);
			}
			continue;
		}
		if (r == 0)
			regset_add(&c->regs, &a);
	}
	return 0;
}

static int parse_named(struct exprctx *x, paramvec_t *out, int strict,
		       int with_dir)
{
	struct param p;
	const struct token *t;
	int r;

	memset(&p, 0, sizeof(p));
	p.dir = PD_IN;
	t = ex_peek(x);
	if (with_dir && t->kind == T_IDENT && x->tl->t.v[x->i + 1].kind == T_IDENT) {
		if (strieq(t->s, "IN"))
			p.dir = PD_IN;
		else if (strieq(t->s, "OUT"))
			p.dir = PD_OUT;
		else if (strieq(t->s, "INOUT"))
			p.dir = PD_INOUT;
		else
			goto bad;
		x->i++;
		t = ex_peek(x);
	}
	if (t->kind != T_IDENT)
		goto bad;
	p.name = xstrdup(t->s);
	x->i++;
	if (!ex_accept(x, P_COLON))
		goto bad;
	r = tracked_reg(x, &p.reg, strict, p.name);
	if (r < 0) {
		free(p.name);
		return -1;
	}
	if (r > 0) {
		value_num(&p.reg, 0);
		p.reg.flags = VF_UNDEF;
	}
	VEC_PUSH(*out, p);
	return 0;
bad:
	diag_error(x->loc, "expected %sname:register", with_dir ? "[IN|OUT|INOUT] " : "");
	return -1;
}

static int parse_header(const char *text, const struct srcloc *loc,
			struct contract *c, int strict)
{
	struct toklist tl = { 0 };
	struct exprctx x = { &tl, 0, loc, 0 };
	int r = -1;

	if (lex_line(text, &tl, loc))
		goto out;
	while (!ex_at_end(&x) && !tok_is_punct(ex_peek(&x), P_COMMA)) {
		const struct token *t = ex_next(&x);

		if (tok_is_kw(t, "LINK")) {
			struct value v;

			if (expr_parse(&x, &v))
				goto out;
			if (v.kind != V_REG || v.rc != RC_LR) {
				diag_error(loc, "LINK needs a link register (lr0..lr3)");
				goto out;
			}
			c->link = (int)v.n;
		} else if (tok_is_kw(t, "USES")) {
			if (parse_uses(&x, c, strict))
				goto out;
		} else {
			diag_error(loc, "unknown PROC attribute '%s'", t->s ? t->s : "?");
			goto out;
		}
	}
	while (ex_accept(&x, P_COMMA))
		if (parse_named(&x, &c->params, strict, 1))
			goto out;
	if (!ex_at_end(&x)) {
		diag_error(loc, "junk at end of PROC header");
		goto out;
	}
	r = 0;
out:
	toklist_free(&tl);
	return r;
}

int contract_locals_line(const struct bodyline *bl, paramvec_t *out, int strict)
{
	struct toklist tl = { 0 };
	struct exprctx x = { &tl, 0, bl->loc, 0 };
	int r = -1;

	if (lex_line(bl->text, &tl, bl->loc))
		goto out;
	do {
		if (parse_named(&x, out, strict, 0))
			goto out;
	} while (ex_accept(&x, P_COMMA));
	if (!ex_at_end(&x)) {
		diag_error(bl->loc, "junk at end of LOCAL");
		goto out;
	}
	r = 0;
out:
	toklist_free(&tl);
	return r;
}

int contract_eval(struct proc *p, struct contract *c, int strict)
{
	size_t i;

	memset(c, 0, sizeof(*c));
	if (parse_header(p->header, p->header_loc, c, strict))
		return -1;
	for (i = 0; i < p->locals.n; i++)
		if (contract_locals_line(&p->locals.v[i], &c->locals, strict))
			return -1;
	for (i = 0; i < c->params.n; i++)
		if (value_known(&c->params.v[i].reg))
			regset_add(&c->regs, &c->params.v[i].reg);
	for (i = 0; i < c->locals.n; i++)
		if (value_known(&c->locals.v[i].reg))
			regset_add(&c->regs, &c->locals.v[i].reg);
	return 0;
}

static int params_equal(const struct param *a, const struct param *b, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		if (strcmp(a[i].name, b[i].name) || a[i].dir != b[i].dir ||
		    a[i].reg.rc != b[i].reg.rc || a[i].reg.n != b[i].reg.n)
			return 0;
	return 1;
}

int contract_equal(const struct contract *a, const struct contract *b)
{
	return a->link == b->link && a->uses_all == b->uses_all &&
	       a->regs.gpr == b->regs.gpr && a->regs.off == b->regs.off &&
	       a->params.n == b->params.n && a->locals.n == b->locals.n &&
	       params_equal(a->params.v, b->params.v, a->params.n) &&
	       params_equal(a->locals.v, b->locals.v, a->locals.n);
}

void contract_free(struct contract *c)
{
	size_t i;

	for (i = 0; i < c->params.n; i++)
		free(c->params.v[i].name);
	for (i = 0; i < c->locals.n; i++)
		free(c->locals.v[i].name);
	free(c->params.v);
	free(c->locals.v);
	memset(c, 0, sizeof(*c));
}

static int walk(struct proc *p, int arch, struct proc **stack, size_t sp)
{
	size_t i;

	if (p->walk == 2)
		return 0;
	if (p->walk == 1) {
		struct strbuf sb = { 0 };
		size_t k = sp;

		while (k > 0 && stack[k - 1] != p)
			k--;
		for (k = k ? k - 1 : 0; k < sp; k++)
			sb_printf(&sb, "%s -> ", stack[k]->name);
		sb_puts(&sb, p->name);
		diag_error(p->loc, "recursive call chain: %s", sb_cstr(&sb));
		free(sb.s);
		return -1;
	}
	p->walk = 1;
	stack[sp] = p;
	p->linkset = (p->is_top || arch != 5) ? 0 : 1u << p->c.link;
	p->depth = 0;
	for (i = 0; i < p->calls.n; i++) {
		struct proc *q = p->calls.v[i].callee;
		int d;

		if (!q)
			continue;
		if (walk(q, arch, stack, sp + 1)) {
			p->walk = 2;
			return -1;
		}
		p->linkset |= q->linkset;
		d = q->depth + (p->calls.v[i].tail ? 0 : 1);
		if (d > p->depth)
			p->depth = d;
	}
	p->walk = 2;
	return 0;
}

void proc_analyze(struct proc **procs, size_t n, int arch)
{
	struct proc **stack = xcalloc(n + 1, sizeof(*stack));
	size_t i;

	for (i = 0; i < n; i++)
		procs[i]->walk = 0;
	for (i = 0; i < n; i++)
		walk(procs[i], arch, stack, 0);
	free(stack);
}

void proc_check_write(struct proc *p, const struct value *dst,
		      const struct srcloc *loc)
{
	if (p->is_top || p->c.uses_all || regset_has(&p->c.regs, dst))
		return;
	diag_error(loc, "PROC %s writes %s%lld, which its contract does not declare",
		   p->name, dst->rc == RC_GPR ? "r" : "off", (long long)dst->n);
}

void proc_check_call(struct proc *caller, struct proc *callee, int tail,
		     int raw_link, int arch, const struct srcloc *loc)
{
	if (arch == 5 && raw_link >= 0 && raw_link != callee->c.link)
		diag_error(loc, "%s returns through lr%d, but is called through lr%d",
			   callee->name, callee->c.link, raw_link);
	if (caller->is_top)
		return;
	if (arch == 5) {
		if (tail && callee->c.link != caller->c.link)
			diag_error(loc, "tail jump from %s (lr%d) to %s, which returns through lr%d",
				   caller->name, caller->c.link, callee->name, callee->c.link);
		if (!tail && (callee->linkset & (1u << caller->c.link)))
			diag_error(loc, "calling %s overwrites lr%d, which holds the return address of %s",
				   callee->name, caller->c.link, caller->name);
	}
	if (caller->c.uses_all)
		return;
	if (callee->c.uses_all) {
		diag_error(loc, "%s has no contract (USES ALL) and cannot be called from %s",
			   callee->name, caller->name);
		return;
	}
	{
		struct regset miss;

		miss.gpr = callee->c.regs.gpr & ~caller->c.regs.gpr;
		miss.off = callee->c.regs.off & ~caller->c.regs.off;
		if (miss.gpr || miss.off) {
			struct strbuf sb = { 0 };

			regset_format(&miss, &sb);
			diag_error(loc, "%s %s clobbers %s, not in the contract of %s",
				   tail ? "jumping to" : "calling", callee->name,
				   sb_cstr(&sb), caller->name);
			free(sb.s);
		}
	}
}
