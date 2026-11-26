#include "asm.h"
#include "isa.h"
#include "macro.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define MAX_PASSES	8
#define MAX_EXPAND	32
#define MAX_OPERANDS	24

enum blk_kind {
	BK_SEGMENT,
	BK_STRUCT,
	BK_INITVALS,
};

struct block {
	enum blk_kind kind;
	char *name;			/* NULL for an anonymous nested STRUC/UNION */
	const struct srcloc *loc;
	struct segment *prev_seg;
	struct type *type;
	int is_union, member_done;
	unsigned int base, cur, max;
	struct ivsect *iv;
};

struct cond {
	int active, done, parent_active, seen_else;
	const struct srcloc *loc;
};

enum cap_kind {
	CAP_NONE,
	CAP_MACRO,
	CAP_REPT,
	CAP_FOR,
	CAP_FORC,
	CAP_WHILE,
};

struct capture {
	enum cap_kind kind;
	int depth;
	char *name, *args;
	const struct srcloc *loc;
	body_t body;
};

struct anon {
	struct segment *seg;
	unsigned int off;
};

typedef VEC(struct anon) anonvec_t;

static struct {
	const struct asm_opts *opts;
	int pass;
	int arch, corerev;
	struct symtab *global, *global_prev;
	struct proc *top, *proc;
	struct segment *seg, *text;
	VEC(struct segment *) segs;
	VEC(struct proc *) procs;
	VEC(struct ivsect *) ivs;
	VEC(struct cond) cond;
	VEC(struct block) blocks;
	struct capture cap;
	anonvec_t anon, anon_prev;
	unsigned int uniq;
	int ended, changed, last_jump, expand_depth;
	char *entry;
	const struct srcloc *entry_loc;
	unsigned int memlimit[NR_SPACES];
	int stackdepth;
	const struct srcloc *stackdepth_loc;
	VEC(struct listent) listing;
} as;

static void process_line(const struct line *ln);

static const char *rest(const struct toklist *tl, size_t i)
{
	return tl->line + tl->t.v[i].pos;
}

static char *trimdup(const char *s)
{
	size_t n;

	while (isspace((unsigned char)*s))
		s++;
	n = strlen(s);
	while (n && isspace((unsigned char)s[n - 1]))
		n--;
	return xstrndup(s, n);
}

static char *trimndup(const char *s, size_t n)
{
	char *t = xstrndup(s, n);
	char *r = trimdup(t);

	free(t);
	return r;
}

static const char *word(const char *s, char *w, size_t size)
{
	size_t n = 0;

	while (isspace((unsigned char)*s))
		s++;
	if (*s == '.' || is_ident_start((unsigned char)*s)) {
		do {
			if (n + 1 < size)
				w[n++] = *s;
			s++;
		} while (is_ident_char((unsigned char)*s));
	}
	w[n] = '\0';
	return s;
}

static int in_block(enum blk_kind k)
{
	return as.blocks.n && as.blocks.v[as.blocks.n - 1].kind == k;
}

static int in_proc(void)
{
	return as.proc && !as.proc->is_top;
}

/* Symbols */

static struct sym *prev_sym(const char *name, int local)
{
	if (local)
		return as.proc ? sym_get(as.proc->scope_prev, name) : NULL;
	return sym_get(as.global_prev, name);
}

static struct sym *define(const char *name, enum sym_kind kind, int global,
			  const struct srcloc *loc)
{
	int local = !global && in_proc();
	struct symtab *t = local ? as.proc->scope : as.global;
	struct sym *s = sym_get(t, name);
	struct value rv;

	if (parse_register(name, &rv) != 0 || !strcmp(name, "$") || !strcmp(name, "?")) {
		diag_error(loc, "'%s' is a reserved name", name);
		return NULL;
	}
	if (s) {
		if (s->kind != kind || (kind != S_REDEF && kind != S_TEXT && kind != S_MACRO)) {
			diag_error(loc, "'%s' already defined at %s:%d", name,
				   s->loc->file, s->loc->line);
			return NULL;
		}
		sym_remove(t, name);
	}
	return sym_put(t, name, kind, loc);
}

/* Records whether a located symbol moved since the previous pass. */
static void track(const struct sym *s)
{
	struct sym *p;

	if (as.pass < 2)
		return;
	p = prev_sym(s->name, in_proc() && sym_get(as.proc->scope, s->name) == s);
	if (!p || p->kind != s->kind || p->seg != s->seg || p->off != s->off)
		as.changed = 1;
}

static struct sym *find_sym(const char *name, int *prev)
{
	struct sym *s = NULL;

	*prev = 0;
	if (in_proc())
		s = sym_get(as.proc->scope, name);
	if (!s)
		s = sym_get(as.global, name);
	if (s || as.pass < 2)
		return s;
	if (in_proc())
		s = sym_get(as.proc->scope_prev, name);
	if (!s)
		s = sym_get(as.global_prev, name);
	if (s && (s->kind == S_LABEL || s->kind == S_DATA || s->kind == S_CONST ||
		  s->kind == S_REDEF || s->kind == S_PROC)) {
		*prev = 1;
		return s;
	}
	return NULL;
}

static void seg_location(const struct segment *seg, unsigned int off, struct value *v)
{
	int64_t a = seg->base + off;

	if (seg->space == SP_GPR)
		value_reg(v, RC_GPR, a);
	else
		value_num(v, a);
	if (!seg->placed)
		v->flags |= VF_UNDEF;
	if (seg->space == SP_CODE)
		v->flags |= VF_CODE;
	else if (seg->space == SP_SHM)
		v->flags |= VF_MEM;
}

static int sym_value(const struct sym *s, int prev, struct value *v,
		     const struct srcloc *loc)
{
	int r;

	switch (s->kind) {
	case S_CONST:
	case S_REDEF:
		*v = s->val;
		if (prev && !value_known(v)) {
			diag_error(loc, "'%s' is used before its definition and was not known in the previous pass",
				   s->name);
			return -1;
		}
		break;
	case S_TEXT:
		if (++as.expand_depth > MAX_EXPAND) {
			diag_error(loc, "text macro %s expands recursively", s->name);
			as.expand_depth--;
			return -1;
		}
		r = eval_text(s->text, v, loc);
		as.expand_depth--;
		return r;
	case S_LABEL:
	case S_PROC:
		seg_location(s->seg, s->off, v);
		if (s->seg->space != SP_CODE)
			v->type = &type_word;
		break;
	case S_DATA:
		seg_location(s->seg, s->off, v);
		v->type = s->type;
		v->count = s->count;
		break;
	case S_TYPE:
		value_num(v, 0);
		v->flags = VF_TYPE;
		v->type = s->type;
		break;
	case S_SEGMENT:
		seg_location(s->seg, 0, v);
		break;
	case S_MACRO:
		diag_error(loc, "macro %s used as a value", s->name);
		return -1;
	}
	v->sym = s;
	return 0;
}

int asm_lookup(const char *name, struct value *v, const struct srcloc *loc)
{
	int prev;
	struct sym *s = find_sym(name, &prev);

	if (!s) {
		value_num(v, 0);
		v->flags = VF_UNDEF;
		if (as.pass >= 2) {
			diag_error(loc, "undefined symbol '%s'", name);
			return -1;
		}
		return 0;
	}
	return sym_value(s, prev, v, loc);
}

int asm_defined(const char *name)
{
	return (in_proc() && sym_get(as.proc->scope, name)) || sym_get(as.global, name);
}

int asm_here(struct value *v, const struct srcloc *loc)
{
	if (!as.seg) {
		diag_error(loc, "'$' outside a segment");
		return -1;
	}
	seg_location(as.seg, as.seg->cur, v);
	return 0;
}

int asm_anon_label(int forward, struct value *v, const struct srcloc *loc)
{
	const struct anon *a = NULL;

	if (!forward && as.anon.n)
		a = &as.anon.v[as.anon.n - 1];
	else if (forward && as.anon.n < as.anon_prev.n)
		a = &as.anon_prev.v[as.anon.n];
	if (!a) {
		value_num(v, 0);
		v->flags = VF_UNDEF;
		if (!forward || as.pass >= 2) {
			diag_error(loc, "no @@ label %s", forward ? "after @F" : "before @B");
			return -1;
		}
		return 0;
	}
	seg_location(a->seg, a->off, v);
	return 0;
}

int asm_segment_size(const char *name, struct value *v)
{
	struct sym *s = sym_get(as.global, name);

	if (!s)
		s = sym_get(as.global_prev, name);
	if (!s || s->kind != S_SEGMENT)
		return -1;
	value_num(v, s->seg->size_prev);
	if (as.pass < 2)
		v->flags = VF_UNDEF;
	return 0;
}

int asm_pass(void)
{
	return as.pass;
}

static const struct type *find_type(const char *name)
{
	int prev;
	struct sym *s = find_sym(name, &prev);

	return s && !prev && s->kind == S_TYPE ? s->type : NULL;
}

static struct macro *find_macro(const char *name)
{
	int prev;
	struct sym *s = find_sym(name, &prev);

	return s && !prev && s->kind == S_MACRO ? s->macro : NULL;
}

/* Segments */

static struct segment *seg_find(const char *name)
{
	size_t i;

	for (i = 0; i < as.segs.n; i++)
		if (strcmp(as.segs.v[i]->name, name) == 0)
			return as.segs.v[i];
	return NULL;
}

static struct ivsect *ivsect_get(const char *name)
{
	struct ivsect *iv;
	size_t i;

	for (i = 0; i < as.ivs.n; i++)
		if (strcmp(as.ivs.v[i]->name, name) == 0)
			return as.ivs.v[i];
	iv = xcalloc(1, sizeof(*iv));
	iv->name = xstrdup(name);
	VEC_PUSH(as.ivs, iv);
	return iv;
}

static const char *const space_names[] = { "CODE", "SHM", "GPR" };

/* Opens a segment; options are taken on the first opening in a pass. */
static struct segment *seg_open(const char *name, int space, int has_at, int64_t at,
				unsigned int align, const char *ivname,
				const struct srcloc *loc)
{
	struct segment *s = seg_find(name);
	struct sym *sym;

	if (!s) {
		if (space < 0) {
			diag_error(loc, "segment %s needs a space (CODE, SHM or GPR)", name);
			return NULL;
		}
		s = xcalloc(1, sizeof(*s));
		s->name = xstrdup(name);
		s->space = space;
		VEC_PUSH(as.segs, s);
	}
	if (space >= 0 && (int)s->space != space) {
		diag_error(loc, "segment %s was declared as %s", name, space_names[s->space]);
		return NULL;
	}
	if (s->seen_pass == as.pass) {
		if (has_at && (!s->has_at || s->at != at))
			diag_error(loc, "segment %s reopened with a different AT", name);
		if (ivname && (!s->ivname || strcmp(s->ivname, ivname)))
			diag_error(loc, "segment %s reopened with a different INIT", name);
		if (align > s->align)
			s->align = align;
		return s;
	}
	s->seen_pass = as.pass;
	s->loc = loc;
	s->has_at = has_at;
	s->at = at;
	s->align = align ? align : 1;
	free(s->ivname);
	s->ivname = ivname ? xstrdup(ivname) : NULL;
	if (ivname)
		ivsect_get(ivname);
	if (has_at) {
		if (s->placed && s->base != at)
			as.changed = 1;
		s->base = at;
		s->placed = 1;
	}
	sym = define(name, S_SEGMENT, 1, loc);
	if (sym)
		sym->seg = s;
	return s;
}

static void put_cell(uint64_t v, int set, const struct srcloc *loc)
{
	struct segment *s = as.seg;

	if (as.pass >= 2 && set) {
		while (s->cells.n <= s->cur) {
			struct cell c = { 0, 0 };

			VEC_PUSH(s->cells, c);
		}
		if (s->cells.v[s->cur].set)
			diag_error(loc, "%s+0x%X is written twice", s->name, s->cur);
		s->cells.v[s->cur].v = v;
		s->cells.v[s->cur].set = 1;
	}
	s->cur++;
	if (s->cur > s->size)
		s->size = s->cur;
}

static unsigned int space_limit(enum space sp)
{
	if (as.memlimit[sp])
		return as.memlimit[sp];
	switch (sp) {
	case SP_CODE:
		return isa_code_limit(as.arch);
	case SP_GPR:
		return NR_GPR;
	default:
		return 0x10000;
	}
}

static void layout(void)
{
	int64_t cursor[NR_SPACES] = { 0 };
	size_t i, j;

	for (i = 0; i < as.segs.n; i++) {
		struct segment *s = as.segs.v[i];
		int64_t base;

		if (s->has_at) {
			base = s->at;
			if (base % s->align)
				diag_error(s->loc, "AT 0x%llX of %s is not a multiple of its ALIGN %u",
					   (long long)base, s->name, s->align);
		} else {
			base = (cursor[s->space] + s->align - 1) / s->align * s->align;
		}
		if (s->size && base + s->size > cursor[s->space])
			cursor[s->space] = base + s->size;
		if (!s->placed || s->base != base || s->size != s->size_prev)
			as.changed = 1;
		s->base_prev = s->base;
		s->base = base;
		s->placed = 1;
		s->size_prev = s->size;
		if (s->size && base + s->size > space_limit(s->space))
			diag_error(s->loc, "%s (0x%llX..0x%llX) exceeds the %s space (0x%X)",
				   s->name, (long long)base, (long long)(base + s->size - 1),
				   space_names[s->space], space_limit(s->space));
	}
	for (i = 0; i < as.segs.n; i++) {
		struct segment *a = as.segs.v[i];

		for (j = i + 1; j < as.segs.n; j++) {
			struct segment *b = as.segs.v[j];

			if (a->space != b->space || !a->size || !b->size)
				continue;
			if (a->base < b->base + b->size && b->base < a->base + a->size)
				diag_error(b->loc, "segment %s overlaps %s in %s space",
					   b->name, a->name, space_names[a->space]);
		}
	}
}

/* Text macro expansion */

static char *invoke_function(struct macro *m, const char *args,
			     const struct srcloc *loc)
{
	body_t lines = { 0 };
	struct strbuf result = { 0 };
	struct line ln;
	size_t d;

	if (macro_expand(m, args, loc, &as.uniq, &lines))
		return NULL;
	d = src_depth();
	src_push_lines(PK_MACRO, &lines, as.cond.n, &result);
	while (!as.ended && src_next(&ln, d + 1))
		process_line(&ln);
	return result.s ? sb_steal(&result) : xstrdup("");
}

/* Index just past the ')' matching the '(' at s[i], or 0. */
static size_t paren_end(const char *s, size_t i)
{
	int depth = 0;
	char quote = 0;

	for (; s[i]; i++) {
		if (quote) {
			if (s[i] == quote)
				quote = 0;
		} else if (s[i] == '"' || s[i] == '\'') {
			quote = s[i];
		} else if (s[i] == '(') {
			depth++;
		} else if (s[i] == ')' && --depth == 0) {
			return i + 1;
		}
	}
	return 0;
}

/* Expands one identifier at s[i..j); returns -1 on failure. */
static int expand_ident(const char *s, size_t i, size_t j, int may_expand,
			struct strbuf *sb, size_t *next, const struct srcloc *loc);

static char *expand_text(const char *s, const struct srcloc *loc, int skip_first)
{
	struct strbuf sb = { 0 };
	size_t i = 0, j;
	char quote = 0;
	int first = 1;

	if (++as.expand_depth > MAX_EXPAND) {
		diag_error(loc, "text macro expansion nested too deeply");
		as.expand_depth--;
		return NULL;
	}
	while (s[i]) {
		char c = s[i];

		if (quote || c == '"' || c == '\'') {
			if (quote && c == quote)
				quote = 0;
			else if (!quote)
				quote = c;
			sb_putc(&sb, c);
			i++;
		} else if (isdigit((unsigned char)c)) {
			for (j = i; is_ident_char((unsigned char)s[j]); j++)
				;
			sb_putn(&sb, s + i, j - i);
			i = j;
		} else if (is_ident_start((unsigned char)c)) {
			int may = !(i > 0 && s[i - 1] == '.') && !(skip_first && first);

			for (j = i; is_ident_char((unsigned char)s[j]); j++)
				;
			first = 0;
			if (expand_ident(s, i, j, may, &sb, &i, loc)) {
				free(sb.s);
				as.expand_depth--;
				return NULL;
			}
		} else {
			sb_putc(&sb, c);
			i++;
		}
	}
	as.expand_depth--;
	return sb_steal(&sb);
}

static int expand_ident(const char *s, size_t i, size_t j, int may_expand,
			struct strbuf *sb, size_t *next, const struct srcloc *loc)
{
	char *name = xstrndup(s + i, j - i);
	struct sym *sym = NULL;
	size_t k = j, e;
	int prev, r = 0;

	while (isspace((unsigned char)s[k]))
		k++;
	e = s[k] == '(' ? paren_end(s, k) : 0;
	if (may_expand) {
		sym = find_sym(name, &prev);
		if (prev)
			sym = NULL;
	}
	*next = j;
	if (strieq(name, "DEFINED")) {
		if (e)
			*next = e;
		sb_putn(sb, s + i, *next - i);
	} else if (sym && sym->kind == S_TEXT) {
		char *x = expand_text(sym->text, loc, 0);

		if (x)
			sb_puts(sb, x);
		else
			r = -1;
		free(x);
	} else if (sym && sym->kind == S_MACRO && e) {
		char *args = xstrndup(s + k + 1, e - k - 2);
		char *res = invoke_function(sym->macro, args, loc);
		char *x = res ? expand_text(res, loc, 0) : NULL;

		if (x)
			sb_puts(sb, x);
		else
			r = -1;
		*next = e;
		free(args);
		free(res);
		free(x);
	} else {
		sb_puts(sb, name);
	}
	free(name);
	return r;
}

static int is_def_keyword(const char *w)
{
	static const char *const kw[] = {
		"EQU", "TEXTEQU", "MACRO", "STRUC", "STRUCT", "UNION", "RECORD",
		"PROC", "ENDP", "SEGMENT", "ENDS", "INITVALS",
	};
	size_t i;

	for (i = 0; i < ARRAY_SIZE(kw); i++)
		if (strieq(w, kw[i]))
			return 1;
	return 0;
}

static char *expand_line(const char *text, const struct srcloc *loc)
{
	static const char *const raw_kw[] = {
		"IFDEF", "IFNDEF", "ELSEIFDEF", "ELSEIFNDEF", ".ERRDEF", ".ERRNDEF", "PURGE",
	};
	char w1[64], w2[64];
	const char *p = word(text, w1, sizeof(w1));
	size_t i;

	word(p, w2, sizeof(w2));
	for (i = 0; i < ARRAY_SIZE(raw_kw); i++)
		if (strieq(w1, raw_kw[i]))
			return xstrdup(text);
	if (strieq(w2, "MACRO"))
		return xstrdup(text);
	while (isspace((unsigned char)*p))
		p++;
	return expand_text(text, loc, is_def_keyword(w2) || (p[0] == '=' && p[1] != '='));
}

/* Conditional assembly */

enum ckw {
	C_NONE,
	C_IF,
	C_ELSEIF,
	C_ELSE,
	C_ENDIF,
};

static enum ckw cond_kw(const char *w, const char **test)
{
	static const char *const tests[] = {
		"IF", "IFE", "IFDEF", "IFNDEF", "IFB", "IFNB",
		"IFIDN", "IFIDNI", "IFDIF", "IFDIFI",
	};
	size_t i;

	if (strieq(w, "ENDIF"))
		return C_ENDIF;
	if (strieq(w, "ELSE"))
		return C_ELSE;
	for (i = 0; i < ARRAY_SIZE(tests); i++) {
		*test = tests[i];
		if (strieq(w, tests[i]))
			return C_IF;
		if (strncasecmp(w, "ELSE", 4) == 0 && strieq(w + 4, tests[i]))
			return C_ELSEIF;
	}
	return C_NONE;
}

static int cond_active(void)
{
	return as.cond.n == 0 || as.cond.v[as.cond.n - 1].active;
}

static char *text_arg(const char *s)
{
	char *t = trimdup(s);
	char *in = strip_angle(t);

	if (!in)
		return t;
	free(t);
	return in;
}

static int eval_cond(const char *test, const struct toklist *tl, size_t i,
		     const struct srcloc *loc)
{
	if (strieq(test, "IF") || strieq(test, "IFE")) {
		struct exprctx x = { tl, i, loc, 0 };
		struct value v;

		if (expr_parse(&x, &v))
			return -1;
		if (!ex_at_end(&x)) {
			diag_error(loc, "junk after condition");
			return -1;
		}
		if (!value_known(&v) || v.kind != V_NUM) {
			diag_error(loc, "condition must be a number known at this point");
			return -1;
		}
		return strieq(test, "IF") ? v.n != 0 : v.n == 0;
	}
	if (strieq(test, "IFDEF") || strieq(test, "IFNDEF")) {
		const struct token *t = &tl->t.v[i];

		if (t->kind != T_IDENT || tl->t.v[i + 1].kind != T_EOL) {
			diag_error(loc, "%s needs one symbol name", test);
			return -1;
		}
		return strieq(test, "IFDEF") ? asm_defined(t->s) : !asm_defined(t->s);
	}
	if (strieq(test, "IFB") || strieq(test, "IFNB")) {
		char *a = text_arg(rest(tl, i));
		int blank;

		blank = a[0] == '\0';
		free(a);
		return strieq(test, "IFB") ? blank : !blank;
	} else {
		strvec_t items = { 0 };
		char *a, *b;
		int same, r;

		if (split_list(rest(tl, i), &items) || items.n != 2) {
			diag_error(loc, "%s needs two arguments", test);
			return -1;
		}
		a = text_arg(items.v[0]);
		b = text_arg(items.v[1]);
		same = (test[strlen(test) - 1] == 'I' || test[strlen(test) - 1] == 'i')
		       ? strieq(a, b) : strcmp(a, b) == 0;
		r = strncasecmp(test, "IFIDN", 5) == 0 ? same : !same;
		free(a);
		free(b);
		free(items.v[0]);
		free(items.v[1]);
		free(items.v);
		return r;
	}
}

static void do_cond(enum ckw kw, const char *test, const struct toklist *tl, size_t i,
		    const struct srcloc *loc)
{
	struct cond *top = as.cond.n ? &as.cond.v[as.cond.n - 1] : NULL;
	int r;

	switch (kw) {
	case C_IF: {
		struct cond c = { 0, 1, 0, 0, loc };

		if (cond_active()) {
			r = eval_cond(test, tl, i, loc);
			c.parent_active = 1;
			c.active = r > 0;
			c.done = r != 0;
		}
		VEC_PUSH(as.cond, c);
		return;
	}
	case C_ELSEIF:
		if (!top || top->seen_else) {
			diag_error(loc, top ? "ELSEIF after ELSE" : "ELSEIF without IF");
			return;
		}
		if (!top->parent_active || top->done) {
			top->active = 0;
			return;
		}
		r = eval_cond(test, tl, i, loc);
		top->active = r > 0;
		top->done = r != 0;
		return;
	case C_ELSE:
		if (!top || top->seen_else) {
			diag_error(loc, top ? "second ELSE" : "ELSE without IF");
			return;
		}
		top->seen_else = 1;
		top->active = top->parent_active && !top->done;
		top->done = 1;
		return;
	case C_ENDIF:
		if (!top) {
			diag_error(loc, "ENDIF without IF");
			return;
		}
		as.cond.n--;
		return;
	case C_NONE:
		break;
	}
}

static void on_popped(enum prov_kind kind, size_t base, const struct srcloc *end)
{
	if (as.cond.n > base) {
		diag_error(as.cond.v[base].loc, "IF without ENDIF %s",
			   kind == PK_FILE ? "before the end of the file" : "inside the expansion");
		as.cond.n = base;
	}
	(void)end;
}

static int on_eval(const char *expr, const struct srcloc *loc)
{
	char *e = expand_text(expr, loc, 0);
	struct value v;
	int r = 0;

	if (e && eval_text(e, &v, loc) == 0) {
		if (!value_known(&v) || v.kind != V_NUM)
			diag_error(loc, "WHILE condition must be a number known at this point");
		else
			r = v.n != 0;
	}
	free(e);
	return r;
}

/* Macro and repeat bodies */

static int is_repeat_kw(const char *w)
{
	return strieq(w, "REPT") || strieq(w, "REPEAT") || strieq(w, "FOR") ||
	       strieq(w, "IRP") || strieq(w, "FORC") || strieq(w, "IRPC") ||
	       strieq(w, "WHILE");
}

static void capture_start(enum cap_kind kind, const char *name, const char *args,
			  const struct srcloc *loc)
{
	as.cap.kind = kind;
	as.cap.depth = 1;
	as.cap.name = name ? xstrdup(name) : NULL;
	as.cap.args = xstrdup(args);
	as.cap.loc = loc;
	memset(&as.cap.body, 0, sizeof(as.cap.body));
}

static int split_param(const char *args, char **param, char **list,
		       const struct srcloc *loc)
{
	const char *comma = strchr(args, ',');

	if (!comma) {
		diag_error(loc, "expected 'parameter, list'");
		return -1;
	}
	*param = trimndup(args, comma - args);
	*list = trimdup(comma + 1);
	if (!**param || !is_ident_start((unsigned char)(*param)[0])) {
		diag_error(loc, "bad repeat parameter name");
		return -1;
	}
	return 0;
}

static void capture_finish(void)
{
	struct capture c = as.cap;
	body_t out = { 0 };
	strvec_t vals = { 0 };
	char *param = NULL, *list = NULL;
	struct value v;
	size_t i;

	memset(&as.cap, 0, sizeof(as.cap));
	switch (c.kind) {
	case CAP_MACRO: {
		struct macro *m = macro_define(c.name, c.args, &c.body, c.loc);
		struct sym *s;

		if (m && (s = define(c.name, S_MACRO, 1, c.loc)))
			s->macro = m;
		break;
	}
	case CAP_REPT:
		if (eval_text(c.args, &v, c.loc))
			break;
		if (!value_known(&v) || v.kind != V_NUM || v.n < 0 || v.n > 65536) {
			diag_error(c.loc, "REPT count must be a known number in 0..65536");
			break;
		}
		repeat_body(&c.body, NULL, NULL, (size_t)v.n, c.loc, "REPT", &out);
		src_push_lines(PK_REPEAT, &out, as.cond.n, NULL);
		break;
	case CAP_FOR:
	case CAP_FORC:
		if (split_param(c.args, &param, &list, c.loc))
			break;
		if (c.kind == CAP_FOR) {
			char *inner = strip_angle(list);

			if (!inner) {
				diag_error(c.loc, "FOR needs a <list>");
				break;
			}
			split_list(inner, &vals);
			for (i = 0; i < vals.n; i++) {
				char *a = strip_angle(vals.v[i]);

				if (a) {
					free(vals.v[i]);
					vals.v[i] = a;
				}
			}
			free(inner);
		} else {
			char *inner = strip_angle(list);
			const char *p = inner ? inner : list;

			for (; *p; p++)
				VEC_PUSH(vals, xstrndup(p, 1));
			free(inner);
		}
		repeat_body(&c.body, param, &vals, 0, c.loc,
			    c.kind == CAP_FOR ? "FOR" : "FORC", &out);
		src_push_lines(PK_REPEAT, &out, as.cond.n, NULL);
		break;
	case CAP_WHILE:
		src_push_while(c.args, c.loc, &c.body, as.cond.n);
		break;
	case CAP_NONE:
		break;
	}
	for (i = 0; i < vals.n; i++)
		free(vals.v[i]);
	free(vals.v);
	free(param);
	free(list);
	free(c.name);
	free(c.args);
}

static void capture_line(const struct line *ln)
{
	char w1[64], w2[64];
	struct bodyline bl;

	word(word(ln->text, w1, sizeof(w1)), w2, sizeof(w2));
	if (strieq(w1, "ENDM")) {
		if (--as.cap.depth == 0) {
			capture_finish();
			return;
		}
	} else if (is_repeat_kw(w1) || strieq(w2, "MACRO")) {
		as.cap.depth++;
	}
	bl.text = xstrdup(ln->text);
	bl.loc = ln->loc;
	VEC_PUSH(as.cap.body, bl);
}

/* Definitions */

static int eval_known(const struct toklist *tl, size_t *i, struct value *v,
		      const char *what, const struct srcloc *loc)
{
	struct exprctx x = { tl, *i, loc, 0 };

	if (expr_parse(&x, v))
		return -1;
	*i = x.i;
	if (!value_known(v) || v->kind != V_NUM) {
		diag_error(loc, "%s must be a number known at this point", what);
		return -1;
	}
	return 0;
}

static int at_end(const struct toklist *tl, size_t i, const struct srcloc *loc)
{
	if (tl->t.v[i].kind == T_EOL)
		return 1;
	diag_error(loc, "junk at end of line");
	return 0;
}

static void do_equ(const char *name, const char *text, const struct srcloc *loc)
{
	char *t = trimdup(text);
	char *inner = strip_angle(t);
	struct value v;
	struct sym *s;
	int r = -1;

	if (!inner) {
		diag_quiet(1);
		r = eval_text(t, &v, loc);
		diag_quiet(0);
	}
	if (r == 0) {
		struct sym *old = sym_get(in_proc() ? as.proc->scope : as.global, name);

		if (old && old->kind == S_CONST && old->val.kind == v.kind &&
		    old->val.rc == v.rc && old->val.n == v.n && value_known(&v) &&
		    value_known(&old->val)) {
			free(t);
			return;
		}
		s = define(name, S_CONST, 0, loc);
		if (s) {
			s->val = v;
			s->val.sym = NULL;
		}
	} else if ((s = define(name, S_TEXT, 0, loc))) {
		s->text = inner ? inner : xstrdup(t);
		inner = NULL;
	}
	free(inner);
	free(t);
}

static void do_assign(const char *name, const char *text, const struct srcloc *loc)
{
	struct value v;
	struct sym *s;

	if (eval_text(text, &v, loc))
		return;
	s = define(name, S_REDEF, 0, loc);
	if (s) {
		s->val = v;
		s->val.sym = NULL;
	}
}

static void do_textequ(const char *name, const char *text, const struct srcloc *loc)
{
	char *t = trimdup(text);
	char *val = strip_angle(t);
	struct sym *s;

	if (!val && t[0] == '%') {
		struct value v;

		if (eval_text(t + 1, &v, loc) == 0)
			val = xasprintf("%lld", (long long)v.n);
	} else if (!val) {
		int prev;
		struct sym *src = find_sym(t, &prev);

		if (src && !prev && src->kind == S_TEXT)
			val = xstrdup(src->text);
		else
			diag_error(loc, "TEXTEQU needs <text>, %%expression or a text macro");
	}
	if (val && (s = define(name, S_TEXT, 0, loc)))
		s->text = val;
	else
		free(val);
	free(t);
}

static void set_arch(int arch, int corerev, const struct srcloc *loc)
{
	struct sym *s;

	if (as.arch && as.arch != arch) {
		if (corerev)
			diag_error(loc, "core revision %d is arch %d, but the target is arch %d",
				   corerev, arch, as.arch);
		else
			diag_error(loc, "target architecture already set to %d", as.arch);
		return;
	}
	as.arch = arch;
	if (corerev) {
		if (as.corerev && as.corerev != corerev) {
			diag_error(loc, "core revision already set to %d", as.corerev);
			return;
		}
		as.corerev = corerev;
	}
	if (!sym_get(as.global, "@Arch") && (s = define("@Arch", S_CONST, 1, loc)))
		value_num(&s->val, arch);
	if (as.corerev && !sym_get(as.global, "@CoreRev") &&
	    (s = define("@CoreRev", S_CONST, 1, loc)))
		value_num(&s->val, as.corerev);
}

static int corerev_arch(int64_t rev)
{
	if (rev >= 5 && rev < 15)
		return 5;
	if (rev >= 15)
		return 15;
	return 0;
}

static void do_segment(const char *name, const struct toklist *tl, size_t i,
		       const struct srcloc *loc)
{
	int space = -1, has_at = 0;
	int64_t at = 0;
	unsigned int align = 0;
	char *ivname = NULL;
	struct segment *s;
	struct block b;

	if (in_proc()) {
		diag_error(loc, "SEGMENT inside a PROC");
		return;
	}
	while (tl->t.v[i].kind == T_IDENT) {
		const char *w = tl->t.v[i++].s;
		struct value v;
		int sp;

		for (sp = 0; sp < NR_SPACES && !strieq(w, space_names[sp]); sp++)
			;
		if (sp < NR_SPACES) {
			space = sp;
		} else if (strieq(w, "AT")) {
			if (eval_known(tl, &i, &v, "AT", loc))
				return;
			has_at = 1;
			at = v.n;
		} else if (strieq(w, "ALIGN")) {
			if (eval_known(tl, &i, &v, "ALIGN", loc))
				return;
			if (v.n <= 0 || (v.n & (v.n - 1))) {
				diag_error(loc, "ALIGN must be a power of two");
				return;
			}
			align = (unsigned int)v.n;
		} else if (strieq(w, "INIT")) {
			if (tl->t.v[i].kind != T_IDENT) {
				diag_error(loc, "INIT needs an initvals name");
				return;
			}
			ivname = tl->t.v[i++].s;
		} else {
			diag_error(loc, "unknown SEGMENT option '%s'", w);
			return;
		}
	}
	if (!at_end(tl, i, loc))
		return;
	s = seg_open(name, space, has_at, at, align, ivname, loc);
	if (!s)
		return;
	if (ivname && s->space != SP_SHM)
		diag_error(loc, "INIT is only valid for SHM segments");
	memset(&b, 0, sizeof(b));
	b.kind = BK_SEGMENT;
	b.name = xstrdup(name);
	b.loc = loc;
	b.prev_seg = as.seg;
	VEC_PUSH(as.blocks, b);
	as.seg = s;
	as.last_jump = 0;
}

static void struct_open(const char *name, int is_union, const struct srcloc *loc)
{
	struct block b;

	memset(&b, 0, sizeof(b));
	b.kind = BK_STRUCT;
	b.loc = loc;
	b.is_union = is_union;
	if (name) {
		if (in_block(BK_STRUCT)) {
			diag_error(loc, "named STRUC inside a STRUC; declare it outside");
			return;
		}
		b.name = xstrdup(name);
		b.type = type_new(is_union ? TY_UNION : TY_STRUCT, name);
	} else {
		struct block *p;

		if (!in_block(BK_STRUCT)) {
			diag_error(loc, "anonymous STRUC/UNION outside a STRUC");
			return;
		}
		p = &as.blocks.v[as.blocks.n - 1];
		b.type = p->type;
		b.base = b.cur = b.max = p->is_union ? p->base : p->cur;
	}
	VEC_PUSH(as.blocks, b);
}

static int union_alternative(void)
{
	size_t i = as.blocks.n;

	while (i-- && as.blocks.v[i].kind == BK_STRUCT)
		if (as.blocks.v[i].is_union && as.blocks.v[i].member_done)
			return 1;
	return 0;
}

static void member_extent(struct block *b, unsigned int end)
{
	if (b->is_union) {
		if (end > b->max)
			b->max = end;
		b->member_done = 1;
	} else {
		b->cur = end;
		if (b->cur > b->max)
			b->max = b->cur;
	}
}

static void struct_field(const char *name, const struct type *t, const char *items,
			 const struct srcloc *loc)
{
	struct block *b = &as.blocks.v[as.blocks.n - 1];
	struct field f;
	unsigned int count;

	memset(&f, 0, sizeof(f));
	if (!name) {
		diag_error(loc, "STRUC fields need a name");
		return;
	}
	if (type_field(b->type, name)) {
		diag_error(loc, "duplicate field '%s'", name);
		return;
	}
	if (build_items(t, items, &f.def, &count, loc)) {
		free(f.def.v);
		return;
	}
	f.name = xstrdup(name);
	f.offset = b->is_union ? b->base : b->cur;
	f.type = t;
	f.count = count == 1 ? 0 : count;
	f.alt = union_alternative();
	VEC_PUSH(b->type->fields, f);
	member_extent(b, f.offset + (unsigned int)f.def.n);
}

static void struct_close(const char *name, const struct srcloc *loc)
{
	struct block b = as.blocks.v[--as.blocks.n];
	unsigned int end = b.cur > b.max ? b.cur : b.max;
	struct type *t = b.type;
	struct sym *s;
	size_t i, k;

	if (!b.name) {
		if (name)
			diag_error(loc, "anonymous STRUC/UNION closed with a name");
		member_extent(&as.blocks.v[as.blocks.n - 1], end);
		return;
	}
	if (!name || strcmp(name, b.name))
		diag_error(loc, "ENDS does not match STRUC %s", b.name);
	t->size = end;
	image_fill(&t->def, end);
	for (i = 0; i < t->fields.n; i++) {
		const struct field *f = &t->fields.v[i];

		if (f->alt)
			continue;
		for (k = 0; k < f->def.n; k++)
			t->def.v[f->offset + k] = f->def.v[k];
	}
	s = define(b.name, S_TYPE, 1, b.loc);
	if (s)
		s->type = t;
	free(b.name);
}

static void do_ends(const char *name, const struct srcloc *loc)
{
	struct block *b;

	if (!as.blocks.n) {
		diag_error(loc, "ENDS without SEGMENT, STRUC or INITVALS");
		return;
	}
	b = &as.blocks.v[as.blocks.n - 1];
	if (b->kind == BK_STRUCT) {
		struct_close(name, loc);
		return;
	}
	if (!name || strcmp(name, b->name)) {
		diag_error(loc, "ENDS does not match %s", b->name);
		return;
	}
	if (b->kind == BK_SEGMENT)
		as.seg = b->prev_seg;
	free(b->name);
	as.blocks.n--;
}

static void do_record(const char *name, const char *text, const struct srcloc *loc)
{
	struct type *t = type_new(TY_RECORD, name);
	strvec_t items = { 0 };
	unsigned int total = 0, pos;
	uint32_t def = 0;
	struct iword w;
	struct sym *s;
	size_t i;

	split_list(text, &items);
	for (i = 0; i < items.n; i++) {
		const char *it = items.v[i], *colon = strchr(it, ':');
		char *eq, *wtext;
		struct rfield rf;
		struct value v;

		memset(&rf, 0, sizeof(rf));
		if (!colon) {
			diag_error(loc, "record field '%s' needs name:width", it);
			goto out;
		}
		rf.name = trimndup(it, colon - it);
		wtext = xstrdup(colon + 1);
		eq = strchr(wtext, '=');
		if (eq)
			*eq++ = '\0';
		if (eval_text(wtext, &v, loc) || !value_known(&v) || v.n < 1 || v.n > 16) {
			diag_error(loc, "width of %s must be a known number in 1..16", rf.name);
			goto out;
		}
		rf.width = (unsigned int)v.n;
		if (eq) {
			if (eval_text(eq, &v, loc))
				goto out;
			rf.def = (uint16_t)(v.n & ((1u << rf.width) - 1));
		}
		rf.rec = t;
		total += rf.width;
		VEC_PUSH(t->rfields, rf);
		free(wtext);
	}
	if (total > 16) {
		diag_error(loc, "record %s is %u bits wide, more than a word", name, total);
		goto out;
	}
	pos = total;
	for (i = 0; i < t->rfields.n; i++) {
		pos -= t->rfields.v[i].width;
		t->rfields.v[i].shift = pos;
		def |= (uint32_t)t->rfields.v[i].def << pos;
	}
	t->size = 1;
	t->width = total;
	w.v = (uint16_t)def;
	w.set = 1;
	VEC_PUSH(t->def, w);
	s = define(name, S_TYPE, 1, loc);
	if (s)
		s->type = t;
out:
	for (i = 0; i < items.n; i++)
		free(items.v[i]);
	free(items.v);
}

static void do_data(const char *name, const struct type *t, const char *items,
		    const struct srcloc *loc)
{
	image_t img = { 0 };
	unsigned int count;
	struct sym *s;
	size_t i;

	if (in_block(BK_STRUCT)) {
		struct_field(name, t, items, loc);
		return;
	}
	if (in_block(BK_INITVALS) || !as.seg || as.seg->space == SP_CODE) {
		diag_error(loc, "data definition outside a SHM or GPR segment");
		return;
	}
	if (build_items(t, items, &img, &count, loc)) {
		free(img.v);
		return;
	}
	if (name && (s = define(name, S_DATA, 0, loc))) {
		s->seg = as.seg;
		s->off = as.seg->cur;
		s->type = t;
		s->count = count == 1 ? 0 : count;
		track(s);
	}
	for (i = 0; i < img.n; i++) {
		if (img.v[i].set && as.seg->space == SP_GPR) {
			diag_error(loc, "registers cannot be initialized");
			break;
		}
		if (img.v[i].set && !as.seg->ivname) {
			diag_error(loc, "initialized data in %s, which has no INIT", as.seg->name);
			break;
		}
	}
	for (i = 0; i < img.n; i++)
		put_cell(img.v[i].v, img.v[i].set && as.seg->space == SP_SHM, loc);
	free(img.v);
}

static const struct {
	const char *kw;
	enum ivkind kind;
	unsigned int nargs;
} ivkw[] = {
	{ "mmio16", IV_MMIO16, 2 }, { "mmio32", IV_MMIO32, 2 },
	{ "phy", IV_PHY, 2 }, { "radio", IV_RADIO, 2 },
	{ "shm16", IV_SHM16, 3 }, { "shm32", IV_SHM32, 3 },
	{ "tram", IV_TRAM, 2 },
};

static void do_ival(size_t k, const struct toklist *tl, size_t i, const struct srcloc *loc)
{
	struct ivop op;
	int64_t a[3];
	unsigned int n;

	for (n = 0; n < ivkw[k].nargs; n++) {
		struct value v;

		if (n) {
			if (!tok_is_punct(&tl->t.v[i], P_COMMA))
				goto bad;
			i++;
		}
		if (eval_known(tl, &i, &v, ivkw[k].kw, loc))
			return;
		a[n] = v.n;
	}
	if (!at_end(tl, i, loc))
		return;
	memset(&op, 0, sizeof(op));
	op.kind = ivkw[k].kind;
	op.value = (uint32_t)a[0];
	op.offset = (uint32_t)a[n - 1];
	if (n == 3)
		op.routing = (uint32_t)a[1];
	if (a[0] < 0 || a[0] > ((op.kind == IV_MMIO32 || op.kind == IV_SHM32 ||
				 op.kind == IV_TRAM) ? 0xFFFFFFFFLL : 0xFFFF)) {
		diag_error(loc, "%s value 0x%llX out of range", ivkw[k].kw, (long long)a[0]);
		return;
	}
	if (a[n - 1] < 0 || a[n - 1] > (op.kind == IV_TRAM ? 0xFFFFFFFFLL : 0xFFFF) ||
	    (n == 3 && (a[1] < 0 || a[1] > 4))) {
		diag_error(loc, "%s offset or routing out of range", ivkw[k].kw);
		return;
	}
	VEC_PUSH(as.blocks.v[as.blocks.n - 1].iv->ops, op);
	return;
bad:
	diag_error(loc, "%s takes %u comma-separated values", ivkw[k].kw, ivkw[k].nargs);
}

/* Procedures */

static struct proc *proc_get(const char *name)
{
	struct proc *p;
	size_t i;

	for (i = 0; i < as.procs.n; i++)
		if (strcmp(as.procs.v[i]->name, name) == 0)
			return as.procs.v[i];
	p = xcalloc(1, sizeof(*p));
	p->name = xstrdup(name);
	VEC_PUSH(as.procs, p);
	return p;
}

static void define_regs(const paramvec_t *pv, const struct srcloc *loc)
{
	size_t i;

	for (i = 0; i < pv->n; i++) {
		struct sym *s = define(pv->v[i].name, S_CONST, 0, loc);

		if (s)
			s->val = pv->v[i].reg;
	}
}

static void do_proc(const char *name, const char *header, const struct srcloc *loc)
{
	struct proc *p;
	struct sym *s;

	if (in_proc()) {
		diag_error(loc, "PROC %s inside PROC %s", name, as.proc->name);
		return;
	}
	if (!as.seg || as.seg->space != SP_CODE || (as.blocks.n && !in_block(BK_SEGMENT))) {
		diag_error(loc, "PROC outside a CODE segment");
		return;
	}
	p = proc_get(name);
	if (p->seen_pass == as.pass) {
		diag_error(loc, "PROC %s defined twice", name);
		return;
	}
	p->seen_pass = as.pass;
	p->loc = loc;
	free(p->header);
	p->header = xstrdup(header);
	p->header_loc = loc;
	p->locals.n = 0;
	if (as.pass == 1) {
		contract_free(&p->c);
		contract_eval(p, &p->c, 0);
	}
	s = define(name, S_PROC, 1, loc);
	if (s) {
		s->proc = p;
		s->seg = as.seg;
		s->off = as.seg->cur;
		track(s);
	}
	p->scope = symtab_new();
	as.proc = p;
	as.last_jump = 0;
	define_regs(&p->c.params, loc);
}

static void do_endp(const char *name, const struct srcloc *loc)
{
	if (!in_proc() || strcmp(name, as.proc->name)) {
		diag_error(loc, "ENDP %s does not match an open PROC", name);
		return;
	}
	as.proc = as.top;
}

static void do_local(const char *text, const struct srcloc *loc)
{
	struct bodyline bl;
	paramvec_t pv = { 0 };
	size_t i;

	if (!in_proc()) {
		diag_error(loc, "LOCAL outside a PROC or a macro");
		return;
	}
	bl.text = xstrdup(text);
	bl.loc = loc;
	VEC_PUSH(as.proc->locals, bl);
	if (contract_locals_line(&bl, &pv, 0) == 0)
		define_regs(&pv, loc);
	for (i = 0; i < pv.n; i++)
		free(pv.v[i].name);
	free(pv.v);
}

static void add_callsite(const char *target, int tail)
{
	struct callsite cs;

	cs.target = xstrdup(target);
	cs.tail = tail;
	cs.callee = NULL;
	VEC_PUSH(as.proc->calls, cs);
}

/* Instructions */

struct opnd {
	struct operand op;
	struct value v;
	const char *ident;	/* the operand is just this identifier */
	int empty;
};

static int parse_operand(struct exprctx *x, struct opnd *o)
{
	const struct token *t = ex_peek(x);
	const struct token *nt = &x->tl->t.v[x->i + 1];

	memset(o, 0, sizeof(*o));
	o->op.base = -1;
	if (t->kind == T_IDENT && (nt->kind == T_EOL || tok_is_punct(nt, P_COMMA)))
		o->ident = t->s;
	if (ex_accept(x, P_LBRACK)) {
		int r;

		x->addr_mode = 1;
		r = expr_parse(x, &o->v);
		if (r == 0 && ex_accept(x, P_COMMA)) {
			struct value b;

			r = expr_parse(x, &b);
			if (r == 0 && (b.kind != V_REG || b.rc != RC_OFF || o->v.base >= 0)) {
				diag_error(x->loc, "expected [address, offN]");
				r = -1;
			}
			o->v.base = (int)b.n;
		}
		x->addr_mode = 0;
		if (r)
			return -1;
		if (!ex_accept(x, P_RBRACK)) {
			diag_error(x->loc, "expected ']'");
			return -1;
		}
		if (o->v.kind != V_NUM) {
			diag_error(x->loc, "memory operand must be an address, optionally plus offN");
			return -1;
		}
		o->op.kind = OK_MEM;
		o->op.n = o->v.n;
		o->op.base = o->v.base;
		return 0;
	}
	if (tok_is_kw(t, "RAW") && tok_is_punct(nt, P_LPAREN)) {
		x->i++;
		if (expr_parse(x, &o->v))
			return -1;
		if (o->v.kind != V_NUM) {
			diag_error(x->loc, "RAW() needs a number");
			return -1;
		}
		o->op.kind = OK_RAW;
		o->op.n = o->v.n;
		return 0;
	}
	if (expr_parse(x, &o->v))
		return -1;
	if (o->v.kind == V_REG) {
		o->op.kind = OK_REG;
		o->op.rc = o->v.rc;
		o->op.n = o->v.n;
		if (o->v.rc == RC_GPR && (o->v.n < 0 || o->v.n >= NR_GPR) && value_known(&o->v)) {
			diag_error(x->loc, "register r%lld out of range", (long long)o->v.n);
			return -1;
		}
	} else if (o->v.base >= 0) {
		diag_error(x->loc, "offset register outside [ ]");
		return -1;
	} else {
		o->op.kind = (o->v.flags & VF_MEM) ? OK_MEM : OK_IMM;
		o->op.n = o->v.n;
	}
	return 0;
}

static int parse_operands(const struct toklist *tl, size_t i, struct opnd *o,
			  size_t *n, int allow_empty, const struct srcloc *loc)
{
	struct exprctx x = { tl, i, loc, 0 };

	*n = 0;
	if (ex_at_end(&x))
		return 0;
	for (;;) {
		if (*n == MAX_OPERANDS) {
			diag_error(loc, "too many operands");
			return -1;
		}
		if (allow_empty && (ex_at_end(&x) || tok_is_punct(ex_peek(&x), P_COMMA))) {
			memset(&o[*n], 0, sizeof(o[*n]));
			o[*n].empty = 1;
		} else if (parse_operand(&x, &o[*n])) {
			return -1;
		}
		(*n)++;
		if (ex_accept(&x, P_COMMA))
			continue;
		if (!ex_at_end(&x)) {
			diag_error(loc, "unexpected token after operand %zu", *n);
			return -1;
		}
		return 0;
	}
}

static int arch_check(const struct isa_op *op, const struct srcloc *loc)
{
	if (!as.arch) {
		diag_error(loc, "no target architecture (.arch, .corerev or -a)");
		return -1;
	}
	if ((op->flags & IF_ARCH5) && as.arch != 5) {
		diag_error(loc, "'%s' exists only on arch 5; CALL and RET are portable", op->mn);
		return -1;
	}
	if ((op->flags & IF_ARCH15) && as.arch != 15) {
		diag_error(loc, "'%s' exists only on arch 15; CALL and RET are portable", op->mn);
		return -1;
	}
	if ((op->flags & IF_REV11) && as.corerev && as.corerev < 11) {
		diag_error(loc, "'%s' needs core revision 11 or later", op->mn);
		return -1;
	}
	return 0;
}

static int code_segment(const struct srcloc *loc)
{
	if (in_block(BK_STRUCT) || in_block(BK_INITVALS) || !as.seg ||
	    as.seg->space != SP_CODE) {
		diag_error(loc, "instruction outside a CODE segment");
		return 0;
	}
	return 1;
}

/* Emits one instruction slot; pass 1 only reserves it. */
static void emit(const struct isa_op *op, struct opnd *o, int record,
		 const struct srcloc *loc)
{
	int ti = isa_target_index(op->fmt), di = isa_dest_index(op->fmt);
	struct operand ops[5];
	uint64_t w;
	size_t k, n = isa_nr_operands(op->fmt);
	int ok = 1;

	if (record && ti >= 0 && o[ti].ident &&
	    (isa_is_jump(op->fmt) || op->fmt == F_CALL || op->fmt == F_CALLS))
		add_callsite(o[ti].ident, isa_is_jump(op->fmt));
	if (as.pass < 2 || arch_check(op, loc)) {
		put_cell(0, 0, loc);
		as.last_jump = isa_is_jump(op->fmt);
		return;
	}
	for (k = 0; k < n; k++) {
		if (!value_known(&o[k].v)) {
			diag_error(loc, "operand %zu of %s is not resolved", k + 1, op->mn);
			ok = 0;
		}
		ops[k] = o[k].op;
	}
	if (ok && di >= 0 && o[di].v.kind == V_REG)
		proc_check_write(as.proc, &o[di].v, loc);
	if (ok && ti >= 0 && o[ti].v.sym && o[ti].v.sym->kind == S_PROC)
		proc_check_call(as.proc, o[ti].v.sym->proc, isa_is_jump(op->fmt),
				op->fmt == F_CALL && record ? (int)o[0].v.n : -1,
				as.arch, loc);
	if ((op->fmt == F_RET || op->fmt == F_RETS) && as.last_jump)
		diag_warn(loc, "return right after a jump; the hardware does not handle this");
	if (ok && isa_encode(as.arch, op, ops, &w, loc) == 0)
		put_cell(w, 1, loc);
	else
		put_cell(0, 0, loc);
	as.last_jump = isa_is_jump(op->fmt);
}

static void lr_opnd(struct opnd *o, int lr)
{
	memset(o, 0, sizeof(*o));
	value_reg(&o->v, RC_LR, lr);
	o->op.kind = OK_REG;
	o->op.rc = RC_LR;
	o->op.n = lr;
	o->op.base = -1;
}

static struct proc *opnd_proc(const struct opnd *o)
{
	return o->v.sym && o->v.sym->kind == S_PROC ? o->v.sym->proc : NULL;
}

static void emit_call(const struct opnd *target, const struct srcloc *loc)
{
	struct proc *q = opnd_proc(target);
	struct opnd o[2];

	if (as.pass >= 2 && !q)
		diag_error(loc, "CALL target is not a PROC");
	if (as.arch == 15) {
		o[0] = *target;
		emit(isa_find("calls"), o, 0, loc);
	} else {
		lr_opnd(&o[0], q ? q->c.link : 0);
		o[1] = *target;
		emit(isa_find("call"), o, 0, loc);
	}
}

static void do_ret(const struct srcloc *loc)
{
	struct opnd o[2];

	if (!in_proc()) {
		diag_error(loc, "RET outside a PROC");
		put_cell(0, 0, loc);
		return;
	}
	if (as.arch == 15) {
		emit(isa_find("rets"), o, 0, loc);
		return;
	}
	lr_opnd(&o[0], as.proc->c.link);
	lr_opnd(&o[1], as.proc->c.link);
	emit(isa_find("ret"), o, 0, loc);
}

static void reg_opnd(struct opnd *o, const struct value *reg)
{
	memset(o, 0, sizeof(*o));
	o->v = *reg;
	o->v.sym = NULL;
	o->op.kind = OK_REG;
	o->op.rc = reg->rc;
	o->op.n = reg->n;
	o->op.base = -1;
}

static void emit_mov(const struct opnd *src, const struct opnd *dst,
		     const struct srcloc *loc)
{
	struct opnd o[2];

	o[0] = *src;
	o[1] = *dst;
	o[0].ident = o[1].ident = NULL;
	emit(isa_find("mov"), o, 0, loc);
}

static int same_reg(const struct value *a, const struct value *b)
{
	return a->kind == V_REG && b->kind == V_REG && value_known(a) &&
	       value_known(b) && a->rc == b->rc && a->n == b->n;
}

static int reads_any(const struct opnd *o, const struct regset *rs)
{
	struct value base;

	if (o->v.kind == V_REG && o->v.rc != RC_SPR && regset_has(rs, &o->v))
		return 1;
	if (o->op.kind == OK_MEM && o->op.base >= 0) {
		value_reg(&base, RC_OFF, o->op.base);
		return regset_has(rs, &base);
	}
	return 0;
}

static void regset_put(struct regset *rs, const struct value *reg)
{
	if (reg->kind != V_REG)
		return;
	if (reg->rc == RC_GPR)
		rs->gpr |= (uint64_t)1 << reg->n;
	else if (reg->rc == RC_OFF)
		rs->off |= 1u << reg->n;
}

static void do_invoke(const struct toklist *tl, size_t i, const struct srcloc *loc)
{
	struct opnd o[MAX_OPERANDS], preg;
	struct regset written = { 0, 0 };
	const paramvec_t *pv;
	struct proc *q;
	size_t n, k, nargs;

	if (parse_operands(tl, i, o, &n, 1, loc) || n == 0 || o[0].empty) {
		if (n == 0)
			diag_error(loc, "INVOKE needs a PROC");
		put_cell(0, 0, loc);
		return;
	}
	if (o[0].ident)
		add_callsite(o[0].ident, 0);
	q = opnd_proc(&o[0]);
	nargs = n - 1;
	if (!q) {
		if (as.pass >= 2)
			diag_error(loc, "INVOKE target is not a PROC");
		for (k = 0; k < n; k++)
			put_cell(0, 0, loc);
		return;
	}
	pv = &q->c.params;
	if (nargs > pv->n) {
		diag_error(loc, "INVOKE %s: %zu arguments for %zu parameters", q->name, nargs, pv->n);
		nargs = pv->n;
	}
	for (k = 0; k < pv->n; k++) {
		const struct param *p = &pv->v[k];
		struct opnd *a = &o[1 + k];

		if (p->dir == PD_OUT)
			continue;
		if (k >= nargs || a->empty) {
			diag_error(loc, "INVOKE %s: missing argument for %s", q->name, p->name);
			continue;
		}
		if (same_reg(&a->v, &p->reg))
			continue;
		if (as.pass >= 2 && reads_any(a, &written))
			diag_error(loc, "INVOKE %s: argument %zu reads a register already loaded with an earlier argument",
				   q->name, k + 1);
		reg_opnd(&preg, &p->reg);
		emit_mov(a, &preg, loc);
		regset_put(&written, &p->reg);
	}
	emit_call(&o[0], loc);
	memset(&written, 0, sizeof(written));
	for (k = 0; k < nargs; k++) {
		const struct param *p = &pv->v[k];
		struct opnd *a = &o[1 + k];

		if (p->dir == PD_IN || a->empty || same_reg(&a->v, &p->reg))
			continue;
		if (a->op.kind != OK_REG && a->op.kind != OK_MEM) {
			diag_error(loc, "INVOKE %s: OUT argument for %s must be a register or memory",
				   q->name, p->name);
			continue;
		}
		if (as.pass >= 2 && regset_has(&written, &p->reg))
			diag_error(loc, "INVOKE %s: result %s is overwritten by an earlier result",
				   q->name, p->name);
		reg_opnd(&preg, &p->reg);
		emit_mov(&preg, a, loc);
		regset_put(&written, &a->v);
	}
}

static void do_instruction(const struct toklist *tl, size_t i, const struct srcloc *loc)
{
	const struct token *t = &tl->t.v[i];
	struct opnd o[MAX_OPERANDS];
	const struct isa_op *op;
	char mn[16];
	size_t n, j = i + 1;

	snprintf(mn, sizeof(mn), "%s", t->s);
	if (tok_is_punct(&tl->t.v[j], P_DOT) && tl->t.v[j].pos == t->end) {
		strncat(mn, ".", sizeof(mn) - strlen(mn) - 1);
		j++;
	}
	op = isa_find(mn);
	if (!code_segment(loc))
		return;
	if (!op || parse_operands(tl, j, o, &n, 0, loc)) {
		if (!op)
			diag_error(loc, "unknown instruction '%s'", mn);
		put_cell(0, 0, loc);
		return;
	}
	if (strieq(mn, "call") && n == 1) {
		if (o[0].ident)
			add_callsite(o[0].ident, 0);
		emit_call(&o[0], loc);
		return;
	}
	if (strieq(mn, "ret") && n == 0) {
		do_ret(loc);
		return;
	}
	if (n != isa_nr_operands(op->fmt)) {
		diag_error(loc, "%s takes %u operands", op->mn, isa_nr_operands(op->fmt));
		put_cell(0, 0, loc);
		return;
	}
	emit(op, o, 1, loc);
}

/* Statements */

static const struct srcloc cmdline_loc = { "<command line>", 0, NULL, NULL };

static int data_keyword(const char *w, const struct type **t)
{
	if (strieq(w, "DW") || strieq(w, "WORD")) {
		*t = &type_word;
		return 1;
	}
	if (strieq(w, "DD") || strieq(w, "DWORD")) {
		*t = &type_dword;
		return 1;
	}
	return 0;
}

static void define_label(const char *name, int global, const struct srcloc *loc)
{
	struct sym *s;

	if (in_block(BK_STRUCT) || in_block(BK_INITVALS) || !as.seg) {
		diag_error(loc, "label outside a segment");
		return;
	}
	if (strcmp(name, "@@") == 0) {
		struct anon a = { as.seg, as.seg->cur };

		VEC_PUSH(as.anon, a);
		return;
	}
	s = define(name, S_LABEL, global, loc);
	if (s) {
		s->seg = as.seg;
		s->off = as.seg->cur;
		track(s);
	}
}

static void do_initvals(const char *name, const struct srcloc *loc)
{
	struct block b;

	if (as.blocks.n && !in_block(BK_SEGMENT)) {
		diag_error(loc, "INITVALS inside a STRUC or INITVALS");
		return;
	}
	memset(&b, 0, sizeof(b));
	b.kind = BK_INITVALS;
	b.name = xstrdup(name);
	b.loc = loc;
	b.iv = ivsect_get(name);
	VEC_PUSH(as.blocks, b);
}

static int named_directive(const struct toklist *tl, size_t i, const struct srcloc *loc)
{
	const char *name = tl->t.v[i].s;
	const struct token *k = &tl->t.v[i + 1];
	const char *r = rest(tl, i + 2);
	const struct type *t;

	if (tok_is_punct(k, P_ASSIGN)) {
		do_assign(name, r, loc);
		return 1;
	}
	if (k->kind != T_IDENT)
		return 0;
	if (strieq(k->s, "EQU"))
		do_equ(name, r, loc);
	else if (strieq(k->s, "TEXTEQU"))
		do_textequ(name, r, loc);
	else if (strieq(k->s, "MACRO"))
		capture_start(CAP_MACRO, name, r, loc);
	else if (strieq(k->s, "STRUC") || strieq(k->s, "STRUCT"))
		struct_open(name, 0, loc);
	else if (strieq(k->s, "UNION"))
		struct_open(name, 1, loc);
	else if (strieq(k->s, "RECORD"))
		do_record(name, r, loc);
	else if (strieq(k->s, "PROC"))
		do_proc(name, r, loc);
	else if (strieq(k->s, "ENDP"))
		do_endp(name, loc);
	else if (strieq(k->s, "SEGMENT"))
		do_segment(name, tl, i + 2, loc);
	else if (strieq(k->s, "ENDS"))
		do_ends(name, loc);
	else if (strieq(k->s, "INITVALS"))
		do_initvals(name, loc);
	else if (data_keyword(k->s, &t) || (t = find_type(k->s)))
		do_data(name, t, rest(tl, i + 2), loc);
	else
		return 0;
	return 1;
}

static void do_include(const char *text, const struct srcloc *loc)
{
	char *name = trimdup(text);
	size_t n = strlen(name);
	char *in = strip_angle(name);

	if (in) {
		free(name);
		name = in;
	} else if (n >= 2 && (name[0] == '"' || name[0] == '\'') && name[n - 1] == name[0]) {
		memmove(name, name + 1, n - 2);
		name[n - 2] = '\0';
	}
	src_push_file(name, loc, as.cond.n);
	free(name);
}

static void do_exitm(const char *text, const struct srcloc *loc)
{
	char *t = trimdup(text);
	struct strbuf *res;
	size_t base;

	if (!src_in_expansion()) {
		diag_error(loc, "EXITM outside a macro or repeat block");
		free(t);
		return;
	}
	res = src_exit_expansion(&base);
	if (as.cond.n > base)
		as.cond.n = base;
	if (res && t[0] == '%') {
		struct value v;

		if (eval_text(t + 1, &v, loc) == 0)
			sb_printf(res, "%lld", (long long)v.n);
	} else if (res) {
		char *in = strip_angle(t);

		sb_puts(res, in ? in : t);
		free(in);
	}
	free(t);
}

static void do_align(unsigned int a, const struct srcloc *loc)
{
	if (!as.seg || in_block(BK_STRUCT) || in_block(BK_INITVALS)) {
		diag_error(loc, "ALIGN outside a segment");
		return;
	}
	while (as.seg->cur % a)
		put_cell(0, 0, loc);
	if (a > as.seg->align)
		as.seg->align = a;
}

static void do_err(const char *kw, const struct toklist *tl, size_t i,
		   const struct srcloc *loc)
{
	int fail = 1;
	size_t j = i;

	if (strieq(kw, ".ERRE") || strieq(kw, ".ERRNZ")) {
		struct value v;

		if (eval_known(tl, &j, &v, kw, loc))
			return;
		fail = strieq(kw, ".ERRE") ? v.n == 0 : v.n != 0;
	} else if (strieq(kw, ".ERRDEF") || strieq(kw, ".ERRNDEF")) {
		if (tl->t.v[j].kind != T_IDENT) {
			diag_error(loc, "%s needs a symbol name", kw);
			return;
		}
		fail = strieq(kw, ".ERRDEF") ? asm_defined(tl->t.v[j].s) : !asm_defined(tl->t.v[j].s);
		j++;
	}
	if (!fail)
		return;
	if (tok_is_punct(&tl->t.v[j], P_COMMA))
		j++;
	if (tl->t.v[j].kind != T_EOL) {
		char *msg = text_arg(rest(tl, j));

		diag_error(loc, "%s", msg);
		free(msg);
	} else {
		diag_error(loc, "%s condition met", kw);
	}
}

static void do_memory(const struct toklist *tl, size_t i, const struct srcloc *loc)
{
	struct value v;
	int sp;

	if (tl->t.v[i].kind != T_IDENT || !tok_is_punct(&tl->t.v[i + 1], P_COMMA))
		goto bad;
	for (sp = 0; sp < NR_SPACES && !strieq(tl->t.v[i].s, space_names[sp]); sp++)
		;
	if (sp == NR_SPACES)
		goto bad;
	i += 2;
	if (eval_known(tl, &i, &v, ".memory size", loc) || !at_end(tl, i, loc))
		return;
	if (v.n <= 0 || v.n > 0x10000 || (sp == SP_CODE && v.n > isa_code_limit(as.arch ? as.arch : 15)) ||
	    (sp == SP_GPR && v.n > NR_GPR)) {
		diag_error(loc, ".memory %s size out of range", space_names[sp]);
		return;
	}
	as.memlimit[sp] = (unsigned int)v.n;
	return;
bad:
	diag_error(loc, "expected .memory CODE|SHM|GPR, size");
}

static int directive(const struct toklist *tl, size_t i, const struct srcloc *loc)
{
	const char *w = tl->t.v[i].s;
	const char *r = rest(tl, i + 1);
	size_t j = i + 1;
	const struct type *t;
	struct value v;
	size_t k;

	if (strieq(w, ".arch")) {
		if (eval_known(tl, &j, &v, ".arch", loc) == 0 && at_end(tl, j, loc)) {
			if (v.n != 5 && v.n != 15)
				diag_error(loc, "architecture must be 5 or 15");
			else
				set_arch((int)v.n, 0, loc);
		}
	} else if (strieq(w, ".corerev")) {
		if (eval_known(tl, &j, &v, ".corerev", loc) == 0 && at_end(tl, j, loc)) {
			if (!corerev_arch(v.n))
				diag_error(loc, "core revision %lld has no PSM instruction set", (long long)v.n);
			else
				set_arch(corerev_arch(v.n), (int)v.n, loc);
		}
	} else if (strieq(w, ".memory")) {
		do_memory(tl, j, loc);
	} else if (strieq(w, ".stackdepth")) {
		if (eval_known(tl, &j, &v, ".stackdepth", loc) == 0 && at_end(tl, j, loc)) {
			as.stackdepth = (int)v.n;
			as.stackdepth_loc = loc;
		}
	} else if (strieq(w, ".code")) {
		const char *name = tl->t.v[j].kind == T_IDENT ? tl->t.v[j].s : "_TEXT";
		struct segment *s;

		if (as.blocks.n || in_proc()) {
			diag_error(loc, ".code inside a block or PROC");
		} else if ((s = seg_open(name, SP_CODE, 0, 0, 0, NULL, loc))) {
			as.seg = s;
			as.last_jump = 0;
		}
	} else if (strieq(w, "INCLUDE")) {
		do_include(r, loc);
	} else if (strieq(w, "ORG")) {
		if (eval_known(tl, &j, &v, "ORG", loc) == 0 && at_end(tl, j, loc)) {
			if (!as.seg || in_block(BK_STRUCT) || in_block(BK_INITVALS) || v.n < 0)
				diag_error(loc, "ORG needs a non-negative offset inside a segment");
			else
				as.seg->cur = (unsigned int)v.n;
		}
	} else if (strieq(w, "ALIGN") || strieq(w, "EVEN")) {
		if (strieq(w, "EVEN"))
			do_align(2, loc);
		else if (eval_known(tl, &j, &v, "ALIGN", loc) == 0 && at_end(tl, j, loc)) {
			if (v.n <= 0 || v.n > 0x10000 || (v.n & (v.n - 1)))
				diag_error(loc, "ALIGN must be a power of two");
			else
				do_align((unsigned int)v.n, loc);
		}
	} else if (strieq(w, "ECHO")) {
		char *m = trimdup(r);

		diag_echo(m);
		free(m);
	} else if (strieq(w, ".ERR") || strieq(w, ".ERRE") || strieq(w, ".ERRNZ") ||
		   strieq(w, ".ERRDEF") || strieq(w, ".ERRNDEF")) {
		do_err(w, tl, j, loc);
	} else if (strieq(w, "END")) {
		as.ended = 1;
		if (tl->t.v[j].kind == T_IDENT) {
			as.entry = xstrdup(tl->t.v[j].s);
			as.entry_loc = loc;
		}
	} else if (strieq(w, "EXITM")) {
		do_exitm(r, loc);
	} else if (strieq(w, "PURGE")) {
		strvec_t names = { 0 };

		split_list(r, &names);
		for (k = 0; k < names.n; k++) {
			struct sym *s = sym_get(as.global, names.v[k]);

			if (s && s->kind == S_MACRO)
				sym_remove(as.global, names.v[k]);
			else
				diag_error(loc, "PURGE: %s is not a macro", names.v[k]);
			free(names.v[k]);
		}
		free(names.v);
	} else if (strieq(w, "LOCAL")) {
		do_local(r, loc);
	} else if (strieq(w, "INVOKE")) {
		if (code_segment(loc))
			do_invoke(tl, j, loc);
	} else if (strieq(w, "REPT") || strieq(w, "REPEAT")) {
		capture_start(CAP_REPT, NULL, r, loc);
	} else if (strieq(w, "FOR") || strieq(w, "IRP")) {
		capture_start(CAP_FOR, NULL, r, loc);
	} else if (strieq(w, "FORC") || strieq(w, "IRPC")) {
		capture_start(CAP_FORC, NULL, r, loc);
	} else if (strieq(w, "WHILE")) {
		capture_start(CAP_WHILE, NULL, r, loc);
	} else if (strieq(w, "STRUC") || strieq(w, "STRUCT") || strieq(w, "UNION")) {
		struct_open(NULL, strieq(w, "UNION"), loc);
	} else if (strieq(w, "ENDS")) {
		do_ends(NULL, loc);
	} else if (strieq(w, "ENDM")) {
		diag_error(loc, "ENDM without MACRO or repeat block");
	} else if (data_keyword(w, &t)) {
		do_data(NULL, t, r, loc);
	} else {
		for (k = 0; k < ARRAY_SIZE(ivkw); k++) {
			if (strieq(w, ivkw[k].kw) && in_block(BK_INITVALS)) {
				do_ival(k, tl, j, loc);
				return 1;
			}
		}
		return 0;
	}
	return 1;
}

static void statement(const struct toklist *tl, size_t i, const struct srcloc *loc)
{
	const struct token *t = &tl->t.v[i];
	struct macro *m;
	const struct type *ty;

	if (t->kind != T_IDENT) {
		diag_error(loc, "statement expected");
		return;
	}
	if (named_directive(tl, i, loc) || directive(tl, i, loc))
		return;
	if ((m = find_macro(t->s))) {
		body_t out = { 0 };

		if (macro_expand(m, rest(tl, i + 1), loc, &as.uniq, &out) == 0)
			src_push_lines(PK_MACRO, &out, as.cond.n, NULL);
		return;
	}
	if ((ty = find_type(t->s))) {
		do_data(NULL, ty, rest(tl, i + 1), loc);
		return;
	}
	if (isa_find(t->s)) {
		do_instruction(tl, i, loc);
		return;
	}
	diag_error(loc, "unknown instruction or directive '%s'", t->s);
}

static void dispatch(const struct toklist *tl, const struct srcloc *loc)
{
	const struct token *t = tl->t.v;
	const char *test = NULL;
	enum ckw ck;
	size_t i = 0;

	if (t[0].kind == T_EOL)
		return;
	if (t[0].kind == T_IDENT && (ck = cond_kw(t[0].s, &test)) != C_NONE) {
		do_cond(ck, test, tl, 1, loc);
		return;
	}
	if (t[0].kind == T_IDENT && (tok_is_punct(&t[1], P_COLON) || tok_is_punct(&t[1], P_DCOLON))) {
		define_label(t[0].s, t[1].p == P_DCOLON, loc);
		i = 2;
		if (t[2].kind == T_EOL)
			return;
	}
	statement(tl, i, loc);
}

static void skip_line(const struct line *ln)
{
	struct toklist tl = { 0 };
	const char *test = NULL;
	char w[64], *text;
	enum ckw k;

	word(ln->text, w, sizeof(w));
	k = cond_kw(w, &test);
	if (k == C_NONE)
		return;
	if (k != C_ELSEIF) {
		do_cond(k, test, NULL, 0, ln->loc);
		return;
	}
	text = expand_line(ln->text, ln->loc);
	if (text && lex_line(text, &tl, ln->loc) == 0)
		do_cond(k, test, &tl, 1, ln->loc);
	toklist_free(&tl);
	free(text);
}

static void process_line(const struct line *ln)
{
	struct toklist tl = { 0 };
	struct segment *seg0;
	unsigned int cur0;
	char *text;

	if (as.cap.kind != CAP_NONE) {
		capture_line(ln);
		return;
	}
	if (!cond_active()) {
		skip_line(ln);
		return;
	}
	text = expand_line(ln->text, ln->loc);
	if (!text)
		return;
	seg0 = as.seg;
	cur0 = seg0 ? seg0->cur : 0;
	if (lex_line(text, &tl, ln->loc) == 0)
		dispatch(&tl, ln->loc);
	if (as.opts->listing && as.pass >= 2) {
		struct listent le;

		le.loc = ln->loc;
		le.text = xstrdup(ln->text);
		le.seg = seg0;
		le.from = cur0;
		le.to = seg0 && as.seg == seg0 && seg0->cur > cur0 ? seg0->cur : cur0;
		VEC_PUSH(as.listing, le);
	}
	toklist_free(&tl);
	free(text);
}

/* Passes */

static int last_arch;

static void all_procs(void (*fn)(struct proc *))
{
	size_t i;

	fn(as.top);
	for (i = 0; i < as.procs.n; i++)
		fn(as.procs.v[i]);
}

static void proc_new_pass(struct proc *p)
{
	size_t i;

	p->scope_prev = p->scope;
	p->scope = NULL;
	if (as.pass >= 2 && !p->is_top) {
		contract_free(&p->c);
		p->c = p->next;
		memset(&p->next, 0, sizeof(p->next));
	}
	for (i = 0; i < p->calls.n; i++)
		free(p->calls.v[i].target);
	p->calls.n = 0;
}

static void proc_resolve_calls(struct proc *p)
{
	size_t i;

	for (i = 0; i < p->calls.n; i++) {
		struct callsite *cs = &p->calls.v[i];
		struct sym *s = p->is_top ? NULL : sym_get(p->scope, cs->target);

		if (!s)
			s = sym_get(as.global, cs->target);
		cs->callee = s && s->kind == S_PROC ? s->proc : NULL;
	}
}

static void proc_resolve_contract(struct proc *p)
{
	if (p->is_top || p->seen_pass != as.pass)
		return;
	contract_free(&p->next);
	contract_eval(p, &p->next, as.pass >= 2);
	if (as.pass >= 2 && !contract_equal(&p->next, &p->c))
		as.changed = 1;
}

static void analyze(void)
{
	struct proc **all = xmalloc((as.procs.n + 1) * sizeof(*all));

	all[0] = as.top;
	memcpy(all + 1, as.procs.v, as.procs.n * sizeof(*all));
	proc_analyze(all, as.procs.n + 1, last_arch);
	free(all);
}

static void begin_pass(int k)
{
	size_t i;

	diag_buffer(1);
	as.pass = k;
	as.arch = as.opts->arch;
	as.corerev = as.opts->corerev;
	as.global_prev = as.global;
	as.global = symtab_new();
	as.cond.n = 0;
	as.blocks.n = 0;
	memset(&as.cap, 0, sizeof(as.cap));
	free(as.anon_prev.v);
	as.anon_prev = as.anon;
	memset(&as.anon, 0, sizeof(as.anon));
	as.uniq = 0;
	as.ended = as.changed = as.last_jump = as.expand_depth = 0;
	free(as.entry);
	as.entry = NULL;
	memset(as.memlimit, 0, sizeof(as.memlimit));
	as.stackdepth = 0;
	as.listing.n = 0;
	for (i = 0; i < as.segs.n; i++) {
		as.segs.v[i]->cur = as.segs.v[i]->size = 0;
		as.segs.v[i]->cells.n = 0;
	}
	for (i = 0; i < as.ivs.n; i++)
		as.ivs.v[i]->ops.n = 0;
	if (k >= 2 && last_arch)
		analyze();
	all_procs(proc_new_pass);
	as.proc = as.top;
	if (as.arch)
		set_arch(as.arch, as.corerev, &cmdline_loc);
	for (i = 0; i < as.opts->defines.n; i++) {
		const char *d = as.opts->defines.v[i];
		const char *eq = strchr(d, '=');
		char *name = eq ? xstrndup(d, eq - d) : xstrdup(d);

		if (eq) {
			do_equ(name, eq + 1, &cmdline_loc);
		} else {
			struct sym *s = define(name, S_CONST, 1, &cmdline_loc);

			if (s)
				value_num(&s->val, 1);
		}
		free(name);
	}
	as.text = seg_open("_TEXT", SP_CODE, 0, 0, 0, NULL, &cmdline_loc);
	as.seg = as.text;
}

static void end_pass(void)
{
	int max_depth = 0;
	size_t i;

	if (as.cap.kind != CAP_NONE)
		diag_error(as.cap.loc, "MACRO or repeat block without ENDM");
	if (as.cond.n)
		diag_error(as.cond.v[0].loc, "IF without ENDIF");
	for (i = 0; i < as.blocks.n; i++)
		diag_error(as.blocks.v[i].loc, "%s without ENDS",
			   as.blocks.v[i].name ? as.blocks.v[i].name : "STRUC");
	if (in_proc())
		diag_error(as.proc->loc, "PROC %s without ENDP", as.proc->name);
	as.proc = as.top;
	if (!as.arch) {
		diag_error(NULL, "no target architecture: use .arch, .corerev or -a");
		return;
	}
	last_arch = as.arch;
	all_procs(proc_resolve_calls);
	layout();
	all_procs(proc_resolve_contract);
	if (as.entry) {
		int prev;
		struct sym *s = find_sym(as.entry, &prev);
		struct value v;

		if (!s || (s->kind != S_LABEL && s->kind != S_PROC) || s->seg->space != SP_CODE)
			diag_error(as.entry_loc, "END: %s is not a code label", as.entry);
		else if (sym_value(s, prev, &v, as.entry_loc) == 0 && v.n != 0)
			diag_error(as.entry_loc, "entry point %s is at 0x%llX, but the PSM starts at 0",
				   as.entry, (long long)v.n);
	}
	if (as.arch == 15 && as.stackdepth) {
		for (i = 0; i < as.procs.n; i++)
			if (as.procs.v[i]->depth > max_depth)
				max_depth = as.procs.v[i]->depth;
		if (as.top->depth > max_depth)
			max_depth = as.top->depth;
		if (max_depth > as.stackdepth)
			diag_error(as.stackdepth_loc, "call chains need %d return stack entries, more than %d",
				   max_depth, as.stackdepth);
	}
}

int asm_run(const char *file, const struct asm_opts *opts, struct asm_result *res)
{
	static const struct src_hooks hooks = { on_popped, on_eval };
	unsigned int errors;
	struct line ln;
	int k;

	memset(&as, 0, sizeof(as));
	last_arch = 0;
	as.opts = opts;
	src_init(&hooks, opts->incdirs.v, opts->incdirs.n);
	as.top = xcalloc(1, sizeof(*as.top));
	as.top->name = xstrdup("(top level)");
	as.top->is_top = 1;
	as.top->c.uses_all = 1;
	for (k = 1;; k++) {
		begin_pass(k);
		if (src_push_file(file, NULL, 0) == 0)
			while (!as.ended && src_next(&ln, 0))
				process_line(&ln);
		src_reset();
		end_pass();
		if (k >= 2 && !as.changed)
			break;
		if (k == MAX_PASSES) {
			diag_error(NULL, "layout did not settle after %d passes", k);
			break;
		}
	}
	errors = diag_errors();
	diag_flush();
	diag_buffer(0);
	memset(res, 0, sizeof(*res));
	res->arch = as.arch;
	res->corerev = as.corerev;
	res->passes = (unsigned int)k;
	res->globals = as.global;
	for (k = 0; k < (int)as.segs.n; k++)
		VEC_PUSH(res->segs, as.segs.v[k]);
	for (k = 0; k < (int)as.ivs.n; k++)
		VEC_PUSH(res->ivs, as.ivs.v[k]);
	VEC_PUSH(res->procs, as.top);
	for (k = 0; k < (int)as.procs.n; k++)
		VEC_PUSH(res->procs, as.procs.v[k]);
	res->listing.v = as.listing.v;
	res->listing.n = as.listing.n;
	return errors ? -1 : 0;
}
