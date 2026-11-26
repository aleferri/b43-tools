#include "masm.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

const struct type type_word = { .kind = TY_WORD, .name = "WORD", .size = 1 };
const struct type type_dword = { .kind = TY_DWORD, .name = "DWORD", .size = 2 };

struct type *type_new(enum type_kind kind, const char *name)
{
	struct type *t = xcalloc(1, sizeof(*t));

	t->kind = kind;
	t->name = xstrdup(name ? name : "");
	return t;
}

const struct field *type_field(const struct type *t, const char *name)
{
	size_t i;

	for (i = 0; i < t->fields.n; i++)
		if (strcmp(t->fields.v[i].name, name) == 0)
			return &t->fields.v[i];
	return NULL;
}

const struct rfield *type_rfield(const struct type *t, const char *name)
{
	size_t i;

	for (i = 0; i < t->rfields.n; i++)
		if (strcmp(t->rfields.v[i].name, name) == 0)
			return &t->rfields.v[i];
	return NULL;
}

void image_append(image_t *dst, const image_t *src)
{
	size_t i;

	for (i = 0; i < src->n; i++)
		VEC_PUSH(*dst, src->v[i]);
}

void image_fill(image_t *img, unsigned int n)
{
	struct iword w = { 0, 0 };

	while (n--)
		VEC_PUSH(*img, w);
}

static void image_put(image_t *img, size_t at, uint16_t v)
{
	while (img->n <= at)
		image_fill(img, 1);
	img->v[at].v = v;
	img->v[at].set = 1;
}

int eval_text(const char *text, struct value *v, const struct srcloc *loc)
{
	struct toklist tl = { 0 };
	struct exprctx x = { &tl, 0, loc, 0 };
	int r = -1;

	if (lex_line(text, &tl, loc) == 0 && expr_parse(&x, v) == 0) {
		if (ex_at_end(&x))
			r = 0;
		else
			diag_error(loc, "junk after expression '%s'", text);
	}
	toklist_free(&tl);
	return r;
}

/* Index of the '>' closing the '<' at s[0], or 0 if there is none. */
static size_t angle_end(const char *s)
{
	size_t i;
	int depth = 0;
	char quote = 0;

	for (i = 0; s[i]; i++) {
		if (quote) {
			if (s[i] == quote)
				quote = 0;
		} else if (s[i] == '!' && s[i + 1]) {
			i++;
		} else if (s[i] == '"' || s[i] == '\'') {
			quote = s[i];
		} else if (s[i] == '<') {
			depth++;
		} else if (s[i] == '>') {
			if (--depth == 0)
				return i;
		}
	}
	return 0;
}

static char *trim(const char *s, size_t n)
{
	while (n && isspace((unsigned char)*s)) {
		s++;
		n--;
	}
	while (n && isspace((unsigned char)s[n - 1]))
		n--;
	return xstrndup(s, n);
}

int split_list(const char *s, strvec_t *out)
{
	size_t i, start = 0;
	int paren = 0;
	char quote = 0;
	const char *p;

	for (p = s; *p && isspace((unsigned char)*p); p++)
		;
	if (!*p)
		return 0;
	for (i = 0;; i++) {
		char c = s[i];

		if (quote) {
			if (c == '\0')
				return -1;
			if (c == quote)
				quote = 0;
			continue;
		}
		if (c == '"' || c == '\'') {
			quote = c;
		} else if (c == '<') {
			size_t e = angle_end(s + i);

			if (e)
				i += e;
		} else if (c == '(' || c == '[') {
			paren++;
		} else if (c == ')' || c == ']') {
			paren--;
		} else if ((c == ',' && paren == 0) || c == '\0') {
			VEC_PUSH(*out, trim(s + start, i - start));
			start = i + 1;
			if (c == '\0')
				break;
		}
	}
	return 0;
}

char *strip_angle(const char *s)
{
	size_t n = strlen(s), e, i;
	struct strbuf sb = { 0 };

	if (n < 2 || s[0] != '<')
		return NULL;
	e = angle_end(s);
	if (e != n - 1)
		return NULL;
	for (i = 1; i < e; i++) {
		if (s[i] == '!' && i + 1 < e)
			i++;
		sb_putc(&sb, s[i]);
	}
	return sb.s ? sb_steal(&sb) : xstrdup("");
}

/* Finds a top-level "DUP" keyword in an item. */
static const char *find_dup(const char *s)
{
	size_t i;
	char quote = 0;
	int depth = 0;

	for (i = 0; s[i]; i++) {
		if (quote) {
			if (s[i] == quote)
				quote = 0;
			continue;
		}
		if (s[i] == '"' || s[i] == '\'')
			quote = s[i];
		else if (s[i] == '(' || s[i] == '<')
			depth++;
		else if (s[i] == ')' || s[i] == '>')
			depth--;
		else if (depth == 0 && strncasecmp(s + i, "DUP", 3) == 0 &&
			 (i == 0 || !is_ident_char((unsigned char)s[i - 1])) &&
			 !is_ident_char((unsigned char)s[i + 3]))
			return s + i;
	}
	return NULL;
}

static int scalar_item(const struct type *t, const char *text, image_t *img,
		       const struct srcloc *loc)
{
	struct value v;
	int64_t lo, hi;

	if (t->kind == TY_STRUCT || t->kind == TY_UNION) {
		diag_error(loc, "%s needs an initializer in <...>", t->name);
		return -1;
	}
	if (eval_text(text, &v, loc))
		return -1;
	if (v.kind != V_NUM || v.base >= 0) {
		diag_error(loc, "data initializer must be a number");
		return -1;
	}
	if (t->kind == TY_DWORD) {
		lo = -((int64_t)1 << 31);
		hi = 0xFFFFFFFFLL;
	} else {
		lo = -32768;
		hi = 0xFFFF;
	}
	if (value_known(&v) && (v.n < lo || v.n > hi)) {
		diag_error(loc, "initializer 0x%llX does not fit a %s",
			   (unsigned long long)v.n, t->name);
		return -1;
	}
	image_put(img, img->n, (uint16_t)v.n);
	if (t->kind == TY_DWORD)
		image_put(img, img->n, (uint16_t)(v.n >> 16));
	return 0;
}

int build_items(const struct type *t, const char *text, image_t *img,
		unsigned int *count, const struct srcloc *loc)
{
	strvec_t items = { 0 };
	size_t i;
	int r = 0;

	*count = 0;
	if (split_list(text, &items)) {
		diag_error(loc, "unterminated string in initializer");
		return -1;
	}
	for (i = 0; i < items.n && r == 0; i++) {
		const char *it = items.v[i];
		const char *dup = find_dup(it);
		char *inner;

		if (it[0] == '\0') {
			diag_error(loc, "empty initializer");
			r = -1;
		} else if (strcmp(it, "?") == 0) {
			image_fill(img, t->size);
			(*count)++;
		} else if (dup) {
			char *ntext = xstrndup(it, dup - it);
			const char *rest = dup + 3;
			struct value n;
			image_t sub = { 0 };
			unsigned int subcount, k;
			size_t rl;

			while (isspace((unsigned char)*rest))
				rest++;
			rl = strlen(rest);
			if (eval_text(ntext, &n, loc)) {
				r = -1;
			} else if (!value_known(&n) || n.kind != V_NUM || n.n < 0 || n.n > 0x10000) {
				diag_error(loc, "DUP count must be a known number in 0..65536");
				r = -1;
			} else if (rl < 2 || rest[0] != '(' || rest[rl - 1] != ')') {
				diag_error(loc, "DUP needs a parenthesized initializer");
				r = -1;
			} else {
				char *body = xstrndup(rest + 1, rl - 2);

				r = build_items(t, body, &sub, &subcount, loc);
				for (k = 0; r == 0 && k < n.n; k++)
					image_append(img, &sub);
				*count += subcount * (unsigned int)n.n;
				free(body);
			}
			free(sub.v);
			free(ntext);
		} else if ((inner = strip_angle(it))) {
			image_t one = { 0 };

			r = build_aggregate(t, inner, &one, loc);
			image_append(img, &one);
			(*count)++;
			free(one.v);
			free(inner);
		} else {
			r = scalar_item(t, it, img, loc);
			(*count)++;
		}
	}
	for (i = 0; i < items.n; i++)
		free(items.v[i]);
	free(items.v);
	return r;
}

static int build_record(const struct type *t, const strvec_t *pieces, image_t *img,
			const struct srcloc *loc)
{
	uint32_t w = 0;
	size_t i;

	if (pieces->n > t->rfields.n) {
		diag_error(loc, "too many initializers for record %s", t->name);
		return -1;
	}
	for (i = 0; i < t->rfields.n; i++) {
		const struct rfield *rf = &t->rfields.v[i];
		uint32_t m = (1u << rf->width) - 1;
		int64_t v = rf->def;

		if (i < pieces->n && pieces->v[i][0]) {
			struct value val;

			if (eval_text(pieces->v[i], &val, loc))
				return -1;
			if (val.kind != V_NUM) {
				diag_error(loc, "record field %s needs a number", rf->name);
				return -1;
			}
			v = val.n;
			if (value_known(&val) &&
			    (v > (int64_t)m || v < -((int64_t)1 << (rf->width - 1)))) {
				diag_error(loc, "%lld does not fit the %u-bit field %s",
					   (long long)v, rf->width, rf->name);
				return -1;
			}
		}
		w |= ((uint32_t)v & m) << rf->shift;
	}
	image_put(img, img->n, (uint16_t)w);
	return 0;
}

static int build_field(const struct field *f, const char *piece, image_t *img,
		       const struct srcloc *loc)
{
	image_t sub = { 0 };
	unsigned int count = 0, total = f->count ? f->count : 1;
	char *inner = strip_angle(piece);
	size_t i;
	int r;

	if (strcmp(piece, "?") == 0) {
		image_fill(&sub, total * f->type->size);
		r = 0;
	} else if (f->count) {
		if (!inner) {
			diag_error(loc, "array field %s needs <...>", f->name);
			free(inner);
			return -1;
		}
		r = build_items(f->type, inner, &sub, &count, loc);
		if (r == 0 && count > f->count) {
			diag_error(loc, "too many initializers for %s", f->name);
			r = -1;
		}
	} else if (inner && f->type->kind != TY_WORD && f->type->kind != TY_DWORD) {
		r = build_aggregate(f->type, inner, &sub, loc);
	} else {
		r = build_items(f->type, piece, &sub, &count, loc);
		if (r == 0 && count != 1) {
			diag_error(loc, "field %s takes one initializer", f->name);
			r = -1;
		}
	}
	for (i = 0; r == 0 && i < sub.n; i++) {
		while (img->n <= f->offset + i)
			image_fill(img, 1);
		img->v[f->offset + i] = sub.v[i];
	}
	free(sub.v);
	free(inner);
	return r;
}

int build_aggregate(const struct type *t, const char *text, image_t *img,
		    const struct srcloc *loc)
{
	strvec_t pieces = { 0 };
	image_t tmp = { 0 };
	size_t i, k = 0;
	int r = 0;

	if (t->kind == TY_WORD || t->kind == TY_DWORD) {
		diag_error(loc, "<...> initializer used for a %s", t->name);
		return -1;
	}
	if (split_list(text, &pieces)) {
		diag_error(loc, "unterminated string in initializer");
		return -1;
	}
	if (t->kind == TY_RECORD) {
		r = build_record(t, &pieces, img, loc);
		goto out;
	}
	image_append(&tmp, &t->def);
	image_fill(&tmp, t->size - (unsigned int)tmp.n);
	for (i = 0; r == 0 && i < t->fields.n && k < pieces.n; i++) {
		if (t->fields.v[i].alt)
			continue;
		if (pieces.v[k][0])
			r = build_field(&t->fields.v[i], pieces.v[k], &tmp, loc);
		k++;
	}
	if (r == 0 && k < pieces.n) {
		diag_error(loc, "too many initializers for %s", t->name);
		r = -1;
	}
	image_append(img, &tmp);
out:
	for (i = 0; i < pieces.n; i++)
		free(pieces.v[i]);
	free(pieces.v);
	free(tmp.v);
	return r;
}
