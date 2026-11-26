#include "macro.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/types.h>

static void free_strvec(strvec_t *v)
{
	size_t i;

	for (i = 0; i < v->n; i++)
		free(v->v[i]);
	free(v->v);
	memset(v, 0, sizeof(*v));
}

static int is_word(const char *s, const char *kw, const char **rest)
{
	size_t n = strlen(kw);

	while (isspace((unsigned char)*s))
		s++;
	if (strncasecmp(s, kw, n) || is_ident_char((unsigned char)s[n]))
		return 0;
	*rest = s + n;
	return 1;
}

struct macro *macro_define(const char *name, const char *params, body_t *body,
			   const struct srcloc *loc)
{
	struct macro *m = xcalloc(1, sizeof(*m));
	strvec_t items = { 0 };
	size_t i;

	m->name = xstrdup(name);
	m->loc = loc;
	m->via = xasprintf("expansion of macro %s", name);
	m->body = *body;
	memset(body, 0, sizeof(*body));
	if (split_list(params, &items)) {
		diag_error(loc, "unterminated string in macro parameters");
		goto bad;
	}
	for (i = 0; i < items.n; i++) {
		struct mparam p;
		const char *s = items.v[i], *colon = strchr(s, ':');
		size_t nl = colon ? (size_t)(colon - s) : strlen(s);

		memset(&p, 0, sizeof(p));
		while (nl && isspace((unsigned char)s[nl - 1]))
			nl--;
		p.name = xstrndup(s, nl);
		if (!nl || !is_ident_start((unsigned char)p.name[0])) {
			diag_error(loc, "bad macro parameter '%s'", s);
			goto bad;
		}
		if (colon) {
			const char *a = colon + 1, *rest;

			if (*a == '=') {
				char *d;

				for (a++; isspace((unsigned char)*a); a++)
					;
				d = strip_angle(a);
				p.def = d ? d : xstrdup(a);
			} else if (is_word(a, "REQ", &rest)) {
				p.req = 1;
			} else if (is_word(a, "VARARG", &rest)) {
				if (i + 1 != items.n) {
					diag_error(loc, "VARARG must be the last parameter");
					goto bad;
				}
				p.vararg = 1;
			} else {
				diag_error(loc, "bad macro parameter qualifier in '%s'", s);
				goto bad;
			}
		}
		VEC_PUSH(m->params, p);
	}
	free_strvec(&items);
	return m;
bad:
	free_strvec(&items);
	return NULL;
}

static char *one_arg(const char *raw, const struct srcloc *loc)
{
	char *inner = strip_angle(raw);
	struct value v;

	if (inner)
		return inner;
	if (raw[0] == '%') {
		if (eval_text(raw + 1, &v, loc))
			return NULL;
		if (v.kind != V_NUM) {
			diag_error(loc, "%%expression must be a number");
			return NULL;
		}
		return xasprintf("%lld", (long long)v.n);
	}
	return xstrdup(raw);
}

int macro_args(const char *text, strvec_t *out, const struct srcloc *loc)
{
	strvec_t raw = { 0 };
	size_t i;

	if (split_list(text, &raw)) {
		diag_error(loc, "unterminated string in macro arguments");
		return -1;
	}
	for (i = 0; i < raw.n; i++) {
		char *a = one_arg(raw.v[i], loc);

		if (!a) {
			free_strvec(&raw);
			return -1;
		}
		VEC_PUSH(*out, a);
	}
	free_strvec(&raw);
	return 0;
}

static ssize_t find_name(char *const *names, size_t n, const char *s, size_t len)
{
	size_t i;

	for (i = 0; i < n; i++)
		if (strlen(names[i]) == len && strncmp(names[i], s, len) == 0)
			return (ssize_t)i;
	return -1;
}

static char *subst(const char *s, char *const *names, char *const *vals, size_t n)
{
	struct strbuf sb = { 0 };
	size_t i = 0;
	char quote = 0;

	while (s[i]) {
		char c = s[i];

		if (quote) {
			if (c == quote) {
				quote = 0;
			} else if (c == '&' && is_ident_start((unsigned char)s[i + 1])) {
				size_t j = i + 1;
				ssize_t k;

				while (is_ident_char((unsigned char)s[j]))
					j++;
				k = find_name(names, n, s + i + 1, j - i - 1);
				if (k >= 0) {
					sb_puts(&sb, vals[k]);
					i = s[j] == '&' ? j + 1 : j;
					continue;
				}
			}
			sb_putc(&sb, c);
			i++;
			continue;
		}
		if (c == '"' || c == '\'') {
			quote = c;
		} else if (is_ident_start((unsigned char)c) &&
			   (i == 0 || !is_ident_char((unsigned char)s[i - 1]))) {
			size_t j = i;
			ssize_t k;

			while (is_ident_char((unsigned char)s[j]))
				j++;
			k = find_name(names, n, s + i, j - i);
			if (k >= 0) {
				if (i > 0 && s[i - 1] == '&' && sb.len && sb.s[sb.len - 1] == '&')
					sb.len--;
				sb_puts(&sb, vals[k]);
				i = s[j] == '&' ? j + 1 : j;
			} else {
				sb_putn(&sb, s + i, j - i);
				i = j;
			}
			continue;
		} else if (isdigit((unsigned char)c)) {
			size_t j = i;

			while (is_ident_char((unsigned char)s[j]))
				j++;
			sb_putn(&sb, s + i, j - i);
			i = j;
			continue;
		}
		sb_putc(&sb, c);
		i++;
	}
	return sb.s ? sb_steal(&sb) : xstrdup("");
}

static void emit(body_t *out, const struct bodyline *bl, char *text,
		 const struct srcloc *loc, const char *via)
{
	struct bodyline nl;

	nl.text = text;
	nl.loc = srcloc_new(bl->loc->file, bl->loc->line, loc, via);
	VEC_PUSH(*out, nl);
}

int macro_expand(const struct macro *m, const char *args, const struct srcloc *loc,
		 unsigned int *uniq, body_t *out)
{
	strvec_t raw = { 0 }, names = { 0 }, vals = { 0 };
	size_t i, first = 0;
	int r = -1;

	if (split_list(args, &raw)) {
		diag_error(loc, "unterminated string in macro arguments");
		goto out;
	}
	if (raw.n > m->params.n && !(m->params.n && m->params.v[m->params.n - 1].vararg)) {
		diag_error(loc, "too many arguments for macro %s", m->name);
		goto out;
	}
	for (i = 0; i < m->params.n; i++) {
		const struct mparam *p = &m->params.v[i];
		char *val = NULL;

		if (p->vararg) {
			struct strbuf sb = { 0 };
			size_t k;

			for (k = i; k < raw.n; k++) {
				if (k > i)
					sb_putc(&sb, ',');
				sb_puts(&sb, raw.v[k]);
			}
			val = sb.s ? sb_steal(&sb) : xstrdup("");
		} else if (i < raw.n && raw.v[i][0]) {
			val = one_arg(raw.v[i], loc);
			if (!val)
				goto out;
		} else if (p->req) {
			diag_error(loc, "macro %s: missing required argument %s", m->name, p->name);
			goto out;
		} else {
			val = xstrdup(p->def ? p->def : "");
		}
		VEC_PUSH(names, xstrdup(p->name));
		VEC_PUSH(vals, val);
	}
	for (first = 0; first < m->body.n; first++) {
		const char *rest;
		strvec_t locs = { 0 };
		size_t k;

		if (!is_word(m->body.v[first].text, "LOCAL", &rest))
			break;
		if (split_list(rest, &locs) || strchr(rest, ':')) {
			free_strvec(&locs);
			break;
		}
		for (k = 0; k < locs.n; k++) {
			VEC_PUSH(names, xstrdup(locs.v[k]));
			VEC_PUSH(vals, xasprintf("??%04X", (*uniq)++));
		}
		free_strvec(&locs);
	}
	for (i = first; i < m->body.n; i++)
		emit(out, &m->body.v[i], subst(m->body.v[i].text, names.v, vals.v, names.n),
		     loc, m->via);
	r = 0;
out:
	free_strvec(&raw);
	free_strvec(&names);
	free_strvec(&vals);
	return r;
}

int repeat_body(const body_t *body, const char *param, const strvec_t *values,
		size_t times, const struct srcloc *loc, const char *via, body_t *out)
{
	size_t it, i, n = param ? values->n : times;

	for (it = 0; it < n; it++) {
		for (i = 0; i < body->n; i++) {
			char *text;

			if (param) {
				char *nm = (char *)param;

				text = subst(body->v[i].text, &nm, &values->v[it], 1);
			} else {
				text = xstrdup(body->v[i].text);
			}
			emit(out, &body->v[i], text, loc, via);
		}
	}
	return 0;
}
