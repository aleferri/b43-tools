#include "source.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_DEPTH	256
#define MAX_WHILE_ITER	65536

struct provider {
	enum prov_kind kind;
	struct provider *up;
	size_t cond_base;

	/* PK_FILE */
	char *buf;
	size_t pos, len;
	const char *path;
	int lineno;
	int in_comment;

	/* PK_MACRO, PK_REPEAT, PK_WHILE */
	body_t body;
	size_t idx;
	struct strbuf *result;

	/* PK_WHILE */
	char *cond;
	const struct srcloc *cond_loc;
	unsigned int iter;
};

static struct provider *top;
static size_t depth;
static struct src_hooks hooks;
static char **incdirs;
static size_t nr_incdirs;
static struct strbuf linebuf;

void src_init(const struct src_hooks *h, char **dirs, size_t n)
{
	hooks = *h;
	incdirs = dirs;
	nr_incdirs = n;
}

static void push(struct provider *p)
{
	if (depth >= MAX_DEPTH)
		diag_fatal(NULL, "include/expansion nesting deeper than %d", MAX_DEPTH);
	p->up = top;
	top = p;
	depth++;
}

static struct provider *pop(void)
{
	struct provider *p = top;

	top = p->up;
	depth--;
	return p;
}

static void free_provider(struct provider *p)
{
	if (p->kind == PK_MACRO || p->kind == PK_REPEAT)
		free(p->body.v);
	if (p->kind == PK_WHILE) {
		free(p->body.v);
		free(p->cond);
	}
	free(p->buf);
	free(p);
}

static char *read_file(const char *path, size_t *len)
{
	FILE *f = fopen(path, "rb");
	char *buf;
	long n;

	if (!f)
		return NULL;
	if (fseek(f, 0, SEEK_END) || (n = ftell(f)) < 0 || fseek(f, 0, SEEK_SET)) {
		fclose(f);
		return NULL;
	}
	buf = xmalloc(n + 1);
	if (fread(buf, 1, n, f) != (size_t)n) {
		fclose(f);
		free(buf);
		return NULL;
	}
	fclose(f);
	buf[n] = '\0';
	*len = n;
	return buf;
}

static const char *current_file(void)
{
	struct provider *p;

	for (p = top; p; p = p->up)
		if (p->kind == PK_FILE)
			return p->path;
	return NULL;
}

int src_push_file(const char *name, const struct srcloc *from, size_t cond_base)
{
	struct provider *p;
	const char *cur = current_file();
	char *path = NULL;
	size_t len, i;
	char *buf = NULL;

	if (name[0] == '/' || !cur) {
		path = xstrdup(name);
		buf = read_file(path, &len);
	} else {
		const char *slash = strrchr(cur, '/');

		path = slash ? xasprintf("%.*s/%s", (int)(slash - cur), cur, name)
			     : xstrdup(name);
		buf = read_file(path, &len);
		for (i = 0; !buf && i < nr_incdirs; i++) {
			free(path);
			path = xasprintf("%s/%s", incdirs[i], name);
			buf = read_file(path, &len);
		}
	}
	if (!buf) {
		diag_error(from, "cannot open '%s'", name);
		free(path);
		return -1;
	}
	p = xcalloc(1, sizeof(*p));
	p->kind = PK_FILE;
	p->cond_base = cond_base;
	p->buf = buf;
	p->len = len;
	p->path = path;
	push(p);
	return 0;
}

void src_push_lines(enum prov_kind kind, body_t *lines, size_t cond_base,
		    struct strbuf *result)
{
	struct provider *p = xcalloc(1, sizeof(*p));

	p->kind = kind;
	p->body = *lines;
	memset(lines, 0, sizeof(*lines));
	p->cond_base = cond_base;
	p->result = result;
	push(p);
}

void src_push_while(const char *cond, const struct srcloc *cond_loc,
		    const body_t *body, size_t cond_base)
{
	struct provider *p = xcalloc(1, sizeof(*p));
	size_t i;

	p->kind = PK_WHILE;
	for (i = 0; i < body->n; i++)
		VEC_PUSH(p->body, body->v[i]);
	p->idx = p->body.n;
	p->cond = xstrdup(cond);
	p->cond_loc = cond_loc;
	p->cond_base = cond_base;
	push(p);
}

/* Appends one physical line to linebuf with comments removed. */
static void strip_physical(struct provider *p, const char *s, size_t n)
{
	size_t i = 0;
	char quote = 0;

	while (i < n) {
		char c = s[i];

		if (p->in_comment) {
			if (c == '*' && i + 1 < n && s[i + 1] == '/') {
				p->in_comment = 0;
				i += 2;
				sb_putc(&linebuf, ' ');
			} else {
				i++;
			}
			continue;
		}
		if (quote) {
			sb_putc(&linebuf, c);
			if (c == quote)
				quote = 0;
			i++;
			continue;
		}
		if (c == '"' || c == '\'') {
			quote = c;
		} else if (c == ';' || (c == '/' && i + 1 < n && s[i + 1] == '/')) {
			break;
		} else if (c == '/' && i + 1 < n && s[i + 1] == '*') {
			p->in_comment = 1;
			i += 2;
			continue;
		}
		sb_putc(&linebuf, c);
		i++;
	}
}

static int next_file_line(struct provider *p, struct line *out)
{
	int first_line = 0;

	sb_reset(&linebuf);
	while (p->pos < p->len) {
		const char *s = p->buf + p->pos;
		const char *nl = memchr(s, '\n', p->len - p->pos);
		size_t n = nl ? (size_t)(nl - s) : p->len - p->pos;
		size_t end;

		p->pos += n + (nl ? 1 : 0);
		p->lineno++;
		if (!first_line)
			first_line = p->lineno;
		if (n && s[n - 1] == '\r')
			n--;
		strip_physical(p, s, n);
		end = linebuf.len;
		while (end && isspace((unsigned char)linebuf.s[end - 1]))
			end--;
		if (end && linebuf.s[end - 1] == '\\') {
			linebuf.len = end - 1;
			sb_putc(&linebuf, ' ');
			continue;
		}
		linebuf.len = end;
		out->text = sb_cstr(&linebuf);
		out->loc = srcloc_new(p->path, first_line, NULL, NULL);
		return 1;
	}
	if (linebuf.len) {
		out->text = sb_cstr(&linebuf);
		out->loc = srcloc_new(p->path, first_line, NULL, NULL);
		return 1;
	}
	return 0;
}

static int next_body_line(struct provider *p, struct line *out)
{
	if (p->kind == PK_WHILE && p->idx == p->body.n) {
		if (++p->iter > MAX_WHILE_ITER) {
			diag_error(p->cond_loc, "WHILE exceeded %d iterations", MAX_WHILE_ITER);
			return 0;
		}
		if (!hooks.eval(p->cond, p->cond_loc))
			return 0;
		p->idx = 0;
	}
	if (p->idx == p->body.n)
		return 0;
	out->text = p->body.v[p->idx].text;
	out->loc = p->body.v[p->idx].loc;
	if (p->kind == PK_WHILE)
		out->loc = srcloc_new(out->loc->file, out->loc->line, p->cond_loc, "WHILE");
	p->idx++;
	return 1;
}

int src_next(struct line *out, size_t min_depth)
{
	while (top && depth >= min_depth) {
		struct provider *p = top;
		int got = p->kind == PK_FILE ? next_file_line(p, out)
					     : next_body_line(p, out);
		struct srcloc *end;

		if (got)
			return 1;
		end = p->kind == PK_FILE ? srcloc_new(p->path, p->lineno, NULL, NULL) : NULL;
		pop();
		hooks.popped(p->kind, p->cond_base, end);
		free_provider(p);
	}
	return 0;
}

size_t src_depth(void)
{
	return depth;
}

int src_in_expansion(void)
{
	struct provider *p;

	for (p = top; p; p = p->up)
		if (p->kind != PK_FILE)
			return 1;
	return 0;
}

struct strbuf *src_exit_expansion(size_t *cond_base)
{
	while (top) {
		struct provider *p = pop();
		struct strbuf *res = p->result;
		enum prov_kind kind = p->kind;

		*cond_base = p->cond_base;
		free_provider(p);
		if (kind != PK_FILE)
			return res;
	}
	return NULL;
}

void src_reset(void)
{
	while (top)
		free_provider(pop());
}
