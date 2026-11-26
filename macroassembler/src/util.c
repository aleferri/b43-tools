#include "util.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ERRORS	100

static unsigned int nr_errors;
static int werror;
static int buffering;
static int quiet;
static struct strbuf diagbuf;

static void oom(void)
{
	fprintf(stderr, "b43-masm: out of memory\n");
	exit(1);
}

void *xmalloc(size_t size)
{
	void *p = malloc(size ? size : 1);

	if (!p)
		oom();
	return p;
}

void *xcalloc(size_t n, size_t size)
{
	void *p = calloc(n ? n : 1, size ? size : 1);

	if (!p)
		oom();
	return p;
}

void *xrealloc(void *p, size_t size)
{
	p = realloc(p, size ? size : 1);
	if (!p)
		oom();
	return p;
}

char *xstrdup(const char *s)
{
	return xstrndup(s, strlen(s));
}

char *xstrndup(const char *s, size_t n)
{
	char *d = xmalloc(n + 1);

	memcpy(d, s, n);
	d[n] = '\0';
	return d;
}

char *xasprintf(const char *fmt, ...)
{
	va_list ap;
	char *s;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(NULL, 0, fmt, ap);
	va_end(ap);
	s = xmalloc(n + 1);
	va_start(ap, fmt);
	vsnprintf(s, n + 1, fmt, ap);
	va_end(ap);
	return s;
}

int strieq(const char *a, const char *b)
{
	while (*a && *b) {
		if (tolower((unsigned char)*a) != tolower((unsigned char)*b))
			return 0;
		a++;
		b++;
	}
	return *a == *b;
}

static void sb_grow(struct strbuf *sb, size_t extra)
{
	if (sb->len + extra + 1 <= sb->cap)
		return;
	sb->cap = (sb->len + extra + 1) * 2;
	sb->s = xrealloc(sb->s, sb->cap);
}

void sb_putc(struct strbuf *sb, char c)
{
	sb_grow(sb, 1);
	sb->s[sb->len++] = c;
	sb->s[sb->len] = '\0';
}

void sb_putn(struct strbuf *sb, const char *s, size_t n)
{
	sb_grow(sb, n);
	memcpy(sb->s + sb->len, s, n);
	sb->len += n;
	sb->s[sb->len] = '\0';
}

void sb_puts(struct strbuf *sb, const char *s)
{
	sb_putn(sb, s, strlen(s));
}

void sb_printf(struct strbuf *sb, const char *fmt, ...)
{
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(NULL, 0, fmt, ap);
	va_end(ap);
	sb_grow(sb, n);
	va_start(ap, fmt);
	vsnprintf(sb->s + sb->len, n + 1, fmt, ap);
	va_end(ap);
	sb->len += n;
}

const char *sb_cstr(struct strbuf *sb)
{
	sb_grow(sb, 0);
	sb->s[sb->len] = '\0';
	return sb->s;
}

char *sb_steal(struct strbuf *sb)
{
	char *s;

	sb_cstr(sb);
	s = sb->s;
	memset(sb, 0, sizeof(*sb));
	return s;
}

void sb_reset(struct strbuf *sb)
{
	sb->len = 0;
	if (sb->s)
		sb->s[0] = '\0';
}

struct srcloc *srcloc_new(const char *file, int line,
			  const struct srcloc *parent, const char *via)
{
	struct srcloc *l = xmalloc(sizeof(*l));

	l->file = file;
	l->line = line;
	l->parent = parent;
	l->via = via;
	return l;
}

static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void out(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (buffering) {
		char *s;
		int n = vsnprintf(NULL, 0, fmt, ap);

		va_end(ap);
		s = xmalloc(n + 1);
		va_start(ap, fmt);
		vsnprintf(s, n + 1, fmt, ap);
		sb_puts(&diagbuf, s);
		free(s);
	} else {
		vfprintf(stderr, fmt, ap);
	}
	va_end(ap);
}

static void vdiag(const char *kind, const struct srcloc *loc,
		  const char *fmt, va_list ap)
{
	const struct srcloc *l;
	char *msg;
	int n;
	va_list cp;

	va_copy(cp, ap);
	n = vsnprintf(NULL, 0, fmt, cp);
	va_end(cp);
	msg = xmalloc(n + 1);
	vsnprintf(msg, n + 1, fmt, ap);
	if (loc)
		out("%s:%d: ", loc->file, loc->line);
	out("%s: %s\n", kind, msg);
	free(msg);
	for (l = loc; l && l->parent; l = l->parent)
		out("%s:%d: note: in %s\n", l->parent->file, l->parent->line, l->via);
}

void diag_error(const struct srcloc *loc, const char *fmt, ...)
{
	va_list ap;

	if (quiet)
		return;
	if (nr_errors >= MAX_ERRORS) {
		if (nr_errors++ == MAX_ERRORS)
			out("b43-masm: too many errors, the rest are not shown\n");
		return;
	}
	va_start(ap, fmt);
	vdiag("error", loc, fmt, ap);
	va_end(ap);
	nr_errors++;
}

void diag_warn(const struct srcloc *loc, const char *fmt, ...)
{
	va_list ap;

	if (quiet)
		return;
	va_start(ap, fmt);
	vdiag(werror ? "error" : "warning", loc, fmt, ap);
	va_end(ap);
	if (werror)
		nr_errors++;
}

void diag_note(const struct srcloc *loc, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vdiag("note", loc, fmt, ap);
	va_end(ap);
}

void diag_fatal(const struct srcloc *loc, const char *fmt, ...)
{
	va_list ap;

	diag_flush();
	buffering = 0;
	va_start(ap, fmt);
	vdiag("fatal", loc, fmt, ap);
	va_end(ap);
	exit(1);
}

void diag_echo(const char *text)
{
	out("%s\n", text);
}

void diag_set_werror(int on)
{
	werror = on;
}

void diag_buffer(int on)
{
	buffering = on;
	sb_reset(&diagbuf);
	nr_errors = 0;
}

void diag_quiet(int on)
{
	quiet += on ? 1 : -1;
}

void diag_flush(void)
{
	if (diagbuf.len)
		fputs(sb_cstr(&diagbuf), stderr);
	sb_reset(&diagbuf);
}

unsigned int diag_errors(void)
{
	return nr_errors;
}
