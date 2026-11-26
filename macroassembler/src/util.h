#ifndef B43_MASM_UTIL_H_
#define B43_MASM_UTIL_H_

#include <stddef.h>
#include <stdint.h>

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))

#define VEC(T)	struct { T *v; size_t n, cap; }

#define VEC_PUSH(vec, x) do {						\
	if ((vec).n == (vec).cap) {					\
		(vec).cap = (vec).cap ? 2 * (vec).cap : 8;		\
		(vec).v = xrealloc((vec).v, (vec).cap * sizeof(*(vec).v)); \
	}								\
	(vec).v[(vec).n++] = (x);					\
} while (0)

typedef VEC(char *) strvec_t;

void *xmalloc(size_t size);
void *xcalloc(size_t n, size_t size);
void *xrealloc(void *p, size_t size);
char *xstrdup(const char *s);
char *xstrndup(const char *s, size_t n);
char *xasprintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int strieq(const char *a, const char *b);

struct strbuf {
	char *s;
	size_t len, cap;
};

void sb_putc(struct strbuf *sb, char c);
void sb_putn(struct strbuf *sb, const char *s, size_t n);
void sb_puts(struct strbuf *sb, const char *s);
void sb_printf(struct strbuf *sb, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
const char *sb_cstr(struct strbuf *sb);
char *sb_steal(struct strbuf *sb);
void sb_reset(struct strbuf *sb);

/* A source position; expansions chain back to the line that triggered them. */
struct srcloc {
	const char *file;
	int line;
	const struct srcloc *parent;
	const char *via;
};

struct srcloc *srcloc_new(const char *file, int line,
			  const struct srcloc *parent, const char *via);

void diag_error(const struct srcloc *loc, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void diag_warn(const struct srcloc *loc, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void diag_note(const struct srcloc *loc, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
__attribute__((noreturn))
void diag_fatal(const struct srcloc *loc, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void diag_echo(const char *text);
void diag_set_werror(int on);
unsigned int diag_errors(void);
/* While buffering, diagnostics are kept until diag_flush() or the next diag_buffer(). */
void diag_buffer(int on);
void diag_flush(void);
/* Nestable: while quiet, diagnostics are dropped and not counted. */
void diag_quiet(int on);

#endif /* B43_MASM_UTIL_H_ */
