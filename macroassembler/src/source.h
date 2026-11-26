#ifndef B43_MASM_SOURCE_H_
#define B43_MASM_SOURCE_H_

#include "util.h"

struct bodyline {
	char *text;
	const struct srcloc *loc;
};

typedef VEC(struct bodyline) body_t;

enum prov_kind {
	PK_FILE,
	PK_MACRO,
	PK_REPEAT,
	PK_WHILE,
};

struct line {
	const char *text;
	const struct srcloc *loc;
};

struct src_hooks {
	/* A provider ran out of lines; cond_base is the conditional depth it started at. */
	void (*popped)(enum prov_kind kind, size_t cond_base, const struct srcloc *end);
	/* Evaluates a WHILE condition. */
	int (*eval)(const char *expr, const struct srcloc *loc);
};

void src_init(const struct src_hooks *hooks, char **incdirs, size_t nr_incdirs);
int src_push_file(const char *name, const struct srcloc *from, size_t cond_base);
void src_push_lines(enum prov_kind kind, body_t *lines, size_t cond_base,
		    struct strbuf *result);
void src_push_while(const char *cond, const struct srcloc *cond_loc,
		    const body_t *body, size_t cond_base);
/* Returns 0 at the end of input, or when it would drop below min_depth. */
int src_next(struct line *out, size_t min_depth);
size_t src_depth(void);
/* EXITM: drops the innermost expansion; returns its result buffer (if any). */
struct strbuf *src_exit_expansion(size_t *cond_base);
int src_in_expansion(void);
void src_reset(void);

#endif /* B43_MASM_SOURCE_H_ */
