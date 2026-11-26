#ifndef B43_MASM_MACRO_H_
#define B43_MASM_MACRO_H_

#include "masm.h"

struct mparam {
	char *name;
	int req;
	int vararg;
	char *def;
};

struct macro {
	char *name;
	VEC(struct mparam) params;
	body_t body;
	const struct srcloc *loc;
	char *via;
};

struct macro *macro_define(const char *name, const char *params, body_t *body,
			   const struct srcloc *loc);
int macro_expand(const struct macro *m, const char *args, const struct srcloc *loc,
		 unsigned int *uniq, body_t *out);
int repeat_body(const body_t *body, const char *param, const strvec_t *values,
		size_t times, const struct srcloc *loc, const char *via, body_t *out);
int macro_args(const char *text, strvec_t *out, const struct srcloc *loc);

#endif /* B43_MASM_MACRO_H_ */
