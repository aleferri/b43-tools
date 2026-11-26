#include "masm.h"

#include <stdlib.h>
#include <string.h>

#define NR_BUCKETS	512

struct symtab {
	struct sym *b[NR_BUCKETS];
	size_t n;
};

static unsigned int hash(const char *s)
{
	unsigned int h = 5381;

	while (*s)
		h = h * 33 + (unsigned char)*s++;
	return h % NR_BUCKETS;
}

struct symtab *symtab_new(void)
{
	return xcalloc(1, sizeof(struct symtab));
}

struct sym *sym_get(struct symtab *t, const char *name)
{
	struct sym *s;

	if (!t)
		return NULL;
	for (s = t->b[hash(name)]; s; s = s->next)
		if (strcmp(s->name, name) == 0)
			return s;
	return NULL;
}

struct sym *sym_put(struct symtab *t, const char *name, enum sym_kind kind,
		    const struct srcloc *loc)
{
	struct sym *s = xcalloc(1, sizeof(*s));
	unsigned int h = hash(name);

	s->name = xstrdup(name);
	s->kind = kind;
	s->loc = loc;
	s->next = t->b[h];
	t->b[h] = s;
	t->n++;
	return s;
}

void sym_remove(struct symtab *t, const char *name)
{
	struct sym **pp;

	for (pp = &t->b[hash(name)]; *pp; pp = &(*pp)->next) {
		if (strcmp((*pp)->name, name) == 0) {
			*pp = (*pp)->next;
			t->n--;
			return;
		}
	}
}

static int cmp_sym(const void *a, const void *b)
{
	return strcmp((*(struct sym *const *)a)->name, (*(struct sym *const *)b)->name);
}

size_t symtab_list(struct symtab *t, struct sym ***out)
{
	struct sym **v = xmalloc(t->n * sizeof(*v));
	size_t i, k = 0;
	struct sym *s;

	for (i = 0; i < NR_BUCKETS; i++)
		for (s = t->b[i]; s; s = s->next)
			v[k++] = s;
	qsort(v, k, sizeof(*v), cmp_sym);
	*out = v;
	return k;
}
