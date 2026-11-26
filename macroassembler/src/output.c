#include "output.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FW_TYPE_UCODE	'u'
#define FW_TYPE_IV	'i'
#define FW_HDR_VER	0x01

#define IV_OFFSET_MASK	0x7FFF
#define IV_32BIT	0x8000

struct ival {
	uint32_t offset;
	int wide;
	uint32_t value;
};

typedef VEC(struct ival) ivals_t;

static void put_be32(unsigned char *p, uint32_t v)
{
	p[0] = v >> 24;
	p[1] = v >> 16;
	p[2] = v >> 8;
	p[3] = v;
}

static void put_le32(unsigned char *p, uint32_t v)
{
	p[0] = v;
	p[1] = v >> 8;
	p[2] = v >> 16;
	p[3] = v >> 24;
}

static int write_header(FILE *f, char type, uint32_t size)
{
	unsigned char h[8] = { (unsigned char)type, FW_HDR_VER, 0, 0 };

	put_be32(h + 4, size);
	return fwrite(h, sizeof(h), 1, f) == 1 ? 0 : -1;
}

static FILE *open_out(const char *path)
{
	FILE *f = fopen(path, "wb");

	if (!f)
		fprintf(stderr, "b43-masm: cannot create '%s'\n", path);
	return f;
}

static int close_out(FILE *f, const char *path, int err)
{
	if (fclose(f) || err) {
		fprintf(stderr, "b43-masm: error writing '%s'\n", path);
		return -1;
	}
	return 0;
}

int write_ucode(const struct asm_result *r, const char *path, enum outfmt fmt,
		int print_size)
{
	uint64_t *image;
	size_t n = 0, i, k;
	FILE *f;
	int err = 0;

	for (i = 0; i < r->segs.n; i++) {
		const struct segment *s = r->segs.v[i];

		if (s->space == SP_CODE && s->size && (size_t)(s->base + s->size) > n)
			n = s->base + s->size;
	}
	image = xcalloc(n, sizeof(*image));
	for (i = 0; i < r->segs.n; i++) {
		const struct segment *s = r->segs.v[i];

		if (s->space != SP_CODE)
			continue;
		for (k = 0; k < s->cells.n; k++)
			if (s->cells.v[k].set)
				image[s->base + k] = s->cells.v[k].v;
	}
	f = open_out(path);
	if (!f) {
		free(image);
		return -1;
	}
	if (fmt == FMT_B43)
		err |= write_header(f, FW_TYPE_UCODE, (uint32_t)(8 * n));
	for (i = 0; i < n && !err; i++) {
		unsigned char b[8];
		uint32_t lo = (uint32_t)image[i], hi = (uint32_t)(image[i] >> 32);

		if (fmt == FMT_RAW_LE32) {
			put_le32(b, lo);
			put_le32(b + 4, hi);
		} else {
			put_be32(b, lo);
			put_be32(b + 4, hi);
		}
		err |= fwrite(b, sizeof(b), 1, f) != 1;
	}
	free(image);
	if (print_size)
		printf("%s:  text = %zu instructions (%zu bytes)\n", path, n, n * 8);
	return close_out(f, path, err);
}

static void iv_add(ivals_t *iv, uint32_t offset, int wide, uint32_t value)
{
	struct ival v = { offset, wide, value };

	VEC_PUSH(*iv, v);
}

static void shm_control(ivals_t *iv, uint32_t routing, uint32_t offset)
{
	iv_add(iv, 0x160, 1, (routing & 0xFFFF) << 16 | (offset & 0xFFFF));
}

static void shm_write(ivals_t *iv, uint32_t routing, uint32_t offset,
		      uint32_t value, int wide)
{
	if ((routing & 0xFF) != 0x01) {
		shm_control(iv, routing, offset);
		iv_add(iv, 0x164, wide, value);
		return;
	}
	if (!(offset & 3)) {
		shm_control(iv, routing, offset >> 2);
		iv_add(iv, 0x164, wide, value);
		return;
	}
	shm_control(iv, routing, offset >> 2);
	if (!wide) {
		iv_add(iv, 0x166, 0, value);
		return;
	}
	iv_add(iv, 0x166, 0, value >> 16);
	shm_control(iv, routing, (offset >> 2) + 1);
	iv_add(iv, 0x164, 0, value & 0xFFFF);
}

static void lower_op(ivals_t *iv, const struct ivop *op)
{
	switch (op->kind) {
	case IV_MMIO16:
		iv_add(iv, op->offset, 0, op->value);
		break;
	case IV_MMIO32:
		iv_add(iv, op->offset, 1, op->value);
		break;
	case IV_PHY:
		iv_add(iv, 0x3FC, 0, op->offset);
		iv_add(iv, 0x3FE, 0, op->value);
		break;
	case IV_RADIO:
		iv_add(iv, 0x3F6, 0, op->offset);
		iv_add(iv, 0x3FA, 0, op->value);
		break;
	case IV_SHM16:
		shm_write(iv, op->routing, op->offset, op->value, 0);
		break;
	case IV_SHM32:
		shm_write(iv, op->routing, op->offset, op->value, 1);
		break;
	case IV_TRAM:
		iv_add(iv, 0x130, 1, op->offset);
		iv_add(iv, 0x134, 1, op->value);
		break;
	}
}

/* SHM words are word-addressed in the PSM and byte-addressed by the host. */
static void lower_segment(ivals_t *iv, const struct segment *s)
{
	size_t k;

	for (k = 0; k < s->cells.n; k++) {
		uint32_t w = (uint32_t)(s->base + k);
		const struct cell *c = &s->cells.v[k];

		if (!c->set)
			continue;
		if (!(w & 1) && k + 1 < s->cells.n && s->cells.v[k + 1].set) {
			shm_write(iv, 1, 2 * w, (uint32_t)(c->v & 0xFFFF) |
				  (uint32_t)(s->cells.v[k + 1].v & 0xFFFF) << 16, 1);
			k++;
		} else {
			shm_write(iv, 1, 2 * w, (uint32_t)(c->v & 0xFFFF), 0);
		}
	}
}

static int write_section(const struct asm_result *r, const struct ivsect *sect,
			 const char *dir, const char *ext, int print_size)
{
	ivals_t iv = { 0 };
	char *path;
	size_t i, bytes = 0;
	FILE *f;
	int err = 0;

	for (i = 0; i < sect->ops.n; i++)
		lower_op(&iv, &sect->ops.v[i]);
	for (i = 0; i < r->segs.n; i++) {
		const struct segment *s = r->segs.v[i];

		if (s->space == SP_SHM && s->ivname && strcmp(s->ivname, sect->name) == 0)
			lower_segment(&iv, s);
	}
	if (!iv.n) {
		free(iv.v);
		return 0;
	}
	path = dir ? xasprintf("%s/%s%s", dir, sect->name, ext) : xasprintf("%s%s", sect->name, ext);
	f = open_out(path);
	if (!f) {
		free(path);
		free(iv.v);
		return -1;
	}
	err |= write_header(f, FW_TYPE_IV, (uint32_t)iv.n);
	for (i = 0; i < iv.n && !err; i++) {
		unsigned char b[6];
		size_t len = iv.v[i].wide ? 6 : 4;

		if (iv.v[i].offset & ~IV_OFFSET_MASK) {
			fprintf(stderr, "b43-masm: initval offset 0x%X too big in %s\n",
				iv.v[i].offset, sect->name);
			err = 1;
			break;
		}
		b[0] = (iv.v[i].offset | (iv.v[i].wide ? IV_32BIT : 0)) >> 8;
		b[1] = iv.v[i].offset;
		if (iv.v[i].wide) {
			put_be32(b + 2, iv.v[i].value);
		} else {
			b[2] = iv.v[i].value >> 8;
			b[3] = iv.v[i].value;
		}
		err |= fwrite(b, len, 1, f) != 1;
		bytes += len;
	}
	if (print_size)
		printf("%s:  %zu values (%zu bytes)\n", path, iv.n, bytes);
	err = close_out(f, path, err);
	free(path);
	free(iv.v);
	return err;
}

int write_initvals(const struct asm_result *r, const char *dir, const char *ext,
		   int print_size)
{
	size_t i;
	int err = 0;

	for (i = 0; i < r->ivs.n; i++)
		err |= write_section(r, r->ivs.v[i], dir, ext, print_size);
	return err;
}

static const char *const space_names[] = { "CODE", "SHM", "GPR" };

static void map_proc(FILE *f, const struct proc *p, int arch)
{
	struct strbuf sb = { 0 };
	size_t i;

	if (p->c.uses_all)
		sb_puts(&sb, "ALL");
	else
		regset_format(&p->c.regs, &sb);
	fprintf(f, "  %-24s", p->name);
	if (arch == 5)
		fprintf(f, " link lr%d  lr live:", p->c.link);
	for (i = 0; arch == 5 && i < NR_LR; i++)
		if (p->linkset & (1u << i))
			fprintf(f, " lr%zu", i);
	if (arch == 15)
		fprintf(f, " stack %d", p->depth);
	fprintf(f, "  writes %s\n", sb.len ? sb_cstr(&sb) : "-");
	for (i = 0; i < p->c.params.n; i++) {
		const struct param *q = &p->c.params.v[i];
		static const char *const dirs[] = { "IN", "OUT", "INOUT" };

		fprintf(f, "      %-5s %-16s %s%lld\n", dirs[q->dir], q->name,
			q->reg.rc == RC_GPR ? "r" : "off", (long long)q->reg.n);
	}
	free(sb.s);
}

int write_map(const struct asm_result *r, const char *path)
{
	static const char *const kinds[] = {
		"const", "var", "text", "label", "data", "type", "macro", "proc", "segment",
	};
	FILE *f = open_out(path);
	struct sym **syms;
	size_t i, n;

	if (!f)
		return -1;
	fprintf(f, "arch %d", r->arch);
	if (r->corerev)
		fprintf(f, ", core revision %d", r->corerev);
	fprintf(f, ", %u passes\n\nSegments\n", r->passes);
	for (i = 0; i < r->segs.n; i++) {
		const struct segment *s = r->segs.v[i];

		if (!s->size && !s->has_at)
			continue;
		fprintf(f, "  %-24s %-4s 0x%04llX..0x%04llX  %5u%s%s\n", s->name,
			space_names[s->space], (long long)s->base,
			(long long)(s->base + s->size - (s->size ? 1 : 0)), s->size,
			s->ivname ? "  INIT " : "", s->ivname ? s->ivname : "");
	}
	fprintf(f, "\nProcedures\n");
	for (i = 1; i < r->procs.n; i++)
		if (r->procs.v[i]->seen_pass == (int)r->passes)
			map_proc(f, r->procs.v[i], r->arch);
	if (r->arch == 15)
		fprintf(f, "  top level needs %d return stack entries\n", r->procs.v[0]->depth);
	fprintf(f, "\nSymbols\n");
	n = symtab_list(r->globals, &syms);
	for (i = 0; i < n; i++) {
		const struct sym *s = syms[i];

		fprintf(f, "  %-32s %-8s", s->name, kinds[s->kind]);
		if (s->kind == S_LABEL || s->kind == S_DATA || s->kind == S_PROC)
			fprintf(f, " %s:0x%04llX", s->seg->name, (long long)(s->seg->base + s->off));
		else if ((s->kind == S_CONST || s->kind == S_REDEF) && s->val.kind == V_NUM)
			fprintf(f, " 0x%llX", (long long)s->val.n);
		else if ((s->kind == S_CONST || s->kind == S_REDEF) && s->val.kind == V_REG)
			fprintf(f, s->val.rc == RC_SPR ? " spr%03llx" : s->val.rc == RC_GPR ? " r%lld" :
				s->val.rc == RC_OFF ? " off%lld" : " lr%lld", (long long)s->val.n);
		else if (s->kind == S_TYPE)
			fprintf(f, " %u words", s->type->size);
		fputc('\n', f);
	}
	free(syms);
	return close_out(f, path, 0);
}

int write_listing(const struct asm_result *r, const char *path)
{
	FILE *f = open_out(path);
	size_t i;

	if (!f)
		return -1;
	for (i = 0; i < r->listing.n; i++) {
		const struct listent *le = &r->listing.v[i];
		unsigned int k = le->from;
		const char *mark = le->loc->parent ? "+" : " ";

		do {
			if (le->seg && k < le->to) {
				const struct cell *c = k < le->seg->cells.n ? &le->seg->cells.v[k] : NULL;

				fprintf(f, "%04llX ", (long long)(le->seg->base + k));
				if (c && c->set)
					fprintf(f, le->seg->space == SP_CODE ? "%016llX" : "%04llX            ",
						(unsigned long long)c->v);
				else
					fprintf(f, "%-16s", "?");
			} else {
				fprintf(f, "%-21s", "");
			}
			if (k == le->from)
				fprintf(f, " %s%s\n", mark, le->text);
			else
				fputc('\n', f);
		} while (++k < le->to);
	}
	return close_out(f, path, 0);
}
