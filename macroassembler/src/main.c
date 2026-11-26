#include "output.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(FILE *f, const char *prog)
{
	fprintf(f,
		"Usage: %s INPUT OUTPUT [OPTIONS]\n"
		"  -a|--arch N          Target architecture (5 or 15)\n"
		"  -r|--corerev N       Target 802.11 core revision (sets the architecture)\n"
		"  -f|--format FMT      Output format: b43 (default), raw-le32, raw-be32\n"
		"  -e|--ivext EXT       Initvals file name extension (default .initvals)\n"
		"  -I DIR               Add an include directory\n"
		"  -D NAME[=VALUE]      Define a symbol\n"
		"  -m|--map FILE        Write a memory, procedure and symbol map\n"
		"  -l|--listing FILE    Write a listing\n"
		"  -W|--werror          Treat warnings as errors\n"
		"  -s|--psize           Print the size of the generated files\n"
		"  -h|--help            Print this help\n",
		prog);
}

static int optarg_of(int argc, char **argv, int *i, const char *s, const char *l,
		     const char **val)
{
	const char *a = argv[*i];
	size_t sl = strlen(s);

	if (strcmp(a, s) == 0 || (l && strcmp(a, l) == 0)) {
		if (*i + 1 >= argc) {
			fprintf(stderr, "b43-masm: %s needs an argument\n", a);
			exit(1);
		}
		*val = argv[++*i];
		return 1;
	}
	if (sl == 2 && strncmp(a, s, 2) == 0 && a[2]) {
		*val = a + 2;
		return 1;
	}
	return 0;
}

static int number(const char *s, int *out)
{
	char *end;
	long v = strtol(s, &end, 0);

	if (!*s || *end)
		return -1;
	*out = (int)v;
	return 0;
}

int main(int argc, char **argv)
{
	struct asm_opts opts;
	struct asm_result res;
	const char *in = NULL, *out = NULL, *map = NULL, *lst = NULL, *val;
	const char *ivext = ".initvals";
	enum outfmt fmt = FMT_B43;
	int i, print_size = 0, err;
	char *dir = NULL, *slash;

	memset(&opts, 0, sizeof(opts));
	for (i = 1; i < argc; i++) {
		const char *a = argv[i];

		if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
			usage(stdout, argv[0]);
			return 0;
		} else if (strcmp(a, "-W") == 0 || strcmp(a, "--werror") == 0) {
			diag_set_werror(1);
		} else if (strcmp(a, "-s") == 0 || strcmp(a, "--psize") == 0) {
			print_size = 1;
		} else if (optarg_of(argc, argv, &i, "-a", "--arch", &val)) {
			if (number(val, &opts.arch) || (opts.arch != 5 && opts.arch != 15)) {
				fprintf(stderr, "b43-masm: architecture must be 5 or 15\n");
				return 1;
			}
		} else if (optarg_of(argc, argv, &i, "-r", "--corerev", &val)) {
			if (number(val, &opts.corerev) || opts.corerev < 5) {
				fprintf(stderr, "b43-masm: invalid core revision '%s'\n", val);
				return 1;
			}
		} else if (optarg_of(argc, argv, &i, "-f", "--format", &val)) {
			if (strcmp(val, "b43") == 0)
				fmt = FMT_B43;
			else if (strcmp(val, "raw-le32") == 0)
				fmt = FMT_RAW_LE32;
			else if (strcmp(val, "raw-be32") == 0)
				fmt = FMT_RAW_BE32;
			else {
				fprintf(stderr, "b43-masm: unknown format '%s'\n", val);
				return 1;
			}
		} else if (optarg_of(argc, argv, &i, "-e", "--ivext", &val)) {
			ivext = val;
		} else if (optarg_of(argc, argv, &i, "-I", NULL, &val)) {
			VEC_PUSH(opts.incdirs, xstrdup(val));
		} else if (optarg_of(argc, argv, &i, "-D", NULL, &val)) {
			VEC_PUSH(opts.defines, xstrdup(val));
		} else if (optarg_of(argc, argv, &i, "-m", "--map", &val)) {
			map = val;
		} else if (optarg_of(argc, argv, &i, "-l", "--listing", &val)) {
			lst = val;
		} else if (a[0] == '-' && a[1]) {
			fprintf(stderr, "b43-masm: unknown option '%s'\n", a);
			usage(stderr, argv[0]);
			return 1;
		} else if (!in) {
			in = a;
		} else if (!out) {
			out = a;
		} else {
			usage(stderr, argv[0]);
			return 1;
		}
	}
	if (!in || !out) {
		usage(stderr, argv[0]);
		return 1;
	}
	if (opts.corerev) {
		int arch = opts.corerev < 15 ? 5 : 15;

		if (opts.arch && opts.arch != arch) {
			fprintf(stderr, "b43-masm: core revision %d is arch %d\n", opts.corerev, arch);
			return 1;
		}
		opts.arch = arch;
	}
	opts.listing = lst != NULL;
	if (asm_run(in, &opts, &res))
		return 1;
	slash = strrchr(out, '/');
	if (slash)
		dir = xstrndup(out, slash - out);
	err = write_ucode(&res, out, fmt, print_size);
	err |= write_initvals(&res, dir, ivext, print_size);
	if (map)
		err |= write_map(&res, map);
	if (lst)
		err |= write_listing(&res, lst);
	free(dir);
	return err ? 1 : 0;
}
