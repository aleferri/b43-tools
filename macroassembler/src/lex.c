#include "lex.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

int is_ident_start(int c)
{
	return isalpha(c) || c == '_' || c == '@' || c == '$' || c == '?';
}

int is_ident_char(int c)
{
	return is_ident_start(c) || isdigit(c);
}

int tok_is_punct(const struct token *t, enum punct p)
{
	return t->kind == T_PUNCT && t->p == p;
}

int tok_is_kw(const struct token *t, const char *kw)
{
	return t->kind == T_IDENT && strieq(t->s, kw);
}

static int digit_value(int c)
{
	if (isdigit(c))
		return c - '0';
	return tolower(c) - 'a' + 10;
}

static int parse_digits(const char *s, size_t n, int base, int64_t *out)
{
	uint64_t v = 0;
	size_t i;

	if (n == 0)
		return -1;
	for (i = 0; i < n; i++) {
		int d;

		if (s[i] == '_')
			continue;
		if (!isxdigit((unsigned char)s[i]))
			return -1;
		d = digit_value((unsigned char)s[i]);
		if (d >= base)
			return -1;
		if (v > (UINT64_MAX - d) / base)
			return -1;
		v = v * base + d;
	}
	*out = (int64_t)v;
	return 0;
}

/* 0x1F, 1Fh, 0b101, 101b, 42 */
static int parse_number(const char *s, size_t n, int64_t *out)
{
	char last = tolower((unsigned char)s[n - 1]);

	if (n > 2 && s[0] == '0' && tolower((unsigned char)s[1]) == 'x')
		return parse_digits(s + 2, n - 2, 16, out);
	if (last == 'h')
		return parse_digits(s, n - 1, 16, out);
	if (n > 2 && s[0] == '0' && tolower((unsigned char)s[1]) == 'b' &&
	    parse_digits(s + 2, n - 2, 2, out) == 0)
		return 0;
	if (last == 'b' && parse_digits(s, n - 1, 2, out) == 0)
		return 0;
	return parse_digits(s, n, 10, out);
}

static const struct {
	const char *s;
	enum punct p;
} puncts[] = {
	{ "::", P_DCOLON }, { ":=", P_COLONEQ }, { "..", P_DOTDOT },
	{ "<<", P_SHL }, { ">>", P_SHR }, { "==", P_EQ }, { "!=", P_NE },
	{ "<=", P_LE }, { ">=", P_GE }, { "&&", P_LAND }, { "||", P_LOR },
	{ "(", P_LPAREN }, { ")", P_RPAREN }, { "[", P_LBRACK }, { "]", P_RBRACK },
	{ "{", P_LBRACE }, { "}", P_RBRACE }, { ",", P_COMMA }, { ":", P_COLON },
	{ "=", P_ASSIGN }, { ".", P_DOT }, { "+", P_PLUS }, { "-", P_MINUS },
	{ "*", P_STAR }, { "/", P_SLASH }, { "%", P_PERCENT }, { "&", P_AMP },
	{ "|", P_PIPE }, { "^", P_CARET }, { "~", P_TILDE }, { "!", P_BANG },
	{ "<", P_LT }, { ">", P_GT },
};

static int ends_operand(const struct toklist *tl)
{
	const struct token *t;

	if (tl->t.n == 0)
		return 0;
	t = &tl->t.v[tl->t.n - 1];
	return t->kind == T_IDENT || t->kind == T_NUM || t->kind == T_STR ||
	       tok_is_punct(t, P_RPAREN) || tok_is_punct(t, P_RBRACK);
}

int lex_line(const char *line, struct toklist *out, const struct srcloc *loc)
{
	size_t i = 0, len = strlen(line);

	out->line = line;
	out->t.n = 0;
	while (i < len) {
		struct token tok;
		unsigned char c = line[i];
		int ws_before = i > 0 && isspace((unsigned char)line[i - 1]);

		if (isspace(c)) {
			i++;
			continue;
		}
		memset(&tok, 0, sizeof(tok));
		tok.pos = i;
		if (c == '.' && i + 1 < len && isalpha((unsigned char)line[i + 1]) &&
		    (ws_before || !ends_operand(out))) {
			size_t j = i + 1;

			while (j < len && is_ident_char((unsigned char)line[j]))
				j++;
			tok.kind = T_IDENT;
			tok.s = xstrndup(line + i, j - i);
			i = j;
		} else if (is_ident_start(c)) {
			size_t j = i;

			while (j < len && is_ident_char((unsigned char)line[j]))
				j++;
			tok.kind = T_IDENT;
			tok.s = xstrndup(line + i, j - i);
			i = j;
		} else if (isdigit(c)) {
			size_t j = i;

			while (j < len && (isalnum((unsigned char)line[j]) || line[j] == '_'))
				j++;
			tok.kind = T_NUM;
			if (parse_number(line + i, j - i, &tok.num)) {
				diag_error(loc, "invalid number '%.*s'", (int)(j - i), line + i);
				return -1;
			}
			i = j;
		} else if (c == '"' || c == '\'') {
			struct strbuf sb = { 0 };
			size_t j = i + 1;

			for (;;) {
				if (j >= len) {
					diag_error(loc, "unterminated string");
					free(sb.s);
					return -1;
				}
				if (line[j] == c) {
					if (j + 1 < len && line[j + 1] == c) {
						sb_putc(&sb, c);
						j += 2;
						continue;
					}
					break;
				}
				sb_putc(&sb, line[j++]);
			}
			tok.kind = T_STR;
			tok.s = sb.s ? sb_steal(&sb) : xstrdup("");
			i = j + 1;
		} else {
			size_t k;

			for (k = 0; k < ARRAY_SIZE(puncts); k++) {
				size_t pl = strlen(puncts[k].s);

				if (strncmp(line + i, puncts[k].s, pl) == 0) {
					tok.kind = T_PUNCT;
					tok.p = puncts[k].p;
					i += pl;
					break;
				}
			}
			if (k == ARRAY_SIZE(puncts)) {
				diag_error(loc, "unexpected character '%c'", c);
				return -1;
			}
		}
		tok.end = i;
		VEC_PUSH(out->t, tok);
	}
	{
		struct token eol;

		memset(&eol, 0, sizeof(eol));
		eol.kind = T_EOL;
		eol.pos = eol.end = len;
		VEC_PUSH(out->t, eol);
	}
	return 0;
}

void toklist_free(struct toklist *tl)
{
	size_t i;

	for (i = 0; i < tl->t.n; i++)
		free(tl->t.v[i].s);
	free(tl->t.v);
	tl->t.v = NULL;
	tl->t.n = tl->t.cap = 0;
}
