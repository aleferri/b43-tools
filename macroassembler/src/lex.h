#ifndef B43_MASM_LEX_H_
#define B43_MASM_LEX_H_

#include "util.h"

enum tok_kind {
	T_EOL,
	T_IDENT,
	T_NUM,
	T_STR,
	T_PUNCT,
};

enum punct {
	P_LPAREN = 1, P_RPAREN, P_LBRACK, P_RBRACK, P_LBRACE, P_RBRACE,
	P_COMMA, P_COLON, P_DCOLON, P_COLONEQ, P_ASSIGN, P_DOT, P_DOTDOT,
	P_PLUS, P_MINUS, P_STAR, P_SLASH, P_PERCENT, P_SHL, P_SHR,
	P_AMP, P_PIPE, P_CARET, P_TILDE, P_BANG,
	P_EQ, P_NE, P_LT, P_GT, P_LE, P_GE, P_LAND, P_LOR,
};

struct token {
	enum tok_kind kind;
	enum punct p;
	int64_t num;
	char *s;		/* identifier name or string contents */
	unsigned int pos, end;	/* byte range in the line */
};

struct toklist {
	VEC(struct token) t;
	const char *line;
};

/* Tokenizes one logical line; on a lexical error reports it and returns -1. */
int lex_line(const char *line, struct toklist *out, const struct srcloc *loc);
void toklist_free(struct toklist *tl);

int tok_is_punct(const struct token *t, enum punct p);
int tok_is_kw(const struct token *t, const char *kw);
int is_ident_start(int c);
int is_ident_char(int c);

#endif /* B43_MASM_LEX_H_ */
