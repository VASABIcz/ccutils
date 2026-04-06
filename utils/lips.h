#include <string>
#include <vector>
#include <optional>
#include <span>
#include "ccutils/lexing/SourceProvider.h"
#include "ccutils/lexing/Token.h"
#include "ccutils/parsing/TokenProvider.h"

namespace lips {
struct Lips {
	std::variant<std::string, std::vector<Lips>> node;

	explicit Lips(std::string value): node(value) {}
	explicit Lips(std::vector<Lips> value): node(value) {}

	template<typename... Args>
	static Lips a(Args... args) {
		std::vector<Lips> l{args...};
	
		return Lips(l);
	}

	bool isValue() {
		return std::holds_alternative<std::string>(node);
	}

	std::string getValue() {
		return std::get<std::string>(node);
	}

	std::vector<Lips> getList() {
		return std::get<std::vector<Lips>>(node);
	}
};

enum class TokenType {
	ORB,
	CRB,
	OPB,
	CPB,
	Join,
	Ident,
};

using _Token = Token<TokenType>;

_Token tokenize(SourceProviderBase& src) {
	using enum TokenType;
	if (src.isPeekConsume('(')) return _Token{ORB};
	if (src.isPeekConsume(')')) return _Token{CRB};
	if (src.isPeekConsume('<')) return _Token{OPB};
	if (src.isPeekConsume('>')) return _Token{CPB};
	if (src.isPeekConsume(':')) return _Token{Join};

	std::string* buf = new std::string{};

	if (src.isPeekConsume('`')) {
		while (not src.isPeekConsume('`')) {
			(*buf) += src.consume();
		}

		return _Token{Ident, *buf};
	}

	while (src.isPeekCharFn([](char c) { return std::isalpha(c) || (c >= '0' && c <= '9'); })) {
		(*buf) += src.consume();
	}

	println("asdasd {}", buf);

	if ((*buf).empty()) {
		println("WHAT? {}", src.remaining());
		PANIC();
	}

	return _Token{Ident, *buf};
}

std::vector<_Token> tokenizeString(std::string _s) {
	std::vector<_Token> res;
	SourceProviderBase s{_s};

	while (not s.isDone()) {
		s.consumeWhite();

		if (s.isPeekStr("//")) {
			while (not s.isPeekConsume('\n')) {
				s.consume();
			}
			continue;
		}

		res.push_back(tokenize(s));
		s.consumeWhite();
	}

	return res;
}

using Parser = TokenProvider<TokenType>;

Lips parseLips(Parser& p, bool isList = false) {
	using enum TokenType;

	if (p.isPeekTypeConsume(ORB)) {
		std::vector<Lips> buf;
		while (not p.isPeekTypeConsume(CRB)) {
			buf.push_back(parseLips(p));
		}

		return Lips(buf);
	}

	auto tmp = Lips{std::string(p.unwrapToken(Ident).content)};

	while (p.isPeekTypeConsume(OPB)) {
		std::vector<Lips> buf;
		while (not p.isPeekTypeConsume(CPB)) {
			buf.push_back(parseLips(p));
		}
		tmp = Lips::a(tmp, Lips(buf));
	}

	if (p.isPeekType(Join) && not isList) {
		std::vector<Lips> buf{tmp};
		while (p.isPeekTypeConsume(Join)) {
			buf.push_back(parseLips(p, true));
		}
		return Lips(buf);
	}

	return tmp;
}

std::vector<Lips> parseTokens(std::span<_Token> tokens) {
	std::vector<Lips> res;
	Parser p{tokens};

	while (not p.isDone()) {
		res.push_back(parseLips(p));
	}

	return res;
}

std::vector<Lips> parseString(std::string s) {
	auto tokens = tokenizeString(s);

	println("got tokens");

	auto xd = parseTokens(tokens);
	return xd;
}

void toString(std::string& buf, Lips l) {
	if (l.isValue()) {
		buf += l.getValue();
	} else {
		buf += "(";
		for (auto l1 : l.getList()) {
			toString(buf, l1);
			buf += " ";
		}
		if (not l.getList().empty()) buf.pop_back();
		buf += ")";
	}
}
} // namespace lips