// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "highlighter.hpp"

#include <QFileInfo>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QPalette>
#include <QScrollBar>

namespace gzgui {
namespace {

bool darkTheme() { return QGuiApplication::palette().color(QPalette::Base).lightness() < 128; }

QTextCharFormat fmt(const char* light, const char* dark, bool bold = false, bool italic = false) {
  QTextCharFormat f;
  f.setForeground(QColor(darkTheme() ? dark : light));
  if (bold) f.setFontWeight(QFont::Bold);
  f.setFontItalic(italic);
  return f;
}

}  // namespace

Language languageForFile(const QString& path) {
  const QString suffix = QFileInfo(path).suffix().toLower();
  if (suffix == "py") return Language::Python;
  if (suffix == "yaml" || suffix == "yml") return Language::Yaml;
  if (suffix == "urdf" || suffix == "sdf" || suffix == "xml" || suffix == "xacro") return Language::Xml;
  return Language::Plain;
}

CodeHighlighter::CodeHighlighter(QTextDocument* doc, Language language) : QSyntaxHighlighter(doc), language_(language) {
  tag_ = fmt("#0b5394", "#79b8ff", true);
  attr_ = fmt("#8a4b00", "#ffab70");
  string_ = fmt("#1a7f37", "#85e89d");
  comment_ = fmt("#6a737d", "#8b949e", false, true);
  buildRules();
}

void CodeHighlighter::setLanguage(Language language) {
  if (language == language_) return;
  language_ = language;
  buildRules();
  rehighlight();
}

void CodeHighlighter::buildRules() {
  rules_.clear();
  auto add = [this](const QString& pattern, const QTextCharFormat& f, int group = 0) {
    rules_.push_back({QRegularExpression(pattern), f, group});
  };
  switch (language_) {
    case Language::Yaml:
      add(R"(^\s*(?:-\s+)?([A-Za-z_][\w./-]*)\s*:(?:\s|$))", tag_, 1);  // keys
      add(R"(-?\b\d+(?:\.\d+)?(?:[eE][-+]?\d+)?\b)", attr_);
      add(R"(\b(?:true|false|null)\b)", attr_);
      add(R"("[^"]*"|'[^']*')", string_);
      add(R"((?:^|\s)(#.*)$)", comment_, 1);
      break;
    case Language::Python: {
      add(R"(\b(?:import|from|as|def|return|for|in|if|else|elif|not|and|or|is|None|True|False|class|with|try|except|lambda|pass)\b)", tag_);
      add(R"(-?\b\d+(?:\.\d+)?\b)", attr_);
      add(R"("(?:[^"\\]|\\.)*"|'(?:[^'\\]|\\.)*')", string_);
      add(R"((#.*)$)", comment_, 1);
      break;
    }
    case Language::Log: {
      QTextCharFormat bad = fmt("#b42318", "#ff7b72", true);
      QTextCharFormat warn = fmt("#8a5a00", "#e3b341");
      QTextCharFormat good = fmt("#1a7f37", "#85e89d");
      QTextCharFormat note = fmt("#0b5394", "#79b8ff", true);
      // Later rules win, so the general tag comes first and the verdicts override it.
      add(R"(^\[(?:gz_bridge_helper|check|build|launch)\].*$)", note);
      add(R"(^.*(?:\[ERROR\]|\bERROR\b|\bFAIL\b|\bError:|Traceback|process has died).*$)", bad);
      add(R"(^.*(?:\[WARN\]|\bWARNING\b|\bWarning:).*$)", warn);
      add(R"(^.*\bPASS\b.*$)", good);
      break;
    }
    case Language::Xml:
    case Language::Plain:
      break;
  }
}

void CodeHighlighter::highlightBlock(const QString& text) {
  if (language_ == Language::Xml) {
    highlightXml(text);
    return;
  }
  for (const Rule& r : rules_) {
    auto it = r.re.globalMatch(text);
    while (it.hasNext()) {
      const auto m = it.next();
      if (m.capturedStart(r.group) < 0) continue;
      setFormat(m.capturedStart(r.group), m.capturedLength(r.group), r.format);
    }
  }
}

void CodeHighlighter::highlightXml(const QString& text) {
  static const QRegularExpression tags(QStringLiteral("</?[A-Za-z_:][\\w:.\\-]*|/?>|<\\?|\\?>"));
  static const QRegularExpression attrs(QStringLiteral("[A-Za-z_:][\\w:.\\-]*(?==\")"));
  static const QRegularExpression strings(QStringLiteral("\"[^\"]*\""));
  int pos = 0;
  setCurrentBlockState(0);
  if (previousBlockState() == 1) {  // inside a comment that started on an earlier line
    const int end = text.indexOf("-->");
    if (end < 0) {
      setFormat(0, text.length(), comment_);
      setCurrentBlockState(1);
      return;
    }
    setFormat(0, end + 3, comment_);
    pos = end + 3;
  }
  auto apply = [&](int from, int to, const QRegularExpression& re, const QTextCharFormat& f) {
    auto it = re.globalMatch(text, from);
    while (it.hasNext()) {
      const auto m = it.next();
      if (m.capturedStart() >= to) break;
      setFormat(m.capturedStart(), m.capturedLength(), f);
    }
  };
  while (pos < text.length()) {
    const int open = text.indexOf("<!--", pos);
    const int segEnd = open < 0 ? text.length() : open;
    apply(pos, segEnd, tags, tag_);
    apply(pos, segEnd, attrs, attr_);
    apply(pos, segEnd, strings, string_);
    if (open < 0) break;
    const int close = text.indexOf("-->", open + 4);
    if (close < 0) {
      setFormat(open, text.length() - open, comment_);
      setCurrentBlockState(1);
      return;
    }
    setFormat(open, close + 3 - open, comment_);
    pos = close + 3;
  }
}

QPlainTextEdit* makePreview(Language language) {
  auto* e = new QPlainTextEdit;
  e->setReadOnly(true);
  e->setLineWrapMode(QPlainTextEdit::NoWrap);
  e->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  e->setTabStopDistance(2 * e->fontMetrics().horizontalAdvance(' '));
  new CodeHighlighter(e->document(), language);
  return e;
}

void setPreviewText(QPlainTextEdit* view, const QString& text) {
  if (view->toPlainText() == text) return;
  const int scroll = view->verticalScrollBar()->value();
  const int hscroll = view->horizontalScrollBar()->value();
  view->setPlainText(text);
  view->verticalScrollBar()->setValue(scroll);
  view->horizontalScrollBar()->setValue(hscroll);
}

}  // namespace gzgui
