// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Light syntax colouring for the read-only previews (URDF/SDF, YAML, Python, process output).
#pragma once

#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <vector>

namespace gzgui {

enum class Language { Plain, Xml, Yaml, Python, Log };

/// Picks the language from a file name ("launch/sim.launch.py" -> Python).
Language languageForFile(const QString& path);

class CodeHighlighter : public QSyntaxHighlighter {
 public:
  CodeHighlighter(QTextDocument* doc, Language language);
  void setLanguage(Language language);
  Language language() const { return language_; }

 protected:
  void highlightBlock(const QString& text) override;

 private:
  struct Rule {
    QRegularExpression re;
    QTextCharFormat format;
    int group = 0;  // capture group that is coloured
  };
  void buildRules();
  void highlightXml(const QString& text);

  Language language_;
  std::vector<Rule> rules_;
  QTextCharFormat tag_, attr_, string_, comment_;
};

/// A read-only monospace editor with the highlighter attached.
QPlainTextEdit* makePreview(Language language);

/// Replaces the text without losing the scroll position (the previews refresh while you read).
void setPreviewText(QPlainTextEdit* view, const QString& text);

}  // namespace gzgui
