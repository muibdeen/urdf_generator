// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "process_job.hpp"

#include <QProcessEnvironment>
#include <QRegularExpression>

#ifdef Q_OS_UNIX
#include <csignal>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace gzgui {

QString stripTerminalCodes(const QString& text) {
  static const QRegularExpression ansi(QStringLiteral("\x1b\\[[0-9;?]*[ -/]*[@-~]"));
  QString out = text;
  out.remove(ansi);
  out.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
  out.replace('\r', '\n');
  return out;
}

ProcessJob::ProcessJob(QObject* parent) : QObject(parent) {
  process_.setProcessChannelMode(QProcess::MergedChannels);
  timeout_.setSingleShot(true);
  grace_.setSingleShot(true);
  connect(&process_, &QProcess::started, this, &ProcessJob::onStarted);
  connect(&process_, &QProcess::readyRead, this, &ProcessJob::onReadyRead);
  connect(&process_, &QProcess::finished, this, &ProcessJob::onFinished);
  connect(&process_, &QProcess::errorOccurred, this, &ProcessJob::onErrorOccurred);
  connect(&timeout_, &QTimer::timeout, this, [this]() {
    timedOut_ = true;
    stop(15 /* SIGTERM */, 2000);
  });
  connect(&grace_, &QTimer::timeout, this, &ProcessJob::escalate);
}

ProcessJob::~ProcessJob() {
  // Nothing may call back into the owner while it is being destroyed (waitForFinished below
  // runs Qt's own event handling, which can still report an error).
  blockSignals(true);
  timeout_.stop();
  grace_.stop();
  if (running()) {
    signalGroup(9 /* SIGKILL */);
    process_.waitForFinished(2000);
  }
}

void ProcessJob::start(const QStringList& setupScripts, const QStringList& argv, const QString& workDir, int timeoutMs) {
  output_.clear();
  pgid_ = 0;
  escalationStep_ = 0;
  stopping_ = timedOut_ = failedToStart_ = reported_ = false;

  // bash -c '<source every script up to "--", then run the rest>' <name> <scripts...> -- <program> <args...>
  QStringList args;
  args << "-c" << R"(while [ "$1" != "--" ]; do . "$1"; shift; done; shift; exec "$@")" << "gz_bridge_helper";
  args << setupScripts << "--" << argv;

  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.insert("PYTHONUNBUFFERED", "1");           // `ros2 topic hz` prints through a pipe
  env.insert("RCUTILS_COLORIZED_OUTPUT", "0");
  env.insert("ANSI_COLORS_DISABLED", "1");
  process_.setProcessEnvironment(env);
  if (!workDir.isEmpty()) process_.setWorkingDirectory(workDir);
#ifdef Q_OS_UNIX
  process_.setChildProcessModifier([]() { ::setsid(); });  // own process group: one kill reaches every child
#endif
  process_.start("bash", args);
  if (timeoutMs > 0) timeout_.start(timeoutMs);
}

void ProcessJob::onStarted() { pgid_ = process_.processId(); }

void ProcessJob::signalGroup(int signal, bool fallbackToProcess) {
#ifdef Q_OS_UNIX
  const qint64 pid = pgid_ > 0 ? pgid_ : process_.processId();
  if (pid <= 0) return;
  if (::kill(-static_cast<pid_t>(pid), signal) != 0 && fallbackToProcess && running()) ::kill(static_cast<pid_t>(pid), signal);
#else
  Q_UNUSED(signal);
  Q_UNUSED(fallbackToProcess);
  if (fallbackToProcess) process_.terminate();
#endif
}

void ProcessJob::stop(int signal, int graceMs) {
  if (!running()) return;
  stopping_ = true;
  signalGroup(signal);
  escalationStep_ = signal == 9 ? 2 : (signal == 15 ? 1 : 0);
  grace_.start(graceMs);
}

void ProcessJob::escalate() {
  if (!running()) return;
  if (escalationStep_ == 0) {
    escalationStep_ = 1;
    signalGroup(15 /* SIGTERM */);
    grace_.start(3000);
  } else {
    escalationStep_ = 2;
    signalGroup(9 /* SIGKILL */);
  }
}

void ProcessJob::appendOutput(const QByteArray& bytes) {
  if (bytes.isEmpty()) return;
  const QString text = stripTerminalCodes(QString::fromUtf8(bytes));
  output_ += text;
  constexpr int kMaxChars = 1 << 20;
  if (output_.size() > kMaxChars) output_.remove(0, output_.size() - kMaxChars);
  emit outputAvailable(text);
}

void ProcessJob::onReadyRead() { appendOutput(process_.readAll()); }

void ProcessJob::onErrorOccurred(QProcess::ProcessError error) {
  if (error != QProcess::FailedToStart) return;
  failedToStart_ = true;
  // No finished() signal follows a failed start: report it here.
  timeout_.stop();
  grace_.stop();
  if (reported_) return;
  reported_ = true;
  Result r;
  r.failedToStart = true;
  r.output = output_;
  emit finished(r);
}

void ProcessJob::onFinished(int exitCode, QProcess::ExitStatus status) {
  timeout_.stop();
  grace_.stop();
  appendOutput(process_.readAll());
  // `ros2 launch` normally takes Gazebo and the bridge down with it. If something is left in the
  // group anyway, it must not keep running unseen.
  signalGroup(15 /* SIGTERM */, false);
  pgid_ = 0;  // the number may be reused from now on: never signal it again
  if (reported_) return;
  reported_ = true;
  Result r;
  r.exitCode = exitCode;
  r.crashed = status == QProcess::CrashExit;
  r.timedOut = timedOut_;
  r.stopped = stopping_ && !timedOut_;
  r.output = output_;
  emit finished(r);
}

}  // namespace gzgui
