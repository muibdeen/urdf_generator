// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// One external command (colcon, ros2, gz) run in its own process group so that stopping it also
// stops everything it started (`ros2 launch` starts Gazebo, the bridge, ...).
#pragma once

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTimer>

namespace gzgui {

class ProcessJob : public QObject {
  Q_OBJECT

 public:
  struct Result {
    int exitCode = -1;
    bool crashed = false;        // killed by a signal (this includes being stopped)
    bool failedToStart = false;  // not even the shell could be started
    bool timedOut = false;       // stopped because the time limit ran out
    bool stopped = false;        // stopped by stop()
    QString output;              // everything printed (stdout and stderr), without colour codes
    bool programMissing() const { return exitCode == 127; }  // the shell could not find the program
  };

  explicit ProcessJob(QObject* parent = nullptr);
  ~ProcessJob() override;

  /// Runs `argv` (program first) after sourcing each of `setupScripts` in a bash. Nothing is
  /// interpreted by a shell: the arguments reach the program exactly as given.
  void start(const QStringList& setupScripts, const QStringList& argv, const QString& workDir = QString(), int timeoutMs = 0);
  /// Sends `signal` to the whole process group; after `graceMs` it escalates to SIGTERM, then SIGKILL.
  void stop(int signal = 2 /* SIGINT */, int graceMs = 8000);

  bool running() const { return process_.state() != QProcess::NotRunning; }
  qint64 processId() const { return process_.processId(); }
  const QString& output() const { return output_; }

 signals:
  void outputAvailable(const QString& chunk);
  void finished(const ProcessJob::Result& result);

 private slots:
  void onStarted();
  void onReadyRead();
  void onFinished(int exitCode, QProcess::ExitStatus status);
  void onErrorOccurred(QProcess::ProcessError error);
  void escalate();

 private:
  void signalGroup(int signal, bool fallbackToProcess = true);
  void appendOutput(const QByteArray& bytes);

  QProcess process_;
  QTimer timeout_, grace_;
  QString output_;
  qint64 pgid_ = 0;  // the shell is the leader of its own group, so the group id is its process id
  int escalationStep_ = 0;
  bool stopping_ = false, timedOut_ = false, failedToStart_ = false, reported_ = false;
};

/// "\x1b[31mred\x1b[0m" -> "red"; carriage returns become newlines.
QString stripTerminalCodes(const QString& text);

}  // namespace gzgui
