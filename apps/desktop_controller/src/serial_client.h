#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

namespace rcj {

class SerialClient : public QObject {
    Q_OBJECT

public:
    explicit SerialClient(QObject *parent = nullptr);

    static QStringList availablePorts();
    bool openPort(const QString &portName);
    void closePort();
    bool isConnected() const;
    void setSimulated(bool enabled);
    bool isSimulated() const;
    void sendPayload(const QString &payload);

signals:
    void logLine(const QString &line);
    void payloadReceived(const QString &payload);
    void connectionChanged(bool connected);

private slots:
    void readSerialData();

private:
    void handleLine(const QString &line);
    void emitSimulatedReply(const QString &payload);
    void transmitPayload(const QString &payload, bool logFrame);

    QSerialPort serial_;
    QByteArray rxBuffer_;
    bool simulated_ = false;
    QTimer continuousTimer_;
    QString continuousPayload_;
};

} // namespace rcj
