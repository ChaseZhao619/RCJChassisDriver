#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QStringList>
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

    QSerialPort serial_;
    QByteArray rxBuffer_;
    bool simulated_ = false;
};

} // namespace rcj
