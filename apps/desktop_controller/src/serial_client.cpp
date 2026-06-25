#include "serial_client.h"

#include "protocol.h"

#include <QtCore/QTimer>

namespace rcj {

SerialClient::SerialClient(QObject *parent)
    : QObject(parent)
{
    connect(&serial_, &QSerialPort::readyRead, this, &SerialClient::readSerialData);
    connect(&serial_, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error == QSerialPort::NoError) {
            return;
        }
        emit logLine(QStringLiteral("串口错误：%1").arg(serial_.errorString()));
    });
}

QStringList SerialClient::availablePorts()
{
    QStringList ports;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ports << info.portName();
    }
    ports.sort();
    return ports;
}

bool SerialClient::openPort(const QString &portName)
{
    closePort();
    serial_.setPortName(portName);
    serial_.setBaudRate(QSerialPort::Baud115200);
    serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);
    serial_.setStopBits(QSerialPort::OneStop);
    serial_.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial_.open(QIODevice::ReadWrite)) {
        emit logLine(QStringLiteral("打开串口失败 %1：%2").arg(portName, serial_.errorString()));
        emit connectionChanged(false);
        return false;
    }

    emit logLine(QStringLiteral("已打开串口：%1").arg(portName));
    emit connectionChanged(true);
    return true;
}

void SerialClient::closePort()
{
    if (serial_.isOpen()) {
        emit logLine(QStringLiteral("已关闭串口：%1").arg(serial_.portName()));
        serial_.close();
    }
    emit connectionChanged(isConnected());
}

bool SerialClient::isConnected() const
{
    return simulated_ || serial_.isOpen();
}

void SerialClient::setSimulated(bool enabled)
{
    simulated_ = enabled;
    emit logLine(enabled ? QStringLiteral("模拟回复：开启") : QStringLiteral("模拟回复：关闭"));
    emit connectionChanged(isConnected());
}

bool SerialClient::isSimulated() const
{
    return simulated_;
}

void SerialClient::sendPayload(const QString &payload)
{
    const QString frame = buildFrame(payload);
    emit logLine(QStringLiteral("TX: %1").arg(frame.trimmed()));

    if (simulated_) {
        emitSimulatedReply(payload);
        return;
    }

    if (!serial_.isOpen()) {
        emit logLine(QStringLiteral("发送跳过：串口未连接"));
        return;
    }

    serial_.write(frame.toUtf8());
}

void SerialClient::readSerialData()
{
    rxBuffer_.append(serial_.readAll());
    while (true) {
        const int newline = rxBuffer_.indexOf('\n');
        if (newline < 0) {
            break;
        }

        QByteArray line = rxBuffer_.left(newline + 1);
        rxBuffer_.remove(0, newline + 1);
        line.replace("\r", "");
        handleLine(QString::fromUtf8(line).trimmed());
    }
}

void SerialClient::handleLine(const QString &line)
{
    if (line.isEmpty()) {
        return;
    }

    emit logLine(QStringLiteral("RX: %1").arg(line));
    const ParsedFrame parsed = parseFrameLine(line);
    if (!parsed.valid) {
        emit logLine(QStringLiteral("接收帧无效：%1").arg(parsed.error));
        return;
    }

    emit payloadReceived(parsed.payload);
}

void SerialClient::emitSimulatedReply(const QString &payload)
{
    const QString command = payload.section(QLatin1Char(' '), 0, 0);
    const QString args = payload.mid(command.size());

    if (command == QStringLiteral("cmd_request")) {
        QTimer::singleShot(30, this, [this]() {
            emit payloadReceived(QStringLiteral("cmd_request 0.00 0.00 0.00 0.00"));
            emit logLine(QStringLiteral("SIM RX: cmd_request 0.00 0.00 0.00 0.00"));
        });
        return;
    }

    QTimer::singleShot(40, this, [this, command, args]() {
        const QString ok = QStringLiteral("%1 ok%2").arg(command, args);
        emit logLine(QStringLiteral("SIM RX: %1").arg(ok));
        emit payloadReceived(ok);
    });

    if (command == QStringLiteral("cmd_dis") || command == QStringLiteral("cmd_turn")) {
        QTimer::singleShot(450, this, [this, command, args]() {
            const QString done = QStringLiteral("%1 done%2").arg(command, args);
            emit logLine(QStringLiteral("SIM RX: %1").arg(done));
            emit payloadReceived(done);
        });
    }
}

} // namespace rcj
