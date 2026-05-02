#pragma once

#include <QtCore/QString>
#include <QtCore/QStringList>

namespace rcj {

struct ParsedFrame {
    bool valid = false;
    QString payload;
    QString error;
    QStringList parts;
};

quint16 crc16Ccitt(const QByteArray &data);
QString buildFrame(const QString &payload);
ParsedFrame parseFrameLine(const QString &line);

} // namespace rcj
