#include "protocol.h"

#include <QtCore/QByteArray>

namespace rcj {

quint16 crc16Ccitt(const QByteArray &data)
{
    quint16 crc = 0xFFFFU;

    for (const char byte : data) {
        crc ^= static_cast<quint16>(static_cast<unsigned char>(byte)) << 8U;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<quint16>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<quint16>(crc << 1U);
            }
        }
    }

    return crc;
}

QString buildFrame(const QString &payload)
{
    const quint16 crc = crc16Ccitt(payload.toUtf8());
    const QString crcText = QStringLiteral("%1")
        .arg(crc, 4, 16, QLatin1Char('0'))
        .toUpper();
    return QStringLiteral("%1 *%2\r\n").arg(payload, crcText);
}

ParsedFrame parseFrameLine(const QString &line)
{
    ParsedFrame parsed;
    QString trimmed = line.trimmed();
    const int star = trimmed.lastIndexOf(QLatin1Char('*'));
    if (star < 0) {
        parsed.error = QStringLiteral("missing crc");
        return parsed;
    }

    QString payload = trimmed.left(star).trimmed();
    const QString crcText = trimmed.mid(star + 1).trimmed();
    if (crcText.size() != 4) {
        parsed.error = QStringLiteral("bad crc length");
        return parsed;
    }

    bool ok = false;
    const quint16 received = crcText.toUShort(&ok, 16);
    if (!ok) {
        parsed.error = QStringLiteral("bad crc text");
        return parsed;
    }

    const quint16 calculated = crc16Ccitt(payload.toUtf8());
    if (received != calculated) {
        parsed.error = QStringLiteral("crc mismatch");
        return parsed;
    }

    parsed.valid = true;
    parsed.payload = payload;
    parsed.parts = payload.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    return parsed;
}

} // namespace rcj
