#include "main_window.h"
#include "protocol.h"

#include <QtWidgets/QApplication>

#include <iostream>

int main(int argc, char *argv[])
{
    if (argc >= 3 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--frame")) {
        QStringList parts;
        for (int i = 2; i < argc; ++i) {
            parts << QString::fromLocal8Bit(argv[i]);
        }
        std::cout << rcj::buildFrame(parts.join(QLatin1Char(' '))).toStdString();
        return 0;
    }

    QApplication app(argc, argv);
    rcj::MainWindow window;
    window.show();
    return app.exec();
}
