#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kReadTimeoutMs = 500;

uint16_t crc16Ccitt(std::string_view data)
{
    uint16_t crc = 0xFFFF;

    for (unsigned char byte : data) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }

    return crc;
}

std::string buildFrame(std::string_view payload)
{
    std::ostringstream frame;
    frame << payload << " *" << std::uppercase << std::hex << std::setw(4)
          << std::setfill('0') << crc16Ccitt(payload) << "\r\n";
    return frame.str();
}

std::string joinPayload(int firstArg, int argc, char *argv[])
{
    std::ostringstream payload;
    for (int i = firstArg; i < argc; ++i) {
        if (i != firstArg) {
            payload << ' ';
        }
        payload << argv[i];
    }
    return payload.str();
}

void configureSerialPort(int fd)
{
    termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        throw std::runtime_error(std::string("tcgetattr failed: ") + std::strerror(errno));
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= static_cast<unsigned int>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<unsigned int>(~PARENB);
    tty.c_cflag &= static_cast<unsigned int>(~CSTOPB);
    tty.c_cflag &= static_cast<unsigned int>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag &= static_cast<unsigned int>(~CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error(std::string("tcsetattr failed: ") + std::strerror(errno));
    }
}

void writeAll(int fd, std::string_view data)
{
    const char *cursor = data.data();
    std::size_t remaining = data.size();

    while (remaining > 0) {
        const ssize_t written = write(fd, cursor, remaining);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error(std::string("write failed: ") + std::strerror(errno));
        }

        cursor += written;
        remaining -= static_cast<std::size_t>(written);
    }
}

std::string readAvailable(int fd)
{
    std::string response;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kReadTimeoutMs);

    while (std::chrono::steady_clock::now() < deadline) {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(fd, &readSet);

        timeval timeout {};
        timeout.tv_sec = 0;
        timeout.tv_usec = 50 * 1000;

        const int ready = select(fd + 1, &readSet, nullptr, nullptr, &timeout);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error(std::string("select failed: ") + std::strerror(errno));
        }

        if (ready == 0 || !FD_ISSET(fd, &readSet)) {
            continue;
        }

        char buffer[256];
        const ssize_t count = read(fd, buffer, sizeof(buffer));
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error(std::string("read failed: ") + std::strerror(errno));
        }

        if (count == 0) {
            continue;
        }

        response.append(buffer, static_cast<std::size_t>(count));
        if (response.find('\n') != std::string::npos) {
            break;
        }
    }

    return response;
}

void printUsage(const char *program)
{
    std::cerr << "Usage:\n"
              << "  " << program << " --frame <payload...>\n"
              << "  " << program << " --port <device> <payload...>\n\n"
              << "Examples:\n"
              << "  " << program << " --frame cmd_dis 10 0\n"
              << "  " << program << " --port /dev/ttyUSB0 cmd_dis 10 0\n";
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc < 3) {
        printUsage(argv[0]);
        return 2;
    }

    try {
        const std::string mode = argv[1];

        if (mode == "--frame") {
            std::cout << buildFrame(joinPayload(2, argc, argv));
            return 0;
        }

        if (mode != "--port" || argc < 4) {
            printUsage(argv[0]);
            return 2;
        }

        const std::string device = argv[2];
        const std::string frame = buildFrame(joinPayload(3, argc, argv));

        const int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            throw std::runtime_error("open " + device + " failed: " + std::strerror(errno));
        }

        configureSerialPort(fd);
        tcflush(fd, TCIOFLUSH);
        writeAll(fd, frame);

        const std::string response = readAvailable(fd);
        close(fd);

        if (!response.empty()) {
            std::cout << response;
        }

        return 0;
    } catch (const std::exception &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
