#include "uwbdriver.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <poll.h>
#include <chrono>

UwbDriver::UwbDriver() : fd1(-1), fd2(-1), is_running(false) {}

UwbDriver::~UwbDriver() {
    closePorts();
}

int UwbDriver::configurePort(const std::string& port_name) {
    // 권한 확인을 위해 O_RDWR로 열기 시도
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror(("Error opening " + port_name).c_str());
        return -1;
    } else {
        std::cout << "Successfully opened " << port_name << " (fd: " << fd << ")" << std::endl;
    }

    // 통신 설정 (Blocking 모드 해제 등)
    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // Raw Mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

bool UwbDriver::openDualPorts(const std::string& port1, const std::string& port2) {
    fd1 = configurePort(port1);
    fd2 = configurePort(port2);

    if (fd1 == -1 && fd2 == -1) {
        std::cerr << "Failed to open ANY ports." << std::endl;
        return false;
    }

    is_running = true;
    worker_thread = std::thread(&UwbDriver::readLoop, this);
    return true;
}

void UwbDriver::closePorts() {
    is_running = false;
    if (worker_thread.joinable()) {
        worker_thread.join();
    }
    if (fd1 != -1) close(fd1);
    if (fd2 != -1) close(fd2);
    fd1 = -1;
    fd2 = -1;
}

void UwbDriver::readLoop() {
    struct pollfd fds[2];
    std::string buf1_str = "";
    std::string buf2_str = "";
    char buffer[256];

    while (is_running) {
        // 유효한 fd만 감시 등록 (음수 fd는 poll이 무시함)
        fds[0].fd = fd1;
        fds[0].events = POLLIN;
        fds[1].fd = fd2;
        fds[1].events = POLLIN;

        int ret = poll(fds, 2, 100); // 100ms Timeout

        if (ret > 0) {
            // --- Port 1 ---
            if (fd1 != -1 && (fds[0].revents & POLLIN)) {
                int n = read(fd1, buffer, sizeof(buffer) - 1);
                if (n > 0) {
                    buffer[n] = '\0';
                    buf1_str += buffer;

                    size_t pos;
                    while ((pos = buf1_str.find('\n')) != std::string::npos) {
                        parseLine(buf1_str.substr(0, pos));
                        buf1_str.erase(0, pos + 1);
                    }
                }
            }
            // --- Port 2 ---
            if (fd2 != -1 && (fds[1].revents & POLLIN)) {
                int n = read(fd2, buffer, sizeof(buffer) - 1);
                if (n > 0) {
                    buffer[n] = '\0';
                    buf2_str += buffer;

                    size_t pos;
                    while ((pos = buf2_str.find('\n')) != std::string::npos) {
                        parseLine(buf2_str.substr(0, pos));
                        buf2_str.erase(0, pos + 1);
                    }
                }
            }
        }
    }
}

void UwbDriver::parseLine(const std::string& line) {
    if (line.empty()) return;

    std::string clean_line = line;
    if (clean_line.back() == '\r') clean_line.pop_back();

    float val = 0.0f;
    bool updated = false;

    {
        std::lock_guard<std::mutex> lock(data_mutex);

        if (sscanf(clean_line.c_str(), "L:%f", &val) == 1) {
            current_l = val;
            updated = true;
        }
        else if (sscanf(clean_line.c_str(), "R:%f", &val) == 1) {
            current_r = val;
            updated = true;
        }
    }
}

void UwbDriver::getDistances(float &l_dist, float &r_dist) {
    std::lock_guard<std::mutex> lock(data_mutex);
    l_dist = current_l;
    r_dist = current_r;
}
