#ifndef UWB_DRIVER_H
#define UWB_DRIVER_H

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

class UwbDriver {
public:
    UwbDriver();
    ~UwbDriver();

    bool openDualPorts(const std::string& port1, const std::string& port2);
    void closePorts();

    // 최신 거리값 가져오기 (Thread-safe)
    void getDistances(float &l_dist, float &r_dist);

private:
    int fd1; // 첫 번째 모듈 파일 디스크립터
    int fd2; // 두 번째 모듈 파일 디스크립터

    std::atomic<bool> is_running;
    std::thread worker_thread;
    std::mutex data_mutex;

    float current_l = 0.0f;
    float current_r = 0.0f;

    // 내부 헬퍼 함수
    int configurePort(const std::string& port_name);
    void readLoop(); // poll()을 이용한 멀티플렉싱 루프
    void parseLine(const std::string& line);
};

#endif // UWB_DRIVER_H
