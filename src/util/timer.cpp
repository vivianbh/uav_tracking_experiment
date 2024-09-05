#include <iostream>
#include <chrono>

class Timer {
public:
 void tic() {
 start_time = std::chrono::high_resolution_clock::now();
    }

 double toc() {
 auto end_time = std::chrono::high_resolution_clock::now();
 auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
 return static_cast<double>(duration.count()) * 1e-6; // 转换为秒
    }

private:
 std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
};