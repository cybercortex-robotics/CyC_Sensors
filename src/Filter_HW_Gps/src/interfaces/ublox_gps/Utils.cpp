#include <chrono>
#include <time.h>
using namespace std;

#include "Stream.h"
#include "Wire.h"
#include <thread>

// Preinstantiate Objects //////////////////////////////////////////////////////
Stream Serial{};

// Preinstantiate Objects //////////////////////////////////////////////////////
TwoWire Wire = TwoWire();

unsigned long millis() {
  auto now = std::chrono::system_clock::now();
  auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
  auto epoch = now_ms.time_since_epoch();
  auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
  return (value.count());
}

void delay(int sec) {
    int ms = 1000 * sec;
    clock_t start_time = clock();
    while (clock() < start_time + ms); 
} 

void delayMicroseconds(unsigned int us) {
#ifdef WIN32
    std::this_thread::sleep_for(std::chrono::microseconds(us));
#else // NOT WIN32
	usleep(us);
#endif
}

void pinMode(uint8_t pin, uint8_t pinMode) {
	printf ("Functionality yet to be implemented...");
}

void digitalWrite(uint8_t pin, uint8_t pinMode) {
	printf ("Functionality yet to be implemented...");
}