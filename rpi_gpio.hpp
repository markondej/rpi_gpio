#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#ifndef GPIO_DMA_CHANNEL
#define GPIO_DMA_CHANNEL 0 // 0 - 15
#endif
#ifndef GPIO_SAMPLE_TIME
#define GPIO_SAMPLE_TIME 1 //1 us
#endif
#define GPIO_COUNT 28

#define GPIO0 0
#define GPIO1 1
#define GPIO2 2
#define GPIO3 3
#define GPIO4 4
#define GPIO5 5
#define GPIO6 6
#define GPIO7 7
#define GPIO8 8
#define GPIO9 9
#define GPIO10 10
#define GPIO11 11
#define GPIO12 12
#define GPIO13 13
#define GPIO14 14
#define GPIO15 15
#define GPIO16 16
#define GPIO17 17
#define GPIO18 18
#define GPIO19 19
#define GPIO20 20
#define GPIO21 21
#define GPIO22 22
#define GPIO23 23
#define GPIO24 24
#define GPIO25 25
#define GPIO26 26
#define GPIO27 27

namespace GPIO {
	struct Event {
		unsigned long long time;
		unsigned number;
		bool high;
	};

	enum class Mode { In, Out };

	enum class Resistor { PullUp, PullDown };

	class Controller
	{
		public:
			virtual ~Controller();
			Controller(const Controller &) = delete;
			Controller(Controller &&) = delete;
			Controller &operator=(const Controller &) = delete;
			static Controller &GetInstance();
			void SetMode(unsigned number, Mode mode);
			void SetResistor(unsigned number, Resistor resistor);
			void Set(unsigned number, bool active);
			bool Get(unsigned number);
			std::vector<Event> GetEvents();
			void Reset();
		private:
			Controller();
			static void IOEventThread(Controller *instance);
			std::thread *ioEventThread;
			std::atomic_bool stopped, reset;
			std::vector<Event> events;
			std::mutex copy;
			void *io;
	};
}