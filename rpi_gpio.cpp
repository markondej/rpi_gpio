#include "rpi_gpio.hpp"
#include "mailbox.h"
#include <bcm_host.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <chrono>
#include <algorithm>

#define PERIPHERALS_PHYS_BASE 0x7e000000
#define BCM2835_PERIPHERALS_VIRT_BASE 0x20000000
#define BCM2711_PERIPHERALS_VIRT_BASE 0xfe000000

#define BCM2835_MEM_FLAG 0x0c
#define BCM2711_MEM_FLAG 0x04

#define BCM2835_PLLD_FREQ 500
#define BCM2711_PLLD_FREQ 750

#define GPIO_FSEL_BASE_OFFSET 0x00200000
#define GPIO_SET0_OFFSET 0x0020001c
#define GPIO_CLR0_OFFSET 0x00200028
#define GPIO_LEVEL0_OFFSET 0x00200034
#define GPIO_PUDCTL_OFFSET 0x00200094
#define GPIO_FSEL_INPUT 0x0
#define GPIO_FSEL_OUTPUT 0x1
#define GPIO_PUD_PULL_DOWN 0x0
#define GPIO_PUD_PULL_UP 0x2
#define GPIO_PUD_OFF 0x0

#define CLK_PASSWORD (0x5a << 24)
#define CLK_CTL_SRC_PLLA 0x04
#define CLK_CTL_SRC_PLLC 0x05
#define CLK_CTL_SRC_PLLD 0x06
#define CLK_CTL_ENAB (0x01 << 4)
#define CLK_CTL_MASH(x) ((x & 0x03) << 9)

#define PWMCLK_BASE_OFFSET 0x001010a0
#define PWM_BASE_OFFSET 0x0020c000
#define PWM_CTL_CLRF1 (0x01 << 6)
#define PWM_CTL_USEF1 (0x01 << 5)
#define PWM_CTL_RPTL1 (0x01 << 2)
#define PWM_CTL_MODE1 (0x01 << 1)
#define PWM_CTL_PWEN1 0x01
#define PWM_STA_BERR (0x01 << 8)
#define PWM_STA_GAPO4 (0x01 << 7)
#define PWM_STA_GAPO3 (0x01 << 6)
#define PWM_STA_GAPO2 (0x01 << 5)
#define PWM_STA_GAPO1 (0x01 << 4)
#define PWM_STA_RERR1 (0x01 << 3)
#define PWM_STA_WERR1 (0x01 << 2)
#define PWM_STA_EMPT1 (0x01 << 1)
#define PWM_STA_FULL1 0x01
#define PWM_DMAC_ENAB (0x01 << 31)
#define PWM_DMAC_PANIC(x) ((x & 0x0f) << 8)
#define PWM_DMAC_DREQ(x) (x & 0x0f)

#ifndef PWM_CHANNEL_RANGE
#define PWM_CHANNEL_RANGE 32
#endif
#ifndef PWM_WRITES_PER_SAMPLE
#define PWM_WRITES_PER_SAMPLE 1
#endif

#define DMA0_BASE_OFFSET 0x00007000
#define DMA15_BASE_OFFSET 0x00e05000
#define DMA_CS_RESET (0x01 << 31)
#define DMA_CS_PANIC_PRIORITY(x) ((x & 0x0f) << 20)
#define DMA_CS_PRIORITY(x) ((x & 0x0f) << 16)
#define DMA_CS_INT (0x01 << 2)
#define DMA_CS_END (0x01 << 1)
#define DMA_CS_ACTIVE 0x01
#define DMA_TI_NO_WIDE_BURST (0x01 << 26)
#define DMA_TI_PERMAP(x) ((x & 0x0f) << 16)
#define DMA_TI_DEST_DREQ (0x01 << 6)
#define DMA_TI_WAIT_RESP (0x01 << 3)

#ifndef DMA_CHANNEL
#define DMA_CHANNEL 0 // 0 - 15
#endif
#ifndef DMA_SAMPLE_TIME
#define DMA_SAMPLE_TIME 2 //2 us
#endif
#ifndef DMA_BUFFER_SIZE
#define DMA_BUFFER_SIZE 4096
#endif

#ifndef EVENTS_LIMIT
#define EVENTS_LIMIT 1024
#endif
#define PAGE_SIZE 4096

namespace GPIO {
    struct ClockRegisters {
        uint32_t ctl;
        uint32_t div;
    };

    struct PullUpDownRegisters {
        uint32_t ctl;
        uint32_t clock0;
        uint32_t clock1;
    };

    struct PWMRegisters {
        uint32_t ctl;
        uint32_t status;
        uint32_t dmaConf;
        uint32_t reserved0;
        uint32_t chn1Range;
        uint32_t chn1Data;
        uint32_t fifoIn;
        uint32_t reserved1;
        uint32_t chn2Range;
        uint32_t chn2Data;
    };

    struct DMAControllBlock {
        uint32_t transferInfo;
        uint32_t srcAddress;
        uint32_t dstAddress;
        uint32_t transferLen;
        uint32_t stride;
        uint32_t nextCbAddress;
        uint32_t reserved0;
        uint32_t reserved1;
    };

    struct DMARegisters {
        uint32_t ctlStatus;
        uint32_t cbAddress;
        uint32_t transferInfo;
        uint32_t srcAddress;
        uint32_t dstAddress;
        uint32_t transferLen;
        uint32_t stride;
        uint32_t nextCbAddress;
        uint32_t debug;
    };

    struct IO {
        volatile uint32_t *fnselRegister;
        uint32_t fnselBit;
        Mode mode;
        std::mutex access;
    };

    class Peripherals {
        public:
            virtual ~Peripherals() {
                munmap(peripherals, GetSize());
            }
            Peripherals(const Peripherals &) = delete;
            Peripherals(Peripherals &&) = delete;
            Peripherals &operator=(const Peripherals &) = delete;
            static Peripherals &GetInstance() {
                static Peripherals instance;
                return instance;
            }
            uintptr_t GetPhysicalAddress(volatile void *object) const {
                return PERIPHERALS_PHYS_BASE + (reinterpret_cast<uintptr_t>(object) - reinterpret_cast<uintptr_t>(peripherals));
            }
            uintptr_t GetVirtualAddress(uintptr_t offset) const {
                return reinterpret_cast<uintptr_t>(peripherals) + offset;
            }
            static uintptr_t GetVirtualBaseAddress() {
                return (bcm_host_get_peripheral_size() == BCM2711_PERIPHERALS_VIRT_BASE) ? BCM2711_PERIPHERALS_VIRT_BASE : bcm_host_get_peripheral_address();
            }
            static float GetClockFrequency() {
                return (GetVirtualBaseAddress() == BCM2711_PERIPHERALS_VIRT_BASE) ? BCM2711_PLLD_FREQ : BCM2835_PLLD_FREQ;
            }
        private:
            Peripherals() {
                int memFd;
                if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
                    throw std::runtime_error("Cannot open /dev/mem file (permission denied)");
                }
                peripherals = mmap(nullptr, GetSize(), PROT_READ | PROT_WRITE, MAP_SHARED, memFd, GetVirtualBaseAddress());
                close(memFd);
                if (peripherals == MAP_FAILED) {
                    throw std::runtime_error("Cannot obtain access to peripherals (mmap error)");
                }
            }
            unsigned GetSize() {
                unsigned size = bcm_host_get_peripheral_size();
                if (size == BCM2711_PERIPHERALS_VIRT_BASE) {
                    size = 0x01000000;
                }
                return size;
            }
            void *peripherals;
    };

    class Device {
        public:
            Device() {
                peripherals = &Peripherals::GetInstance();
            }
            Device(const Device &) = delete;
            Device(Device &&) = delete;
            Device &operator=(const Device &) = delete;
        protected:
            Peripherals *peripherals;
    };

    class Clock : public Device {
        public:
            Clock() = delete;
            Clock(uintptr_t address, unsigned divisor) {
                clock = reinterpret_cast<ClockRegisters *>(peripherals->GetVirtualAddress(address));
                clock->ctl = CLK_PASSWORD | CLK_CTL_SRC_PLLD;
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                clock->div = CLK_PASSWORD | (0xffffff & divisor);
                clock->ctl = CLK_PASSWORD | CLK_CTL_MASH(0x1) | CLK_CTL_ENAB | CLK_CTL_SRC_PLLD;
            }
            virtual ~Clock() {
                clock->ctl = CLK_PASSWORD | CLK_CTL_SRC_PLLD;
            }
        protected:
            volatile ClockRegisters *clock;
    };

    class PWMController : public Clock {
        public:
            PWMController() : Clock(PWMCLK_BASE_OFFSET, static_cast<unsigned>(Peripherals::GetClockFrequency() * 1000000.f * (0x01 << 12) / (PWM_WRITES_PER_SAMPLE * PWM_CHANNEL_RANGE * 1000000.f / DMA_SAMPLE_TIME))) {
                pwm = reinterpret_cast<PWMRegisters *>(peripherals->GetVirtualAddress(PWM_BASE_OFFSET));
                pwm->ctl = 0x00000000;
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                pwm->status = PWM_STA_BERR | PWM_STA_GAPO1 | PWM_STA_RERR1 | PWM_STA_WERR1;
                pwm->ctl = PWM_CTL_CLRF1;
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                pwm->chn1Range = PWM_CHANNEL_RANGE;
                pwm->dmaConf = PWM_DMAC_ENAB | PWM_DMAC_PANIC(0x7) | PWM_DMAC_DREQ(0x7);
                pwm->ctl = PWM_CTL_USEF1 | PWM_CTL_RPTL1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
            }
            virtual ~PWMController() {
                pwm->ctl = 0x00000000;
            }
            inline volatile uint32_t &GetFifoIn() {
                return pwm->fifoIn;
            }
        private:
            volatile PWMRegisters *pwm;
    };

    class DMAController : public Device {
        public:
            DMAController() = delete;
            DMAController(uint32_t address) {
                dma = reinterpret_cast<DMARegisters *>(peripherals->GetVirtualAddress((DMA_CHANNEL < 15) ? DMA0_BASE_OFFSET + DMA_CHANNEL * 0x100 : DMA15_BASE_OFFSET));
                dma->ctlStatus = DMA_CS_RESET;
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                dma->ctlStatus = DMA_CS_INT | DMA_CS_END;
                dma->cbAddress = address;
                dma->ctlStatus = DMA_CS_PANIC_PRIORITY(0xf) | DMA_CS_PRIORITY(0xf) | DMA_CS_ACTIVE;
            }
            virtual ~DMAController() {
                dma->ctlStatus = DMA_CS_RESET;
            }
            inline void SetControllBlockAddress(uint32_t address) {
                dma->cbAddress = address;
            }
            inline volatile uint32_t &GetControllBlockAddress() {
                return dma->cbAddress;
            }
        private:
            volatile DMARegisters *dma;
    };

    class AllocatedMemory {
        public:
            AllocatedMemory() = delete;
            AllocatedMemory(unsigned size) {
                mBoxFd = mbox_open();
                memSize = size;
                if (memSize % PAGE_SIZE) {
                    memSize = (memSize / PAGE_SIZE + 1) * PAGE_SIZE;
                }
                memHandle = mem_alloc(mBoxFd, size, PAGE_SIZE, (Peripherals::GetVirtualBaseAddress() == BCM2835_PERIPHERALS_VIRT_BASE) ? BCM2835_MEM_FLAG : BCM2711_MEM_FLAG);
                if (!memHandle) {
                    mbox_close(mBoxFd);
                    memSize = 0;
                    throw std::runtime_error("Cannot allocate memory (" + std::to_string(size) + " bytes)");
                }
                memAddress = mem_lock(mBoxFd, memHandle);
                memAllocated = mapmem(memAddress & ~0xc0000000, memSize);
            }
            virtual ~AllocatedMemory() {
                unmapmem(memAllocated, memSize);
                mem_unlock(mBoxFd, memHandle);
                mem_free(mBoxFd, memHandle);

                mbox_close(mBoxFd);
                memSize = 0;
            }
            AllocatedMemory(const AllocatedMemory &) = delete;
            AllocatedMemory(AllocatedMemory &&) = delete;
            AllocatedMemory &operator=(const AllocatedMemory &) = delete;
            uintptr_t GetPhysicalAddress(volatile void *object) const {
                return (memSize) ? memAddress + (reinterpret_cast<uintptr_t>(object) - reinterpret_cast<uintptr_t>(memAllocated)) : 0x00000000;
            }
            uintptr_t GetBaseAddress() const {
                return reinterpret_cast<uintptr_t>(memAllocated);
            }
        private:
            unsigned memSize, memHandle;
            uintptr_t memAddress;
            void *memAllocated;
            int mBoxFd;
    };

    IO &Select(unsigned number, void *io) {
        if (number >= GPIO_COUNT) {
            throw std::runtime_error("Selected IO line is not supported");
        }
        return reinterpret_cast<IO *>(io)[number];
    }

    Controller::Controller() : scheduleInterval(0), stopped(false), reset(false) {
        Peripherals &peripherals = Peripherals::GetInstance();
        IO *io = new IO[GPIO_COUNT];
        for (unsigned i = 0; i < GPIO_COUNT; i++) {
            io[i].fnselRegister = reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_FSEL_BASE_OFFSET + i * 3 / 30 * sizeof(uint32_t)));
            io[i].fnselBit = (i * 3) % 30;
            io[i].mode = Mode::Out;
        }
        this->io = reinterpret_cast<void *>(io);
        ioEventThread = new std::thread(IOEventThread, this);
    }

    Controller::~Controller() {
        stopped = true;
        ioEventThread->join();
        delete ioEventThread;
        delete [] reinterpret_cast<IO *>(io);
    }

    Controller &Controller::GetInstance() {
        static Controller instance;
        return instance;
    }

    void Controller::SetMode(unsigned number, Mode mode) {
        IO &selected = Select(number, io);
        {
            uint8_t func;
            switch (mode) {
                case Mode::In:
                    func = GPIO_FSEL_INPUT;
                    break;
                default:
                    func = GPIO_FSEL_OUTPUT;
            }
            std::lock_guard<std::mutex> lock(selected.access);
            uint32_t fnsel = *selected.fnselRegister & ((0xfffffff8 << selected.fnselBit) | (0xffffffff >> (32 - selected.fnselBit)));
            *selected.fnselRegister = fnsel | (func << selected.fnselBit);
            selected.mode = mode;
        }
    }

    void Controller::SetResistor(unsigned number, Resistor resistor) {
        IO &selected = Select(number, io);
        Peripherals &peripherals = Peripherals::GetInstance();
        std::lock_guard<std::mutex> lock(selected.access);
        volatile PullUpDownRegisters *pud = reinterpret_cast<PullUpDownRegisters *>(peripherals.GetVirtualAddress(GPIO_PUDCTL_OFFSET));
        switch (resistor) {
            case Resistor::PullDown:
                pud->ctl = GPIO_PUD_PULL_DOWN;
                break;
            case Resistor::PullUp:
                pud->ctl = GPIO_PUD_PULL_UP;
                break;
            default:
                pud->ctl = GPIO_PUD_OFF;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        pud->clock0 = 0x01 << number;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        pud->ctl = 0x00000000;
        pud->clock0 = 0x00000000;
    }

    void Controller::Set(unsigned number, bool high) {
        Select(number, io);
        Peripherals &peripherals = Peripherals::GetInstance();
        volatile uint32_t *reg = high ?
            reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_SET0_OFFSET)) :
            reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_CLR0_OFFSET));
        *reg = 0x01 << number;
    }

    bool Controller::Get(unsigned number) {
        Select(number, io);
        Peripherals &peripherals = Peripherals::GetInstance();
        return (bool)(*reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_LEVEL0_OFFSET)) & (0x01 << number));
    }

    void Controller::Reset() {
        reset = true;
    }

    std::vector<Event> Controller::GetEvents() {
        std::lock_guard<std::mutex> lock(eventsAccess);
        std::vector<Event> events(std::move(this->events));
        return events;
    }

    void Controller::SetSchedule(std::vector<Event> schedule, unsigned long long scheduleInterval) {
        if (scheduleInterval && schedule.empty()) {
            scheduleInterval = 0;
        }
        std::sort(schedule.begin(), schedule.end(), [](const Event &a, const Event &b) -> bool {
            return a.time < b.time;
        });
        std::lock_guard<std::mutex> lock(scheduleAccess);
        this->scheduleInterval = scheduleInterval;
        this->schedule = schedule;
    }

    void Controller::IOEventThread(Controller *instance)
    {
        try {
            Peripherals &peripherals = Peripherals::GetInstance();
            AllocatedMemory allocated(4 * DMA_BUFFER_SIZE * sizeof(DMAControllBlock) + 3 * DMA_BUFFER_SIZE * sizeof(uint32_t) + sizeof(uint32_t));

            PWMController pwm;
            unsigned cbOffset = 0;

            volatile DMAControllBlock *dmaCb = reinterpret_cast<DMAControllBlock *>(allocated.GetBaseAddress());
            volatile uint32_t *levels = reinterpret_cast<uint32_t *>(reinterpret_cast<uintptr_t>(dmaCb) + 4 * DMA_BUFFER_SIZE * sizeof(DMAControllBlock));
            volatile uint32_t *set = reinterpret_cast<uint32_t *>(reinterpret_cast<uintptr_t>(levels) + sizeof(uint32_t) * DMA_BUFFER_SIZE);
            volatile uint32_t *clr = reinterpret_cast<uint32_t *>(reinterpret_cast<uintptr_t>(set) + sizeof(uint32_t) * DMA_BUFFER_SIZE);
            volatile uint32_t *pwmFifoData = reinterpret_cast<uint32_t *>(reinterpret_cast<uintptr_t>(clr) + sizeof(uint32_t) * DMA_BUFFER_SIZE);
            for (unsigned i = 0; i < DMA_BUFFER_SIZE; i++) {
                set[i] = 0x00000000; clr[i] = 0x00000000;

                dmaCb[cbOffset].transferInfo = DMA_TI_NO_WIDE_BURST | DMA_TI_WAIT_RESP;
                dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&clr[i]);
                dmaCb[cbOffset].dstAddress = PERIPHERALS_PHYS_BASE + GPIO_CLR0_OFFSET;
                dmaCb[cbOffset].transferLen = sizeof(uint32_t);
                dmaCb[cbOffset].stride = 0;
                dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
                cbOffset++;

                dmaCb[cbOffset].transferInfo = DMA_TI_NO_WIDE_BURST | DMA_TI_WAIT_RESP;
                dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&set[i]);
                dmaCb[cbOffset].dstAddress = PERIPHERALS_PHYS_BASE + GPIO_SET0_OFFSET;
                dmaCb[cbOffset].transferLen = sizeof(uint32_t);
                dmaCb[cbOffset].stride = 0;
                dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
                cbOffset++;

                dmaCb[cbOffset].transferInfo = DMA_TI_NO_WIDE_BURST | DMA_TI_WAIT_RESP;
                dmaCb[cbOffset].srcAddress = PERIPHERALS_PHYS_BASE + GPIO_LEVEL0_OFFSET;
                dmaCb[cbOffset].dstAddress = allocated.GetPhysicalAddress(&levels[i]);
                dmaCb[cbOffset].transferLen = sizeof(uint32_t);
                dmaCb[cbOffset].stride = 0;
                dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
                cbOffset++;

                dmaCb[cbOffset].transferInfo = DMA_TI_NO_WIDE_BURST | DMA_TI_PERMAP(0x5) | DMA_TI_DEST_DREQ | DMA_TI_WAIT_RESP;
                dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(pwmFifoData);
                dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress(&pwm.GetFifoIn());
                dmaCb[cbOffset].transferLen = sizeof(uint32_t) * PWM_WRITES_PER_SAMPLE;
                dmaCb[cbOffset].stride = 0;
                dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress((i < DMA_BUFFER_SIZE - 1) ? &dmaCb[cbOffset + 1] : dmaCb);
                cbOffset++;
            }
            *pwmFifoData = 0x00000000;

            std::vector<Event> events;
            events.reserve(DMA_BUFFER_SIZE);
            std::vector<Event> schedule, cycle;

            uint32_t previous = *reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_LEVEL0_OFFSET));
            DMAController dma(allocated.GetPhysicalAddress(dmaCb));

            unsigned long long timeOffset = 0, scheduleInterval = 0, cycleEnd = 0;
            auto finally = [&]() {
                dmaCb[(cbOffset < 4 * DMA_BUFFER_SIZE) ? cbOffset : 0].nextCbAddress = 0x00000000;
                while (dma.GetControllBlockAddress() != 0x00000000) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                if (!events.empty()) {
                    std::lock_guard<std::mutex> lock(instance->eventsAccess);
                    instance->events.insert(instance->events.end(), events.begin(), events.end());
                }
                instance->stopped = true;
            };
            auto getScheduled = [&](unsigned long long time) -> std::pair<uint32_t, uint32_t> {
                uint32_t set = 0x00000000, clr = 0x00000000;
                auto event = cycle.begin();
                while (!cycle.empty() && (event->time <= time)) {
                    if (event->high) {
                        set = set | (0x01 << event->number);
                    } else {
                        clr = clr | (0x01 << event->number);
                    }
                    event = cycle.erase(event);
                }
                return { set, clr };
            };

            std::this_thread::sleep_for(std::chrono::microseconds(DMA_BUFFER_SIZE * DMA_SAMPLE_TIME / 10));

            try {
                while (!instance->stopped) {
                    cbOffset = 0;
                    if (instance->reset) {
                        instance->reset = false;
                        for (auto &event : cycle) {
                            event.time -= timeOffset;
                        }
                        cycleEnd -= timeOffset;
                        timeOffset = 0;
                        events.clear();
                        {
                            std::lock_guard<std::mutex> lock(instance->eventsAccess);
                            instance->events.clear();
                        }
                    }
                    {
                        std::lock_guard<std::mutex> lock(instance->scheduleAccess);
                        if (!instance->schedule.empty()) {
                            scheduleInterval = instance->scheduleInterval;
                            schedule = std::move(instance->schedule);
                            instance->scheduleInterval = 0;
                        }
                    }
                    for (unsigned i = 0; i < DMA_BUFFER_SIZE; i++) {
                        while (i == ((dma.GetControllBlockAddress() - allocated.GetPhysicalAddress(dmaCb)) / (4 * sizeof(DMAControllBlock)))) {
                            std::this_thread::sleep_for(std::chrono::microseconds(DMA_BUFFER_SIZE * DMA_SAMPLE_TIME / 10));
                        }
                        unsigned long long time = timeOffset + i * DMA_SAMPLE_TIME;

                        uint32_t current = levels[i], changes = current ^ previous;
                        if (changes) {
                            for (unsigned number = 0; number < GPIO_COUNT; number++) {
                                if (changes & (0x01 << number)) {
                                    events.push_back({ time, number, current & (0x01 << number) });
                                }
                            }
                            previous = current;
                        }

                        std::pair<uint32_t, uint32_t> scheduled = getScheduled(time), initial = { 0x00000000, 0x00000000 };
                        if (cycle.empty() && (cycleEnd <= time) && !schedule.empty()) {
                            cycleEnd = scheduleInterval + time;
                            cycle = schedule;
                            if (!scheduleInterval) {
                                scheduleInterval = 0;
                                schedule.clear();
                            }
                            for (auto &event : cycle) {
                                event.time += time;
                            }
                            initial = getScheduled(time);
                        }
                        set[i] = scheduled.first | initial.first;
                        clr[i] = scheduled.second | initial.second;

                        cbOffset += 4;
                    }
                    timeOffset += DMA_BUFFER_SIZE * DMA_SAMPLE_TIME;
                    if (!events.empty() && !instance->reset) {
                        std::lock_guard<std::mutex> lock(instance->eventsAccess);
                        if (instance->events.size() < EVENTS_LIMIT) {
                            instance->events.insert(instance->events.end(), events.begin(), std::min(events.end(), events.begin() + (EVENTS_LIMIT - instance->events.size())));
                        }
                    }
                    events.clear();
                }
            } catch (...) {
                finally();
                throw;
            }
            finally();
        } catch (...) { }
    }
}
