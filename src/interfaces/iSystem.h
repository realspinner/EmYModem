#ifndef __ISYSTEM_H__
#define __ISYSTEM_H__


#include <cstdint>


class iSystem {
public:
    virtual ~iSystem() = default;

    // Time/sleep functions
    virtual uint32_t getMillis() = 0;
    virtual void sleepMillis(uint32_t millis) = 0;

    // Synchronization operations
    virtual uint32_t infiniteTime() = 0;

    virtual int getMutexHandle() = 0;
    virtual void freeMutexHandle(int mutexHandle) = 0;
    virtual bool lockMutex(int mutexHandle, uint32_t millis) = 0;
    virtual void unlockMutex(int mutexHandle) = 0;

    virtual int  getSemaphoreHandle() = 0;
    virtual void freeSemaphoreHandle(int semHandle) = 0;
    virtual bool waitSemaphore(int semHandle, uint32_t millis) = 0;
    virtual void giveSemaphore(int semHandle) = 0;
};


#endif /* __ISYSTEM_H__ */
