#include "FreeRTOS.h"
#include "semphr.h"
#include "read_write_lock.h"

// 创建模拟读写锁
void CreateReadWriteLock(ReadWriteLock_t *lock) {
    lock->mutex = xSemaphoreCreateMutex();
    lock->writeMutex = xSemaphoreCreateMutex();
    lock->readers = 0;
}

// 获取读锁
void TakeReadLock(ReadWriteLock_t *lock,TickType_t xTicksToWait) {
    xSemaphoreTake(lock->mutex, xTicksToWait);
    if(++lock->readers == 1) {
        xSemaphoreTake(lock->writeMutex, xTicksToWait);
    }
    xSemaphoreGive(lock->mutex);
}

// 释放读锁
void GiveReadLock(ReadWriteLock_t *lock,TickType_t xTicksToWait) {
    xSemaphoreTake(lock->mutex, xTicksToWait);
    if(--lock->readers == 0) {
        xSemaphoreGive(lock->writeMutex);
    }
    xSemaphoreGive(lock->mutex);
}

// 获取写锁
void TakeWriteLock(ReadWriteLock_t *lock,TickType_t xTicksToWait) {
    xSemaphoreTake(lock->writeMutex, xTicksToWait);
}

// 释放写锁
void GiveWriteLock(ReadWriteLock_t *lock){
    xSemaphoreGive(lock->writeMutex);
}