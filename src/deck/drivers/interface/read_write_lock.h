#ifndef __READ_WRITE_LOCK__
#define __READ_WRITE_LOCK__
#include "semphr.h"

typedef struct {
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t writeMutex;
    int readers;
} ReadWriteLock_t;

void CreateReadWriteLock(ReadWriteLock_t *lock);
void TakeReadLock(ReadWriteLock_t *lock,TickType_t xTicksToWait);
void GiveReadLock(ReadWriteLock_t *lock,TickType_t xTicksToWait);
void TakeWriteLock(ReadWriteLock_t *lock,TickType_t xTicksToWait);
void GiveWriteLock(ReadWriteLock_t *lock);
#endif