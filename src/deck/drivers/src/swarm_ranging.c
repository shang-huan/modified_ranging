#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "adhocdeck.h"
#include "swarm_ranging.h"
#include "timers.h"

#ifndef RANGING_DEBUG_ENABLE
  #undef DEBUG_PRINT
  #define DEBUG_PRINT
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static TimerHandle_t rangingTableSetEvictionTimer;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;
static int TfBufferIndex = 0;
static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static SemaphoreHandle_t TfBufferMutex;
static int rangingSeqNumber = 1;
static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;
static Ranging_Table_t EMPTY_RANGING_TABLE = {
    .state = RANGING_STATE_S1,
    .neighborAddress = UWB_DEST_EMPTY,
    .period = RANGING_PERIOD,
    .nextExpectedDeliveryTime = M2T(RANGING_PERIOD),
    .expirationTime = M2T(RANGING_TABLE_HOLD_TIME),
};

int16_t distanceTowards[31] = {[0 ... 30] = -1};

int16_t getDistance(UWB_Address_t neighborAddress) {
//  ASSERT(neighborAddress <= RANGING_TABLE_SIZE_MAX);
  return distanceTowards[neighborAddress];
}

void setDistance(UWB_Address_t neighborAddress, int16_t distance) {
//  ASSERT(neighborAddress <= RANGING_TABLE_SIZE_MAX);
  distanceTowards[neighborAddress] = distance;
}

void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
  rangingTableBuffer->cur = 0;
  rangingTableBuffer->latest = 0;
  Timestamp_Tuple_t empty = {.seqNumber = 0, .timestamp.full = 0};
  for (set_index_t i = 0; i < Tr_Rr_BUFFER_POOL_SIZE; i++) {
    rangingTableBuffer->candidates[i].Tr = empty;
    rangingTableBuffer->candidates[i].Rr = empty;
  }
}

void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                              Timestamp_Tuple_t Tr,
                              Timestamp_Tuple_t Rr) {
  rangingTableBuffer->candidates[rangingTableBuffer->cur].Tr = Tr;
  rangingTableBuffer->candidates[rangingTableBuffer->cur].Rr = Rr;
  // shift
  rangingTableBuffer->latest = rangingTableBuffer->cur;
  rangingTableBuffer->cur = (rangingTableBuffer->cur + 1) % Tr_Rr_BUFFER_POOL_SIZE;
}

Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                                                               Timestamp_Tuple_t Tf) {
  set_index_t index = rangingTableBuffer->latest;
  uint64_t rightBound = Tf.timestamp.full % UWB_MAX_TIMESTAMP;
  Ranging_Table_Tr_Rr_Candidate_t candidate = {.Rr.timestamp.full = 0, .Tr.timestamp.full = 0};

  for (int count = 0; count < Tr_Rr_BUFFER_POOL_SIZE; count++) {
    if (rangingTableBuffer->candidates[index].Rr.timestamp.full &&
        rangingTableBuffer->candidates[index].Rr.timestamp.full % UWB_MAX_TIMESTAMP < rightBound) {
      candidate.Tr = rangingTableBuffer->candidates[index].Tr;
      candidate.Rr = rangingTableBuffer->candidates[index].Rr;
      break;
    }
    index = (index - 1 + Tr_Rr_BUFFER_POOL_SIZE) % Tr_Rr_BUFFER_POOL_SIZE;
  }

  return candidate;
}

void updateTfBuffer(Timestamp_Tuple_t timestamp) {
  xSemaphoreTake(TfBufferMutex, portMAX_DELAY);
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex] = timestamp;
//  DEBUG_PRINT("updateTfBuffer: time = %llu, seq = %d\n", TfBuffer[TfBufferIndex].timestamp.full, TfBuffer[TfBufferIndex].seqNumber);
  xSemaphoreGive(TfBufferMutex);
}

Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber) {
  xSemaphoreTake(TfBufferMutex, portMAX_DELAY);
  Timestamp_Tuple_t Tf = {.timestamp.full = 0, .seqNumber = 0};
  int startIndex = TfBufferIndex;
  /* Backward search */
  for (int i = startIndex; i >= 0; i--) {
    if (TfBuffer[i].seqNumber == seqNumber) {
      Tf = TfBuffer[i];
      break;
    }
  }
  if (!Tf.timestamp.full) {
    /* Forward search */
    for (int i = startIndex + 1; i < Tf_BUFFER_POOL_SIZE; i++) {
      if (TfBuffer[i].seqNumber == seqNumber) {
        Tf = TfBuffer[i];
        break;
      }
    }
  }
  xSemaphoreGive(TfBufferMutex);
  return Tf;
}

Timestamp_Tuple_t getLatestTxTimestamp() {
  return TfBuffer[TfBufferIndex];
}

void getLatestNTxTimestamps(Timestamp_Tuple_t *timestamps, int n) {
  ASSERT(n <= Tf_BUFFER_POOL_SIZE);
  xSemaphoreTake(TfBufferMutex, portMAX_DELAY);
  int startIndex = (TfBufferIndex + 1 - n + Tf_BUFFER_POOL_SIZE) % Tf_BUFFER_POOL_SIZE;
  for (int i = n - 1; i >= 0; i--) {
    timestamps[i] = TfBuffer[startIndex];
    startIndex = (startIndex + 1) % Tf_BUFFER_POOL_SIZE;
  }
  xSemaphoreGive(TfBufferMutex);
}

Ranging_Table_Set_t *getGlobalRangingTableSet() {
  return &rangingTableSet;
}

void rangingTableInit(Ranging_Table_t *table, UWB_Address_t neighborAddress) {
  memset(table, 0, sizeof(Ranging_Table_t));
  table->state = RANGING_STATE_S1;
  table->neighborAddress = neighborAddress;
  table->period = RANGING_PERIOD;
  table->nextExpectedDeliveryTime = xTaskGetTickCount() + table->period;
  table->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
  rangingTableBufferInit(&table->TrRrBuffer); // Can be safely removed this line since memset() is called
}

/* Ranging Table Set Operations */
void rangingTableSetInit(Ranging_Table_Set_t *set) {
  set->mu = xSemaphoreCreateMutex();
  set->size = 0;
  for (int i = 0; i < RANGING_TABLE_SIZE_MAX; i++) {
    set->tables[i] = EMPTY_RANGING_TABLE;
  }
}

static void rangingTableSetSwapTable(Ranging_Table_Set_t *set, int first, int second) {
  Ranging_Table_t temp = set->tables[first];
  set->tables[first] = set->tables[second];
  set->tables[second] = temp;
}

static int rangingTableSetSearchTable(Ranging_Table_Set_t *set, UWB_Address_t targetAddress) {
  /* Binary Search */
  int left = -1, right = set->size, res = -1;
  while (left + 1 != right) {
    int mid = left + (right - left) / 2;
    if (set->tables[mid].neighborAddress == targetAddress) {
      res = mid;
      break;
    } else if (set->tables[mid].neighborAddress > targetAddress) {
      right = mid;
    } else {
      left = mid;
    }
  }
  return res;
}

typedef int (*rangingTableCompareFunc)(Ranging_Table_t *, Ranging_Table_t *);

static int COMPARE_BY_ADDRESS(Ranging_Table_t *first, Ranging_Table_t *second) {
  if (first->neighborAddress == second->neighborAddress) {
    return 0;
  }
  if (first->neighborAddress > second->neighborAddress) {
    return 1;
  }
  return -1;
}

static int COMPARE_BY_EXPIRATION_TIME(Ranging_Table_t *first, Ranging_Table_t *second) {
  if (first->expirationTime == second->expirationTime) {
    return 0;
  }
  if (first->expirationTime > second->expirationTime) {
    return -1;
  }
  return 1;
}

static int COMPARE_BY_NEXT_EXPECTED_DELIVERY_TIME(Ranging_Table_t *first, Ranging_Table_t *second) {
  if (first->nextExpectedDeliveryTime == second->nextExpectedDeliveryTime) {
    return 0;
  }
  if (first->nextExpectedDeliveryTime > second->nextExpectedDeliveryTime) {
    return 1;
  }
  return -1;
}

/* Build the heap */
static void rangingTableSetArrange(Ranging_Table_Set_t *set, int index, int len, rangingTableCompareFunc compare) {
  int leftChild = 2 * index + 1;
  int rightChild = 2 * index + 2;
  int maxIndex = index;
  if (leftChild < len && compare(&set->tables[maxIndex], &set->tables[leftChild]) < 0) {
    maxIndex = leftChild;
  }
  if (rightChild < len && compare(&set->tables[maxIndex], &set->tables[rightChild]) < 0) {
    maxIndex = rightChild;
  }
  if (maxIndex != index) {
    rangingTableSetSwapTable(set, index, maxIndex);
    rangingTableSetArrange(set, maxIndex, len, compare);
  }
}

/* Sort the ranging table */
static void rangingTableSetRearrange(Ranging_Table_Set_t *set, rangingTableCompareFunc compare) {
  /* Build max heap */
  for (int i = set->size / 2 - 1; i >= 0; i--) {
    rangingTableSetArrange(set, i, set->size, compare);
  }
  for (int i = set->size - 1; i >= 0; i--) {
    rangingTableSetSwapTable(set, 0, i);
    rangingTableSetArrange(set, 0, i, compare);
  }
}

static int rangingTableSetClearExpire(Ranging_Table_Set_t *set) {
  Time_t curTime = xTaskGetTickCount();
  int evictionCount = 0;

  for (int i = 0; i < rangingTableSet.size; i++) {
    if (rangingTableSet.tables[i].expirationTime <= curTime) {
      DEBUG_PRINT("rangingTableSetClearExpire: Clean ranging table for neighbor %u that expire at %lu.\n",
                  rangingTableSet.tables[i].neighborAddress,
                  rangingTableSet.tables[i].expirationTime);
      rangingTableSet.tables[i] = EMPTY_RANGING_TABLE;
      evictionCount++;
    }
  }
  /* Keeps ranging table set in order. */
  rangingTableSetRearrange(&rangingTableSet, COMPARE_BY_ADDRESS);
  rangingTableSet.size -= evictionCount;

  return evictionCount;
}

static void rangingTableSetClearExpireTimerCallback(TimerHandle_t timer) {
  xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);

  Time_t curTime = xTaskGetTickCount();
  DEBUG_PRINT("rangingTableClearExpireTimerCallback: Trigger expiration timer at %lu.\n", curTime);

  int evictionCount = rangingTableSetClearExpire(&rangingTableSet);
  if (evictionCount > 0) {
    DEBUG_PRINT("rangingTableSetClearExpireTimerCallback: Evict total %d ranging tables.\n", evictionCount);
  } else {
    DEBUG_PRINT("rangingTableSetClearExpireTimerCallback: Evict none.\n");
  }

  xSemaphoreGive(rangingTableSet.mu);
}

bool rangingTableSetAddTable(Ranging_Table_Set_t *set, Ranging_Table_t table) {
  int index = rangingTableSetSearchTable(set, table.neighborAddress);
  if (index != -1) {
    DEBUG_PRINT(
        "rangingTableSetAddTable: Try to add an already added ranging table for neighbor %u, update it instead.\n",
        table.neighborAddress);
    set->tables[index] = table;
    return true;
  }
  /* If ranging table is full now and there is no expired ranging table, then ignore. */
  if (set->size == RANGING_TABLE_SIZE_MAX && rangingTableSetClearExpire(&rangingTableSet) == 0) {
    DEBUG_PRINT("rangingTableSetAddTable: Ranging table if full, ignore new neighbor %u.\n",
                table.neighborAddress);
    return false;
  }
  /* Add the new entry to the last */
  uint8_t curIndex = set->size;
  set->tables[curIndex] = table;
  set->size++;
  /* Sort the ranging table, keep it in order. */
  rangingTableSetRearrange(set, COMPARE_BY_ADDRESS);
  DEBUG_PRINT("rangingTableSetAddTable: Add new neighbor %u to ranging table.\n", table.neighborAddress);
  return true;
}

void rangingTableSetUpdateTable(Ranging_Table_Set_t *set, Ranging_Table_t table) {
  int index = rangingTableSetSearchTable(set, table.neighborAddress);
  if (index == -1) {
    DEBUG_PRINT("rangingTableSetUpdateTable: Cannot find correspond table for neighbor %u, add it instead.\n",
                table.neighborAddress);
    rangingTableSetAddTable(set, table);
  } else {
    set->tables[index] = table;
    DEBUG_PRINT("rangingTableSetUpdateTable: Update table for neighbor %u.\n", table.neighborAddress);
  }
}

void rangingTableSetRemoveTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress) {
  if (set->size == 0) {
    DEBUG_PRINT("rangingTableSetRemoveTable: Ranging table is empty, ignore.\n");
    return;
  }
  int index = rangingTableSetSearchTable(set, neighborAddress);
  if (index == -1) {
    DEBUG_PRINT("rangingTableSetRemoveTable: Cannot find correspond table for neighbor %u, ignore.\n", neighborAddress);
    return;
  }
  rangingTableSetSwapTable(set, index, set->size - 1);
  set->tables[set->size - 1] = EMPTY_RANGING_TABLE;
  set->size--;
  rangingTableSetRearrange(set, COMPARE_BY_ADDRESS);
}

Ranging_Table_t rangingTableSetFindTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress) {
  int index = rangingTableSetSearchTable(set, neighborAddress);
  Ranging_Table_t table = EMPTY_RANGING_TABLE;
  if (index == -1) {
    DEBUG_PRINT("rangingTableSetFindTable: Cannot find correspond table for neighbor %u.\n", neighborAddress);
  } else {
    table = set->tables[index];
  }
  return table;
}

void printRangingTable(Ranging_Table_t *table) {
  DEBUG_PRINT("Rp = %u, Tr = %u, Rf = %u, \n",
              table->Rp.seqNumber,
              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.seqNumber,
              table->Rf.seqNumber);
  DEBUG_PRINT("Tp = %u, Rr = %u, Tf = %u, Re = %u, \n",
              table->Tp.seqNumber,
              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.seqNumber,
              table->Tf.seqNumber,
              table->Re.seqNumber);
  DEBUG_PRINT("====\n");
}

void printRangingTableSet(Ranging_Table_Set_t *set) {
  DEBUG_PRINT("neighbor\t distance\t period\t expire\t \n");
  for (int i = 0; i < set->size; i++) {
    if (set->tables[i].neighborAddress == UWB_DEST_EMPTY) {
      continue;
    }
    DEBUG_PRINT("%u\t %d\t %lu\t %lu\t \n",
                set->tables[i].neighborAddress,
                set->tables[i].distance,
                set->tables[i].period,
                set->tables[i].expirationTime);
  }
  DEBUG_PRINT("---\n");
}

void printRangingMessage(Ranging_Message_t *rangingMessage) {
  for (int i = 0; i < MAX_Tr_UNIT; i++) {
    DEBUG_PRINT("lastTxTimestamp %d seq=%u, lastTxTimestamp=%2x%8lx\n",
                i,
                rangingMessage->header.lastTxTimestamps[i].seqNumber,
                rangingMessage->header.lastTxTimestamps[i].timestamp.high8,
                rangingMessage->header.lastTxTimestamps[i].timestamp.low32);
  }
  if (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t) == 0) {
    return;
  }
  int body_unit_number = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
  if (body_unit_number >= MAX_BODY_UNIT) {
    DEBUG_PRINT("===printRangingMessage: wrong body unit number occurs===\n");
    return;
  }
  for (int i = 0; i < body_unit_number; i++) {
    DEBUG_PRINT("body_unit_address=%u, body_unit_seq=%u\n",
                rangingMessage->bodyUnits[i].address,
                rangingMessage->bodyUnits[i].timestamp.seqNumber);
    DEBUG_PRINT("body_unit_timestamp=%2x%8lx\n",
                rangingMessage->bodyUnits[i].timestamp.timestamp.high8,
                rangingMessage->bodyUnits[i].timestamp.timestamp.low32);
  }
}


static int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                               Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                               Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {

  bool isErrorOccurred = false;

  if (Tp.seqNumber != Rp.seqNumber || Tr.seqNumber != Rr.seqNumber || Tf.seqNumber != Rf.seqNumber) {
    DEBUG_PRINT("Ranging Error: sequence number mismatch\n");
    isErrorOccurred = true;
  }

  if (Tp.seqNumber >= Tf.seqNumber || Rp.seqNumber >= Rf.seqNumber) {
    DEBUG_PRINT("Ranging Error: sequence number out of order\n");
    isErrorOccurred = true;
  }

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, t;
  tRound1 = (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tReply1 = (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tRound2 = (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tReply2 = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  t = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t distance = (int16_t) t * 0.4691763978616;

  if (distance < 0) {
    DEBUG_PRINT("Ranging Error: distance < 0\n");
    isErrorOccurred = true;
  }

  if (distance > 1000) {
    DEBUG_PRINT("Ranging Error: distance > 1000\n");
    isErrorOccurred = true;
  }

  if (isErrorOccurred) {
    return -1;
  }

  return distance;
}

static void S1_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = RANGING_STATE_S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S1_Tf: S%d -> S%d\n", prevState, curState);
}

static void S1_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = RANGING_STATE_S1;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
//  DEBUG_PRINT("S1_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S1_RX_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = RANGING_STATE_S1;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
//  DEBUG_PRINT("S1_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S2_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = RANGING_STATE_S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S2_Tf: S%d -> S%d\n", prevState, curState);
}

static void S2_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = RANGING_STATE_S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
//  DEBUG_PRINT("S2_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S2_RX_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Find corresponding Tf in TfBuffer, it is possible that can not find corresponding Tf. */
  rangingTable->Tf = findTfBySeqNumber(rangingTable->Rf.seqNumber);
  if (!rangingTable->Tf.timestamp.full) {
    DEBUG_PRINT("Cannot found corresponding Tf in Tf buffer, the ranging frequency may be too high or Tf buffer is in a small size.");
  }

  /* Shift ranging table
   * Rp <- Rf
   * Tp <- Tf  Rr <- Re
   */
  rangingTable->Rp = rangingTable->Rf;
  rangingTable->Tp = rangingTable->Tf;
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;

  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Rf = empty;
  rangingTable->Tf = empty;
  rangingTable->Re = empty;

  rangingTable->state = RANGING_STATE_S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S2_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S3_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = RANGING_STATE_S4;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S3_Tf: S%d -> S%d\n", prevState, curState);
}

static void S3_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Shift ranging table
   * Rr <- Re
   */
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Re = empty;

  rangingTable->state = RANGING_STATE_S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S3_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S3_RX_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Shift ranging table
   * Rr <- Re
   */
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Re = empty;

  rangingTable->state = RANGING_STATE_S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S3_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S4_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = RANGING_STATE_S4;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S4_Tf: S%d -> S%d\n", prevState, curState);
}

static void S4_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Shift ranging table
   * Rr <- Re
   */
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Re = empty;

  rangingTable->state = RANGING_STATE_S4;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S4_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S4_RX_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Find corresponding Tf in TfBuffer, it is possible that can not find corresponding Tf. */
  rangingTable->Tf = findTfBySeqNumber(rangingTable->Rf.seqNumber);

  Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetCandidate(&rangingTable->TrRrBuffer,
                                                                                   rangingTable->Tf);

//  printRangingTable(rangingTable);

  /* try to compute distance */
  int16_t distance = computeDistance(rangingTable->Tp, rangingTable->Rp,
                                     Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                     rangingTable->Tf, rangingTable->Rf);
  if (distance > 0) {
    rangingTable->distance = distance;
    setDistance(rangingTable->neighborAddress, distance);
  } else {
//    DEBUG_PRINT("distance is not updated since some error occurs\n");
  }

  /* Shift ranging table
   * Rp <- Rf
   * Tp <- Tf  Rr <- Re
   */
  rangingTable->Rp = rangingTable->Rf;
  rangingTable->Tp = rangingTable->Tf;
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;

  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Rf = empty;
  rangingTable->Tf = empty;
  rangingTable->Re = empty;

  // TODO: check if valid
  rangingTable->state = RANGING_STATE_S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S4_RX_Rf: S%d -> S%d\n", prevState, curState);
}

/* Don't call this handler function. */
static void S5_Tf(Ranging_Table_t *rangingTable) {
//  DEBUG_PRINT("S5_Tf: invalid handler invocation of temporary state RANGING_STATE_S5\n");
}

/* Don't call this handler function. */
static void S5_RX_NO_Rf(Ranging_Table_t *rangingTable) {
//  DEBUG_PRINT("S5_RX_NO_Rf: invalid handler invocation of temporary state RANGING_STATE_S5\n");
}

/* Don't call this handler function. */
static void S5_RX_Rf(Ranging_Table_t *rangingTable) {
//  DEBUG_PRINT("S5_RX_Rf: invalid handler invocation of temporary state RANGING_STATE_S5\n");
}

/* Don't call this handler function. */
static void RESERVED_STUB(Ranging_Table_t *rangingTable) {
//  DEBUG_PRINT("RESERVED_STUB: Error, been invoked unexpectedly\n");
}

static RangingTableEventHandler EVENT_HANDLER[RANGING_TABLE_STATE_COUNT][RANGING_TABLE_EVENT_COUNT] = {
    {RESERVED_STUB, RESERVED_STUB, RESERVED_STUB},
    {S1_Tf, S1_RX_NO_Rf, S1_RX_Rf},
    {S2_Tf, S2_RX_NO_Rf, S2_RX_Rf},
    {S3_Tf, S3_RX_NO_Rf, S3_RX_Rf},
    {S4_Tf, S4_RX_NO_Rf, S4_RX_Rf},
    /* RANGING_STATE_S5 is effectively a temporary state for distance calculation, never been invoked */
    {S5_Tf, S5_RX_NO_Rf, S5_RX_Rf}
};

void rangingTableOnEvent(Ranging_Table_t *table, RANGING_TABLE_EVENT event) {
  ASSERT(table->state < RANGING_TABLE_STATE_COUNT);
  ASSERT(event < RANGING_TABLE_EVENT_COUNT);
  EVENT_HANDLER[table->state][event](table);
}

/* Swarm Ranging */
static void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  int neighborIndex = rangingTableSetSearchTable(&rangingTableSet, neighborAddress);

  /* Handle new neighbor */
  if (neighborIndex == -1) {
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress);
    /* Ranging table set is full, ignore this ranging message. */
    if (!rangingTableSetAddTable(&rangingTableSet, table)) {
      DEBUG_PRINT("processRangingMessage: Ranging table is full = %d, cannot handle new neighbor %d.\n",
                  rangingTableSet.size,
                  neighborAddress);
      return;
    } else {
      neighborIndex = rangingTableSetSearchTable(&rangingTableSet, neighborAddress);
    }
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.tables[neighborIndex];
  /* Update Re */
  neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
  neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;
  /* Update latest received timestamp of this neighbor */
  neighborRangingTable->latestReceived = neighborRangingTable->Re;
  /* Update expiration time of this neighbor */
  neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

  /* Each ranging messages contains MAX_Tr_UNIT lastTxTimestamps, find corresponding
   * Tr according to Rr to get a valid Tr-Rr pair if possible, this approach may
   * help when experiencing continuous packet loss.
   */
  Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;
  for (int i = 0; i < MAX_Tr_UNIT; i++) {
    if (rangingMessage->header.lastTxTimestamps[i].timestamp.full
        && neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp.full
        && rangingMessage->header.lastTxTimestamps[i].seqNumber
            == neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber) {
      rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                               rangingMessage->header.lastTxTimestamps[i],
                               neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
      break;
    }
  }
//  printRangingMessage(rangingMessage);

  /* Try to find corresponding Rf for MY_UWB_ADDRESS. */
  Timestamp_Tuple_t neighborRf = {.timestamp.full = 0, .seqNumber = 0};
  if (rangingMessage->header.filter & (1 << (uwbGetAddress() % 16))) {
    /* Retrieve body unit from received ranging message. */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++) {
      if (rangingMessage->bodyUnits[i].address == uwbGetAddress()) {
        neighborRf = rangingMessage->bodyUnits[i].timestamp;
        break;
      }
    }
  }

  /* Trigger event handler according to Rf */
  if (neighborRf.timestamp.full) {
    neighborRangingTable->Rf = neighborRf;
    rangingTableOnEvent(neighborRangingTable, RANGING_EVENT_RX_Rf);
  } else {
    rangingTableOnEvent(neighborRangingTable, RANGING_EVENT_RX_NO_Rf);
  }

  #ifdef ENABLE_DYNAMIC_RANGING_PERIOD
  /* update period according to distance and velocity */
  neighborRangingTable->period = M2T(DYNAMIC_RANGING_COEFFICIENT * (neighborRangingTable->distance / rangingMessage->header.velocity));
  /* bound ranging period between RANGING_PERIOD_MIN and RANGING_PERIOD_MAX */
  neighborRangingTable->period = MAX(neighborRangingTable->period, M2T(RANGING_PERIOD_MIN));
  neighborRangingTable->period = MIN(neighborRangingTable->period, M2T(RANGING_PERIOD_MAX));
  #endif
}

static Time_t generateRangingMessage(Ranging_Message_t *rangingMessage) {
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  Time_t curTime = xTaskGetTickCount();
  /* Using the default RANGING_PERIOD when DYNAMIC_RANGING_PERIOD is not enabled. */
  Time_t taskDelay = M2T(RANGING_PERIOD);
  /* Generate message body */
  #ifdef ENABLE_BUS_BOARDING_SCHEME
  rangingTableSetRearrange(&rangingTableSet, COMPARE_BY_NEXT_EXPECTED_DELIVERY_TIME);
  #endif
  for (int index = 0; index < rangingTableSet.size; index++) {
    Ranging_Table_t *table = &rangingTableSet.tables[index];
    if (bodyUnitNumber >= MAX_BODY_UNIT) {
      break;
    }
    if (table->latestReceived.timestamp.full) {
      #ifdef ENABLE_DYNAMIC_RANGING_PERIOD
      /* Only include timestamps with expected delivery time less or equal than current time. */
      if (curTime < table->nextExpectedDeliveryTime) {
        continue;
      }
      table->nextExpectedDeliveryTime = curTime + table->period;
      /* Change task delay dynamically, may increase packet loss rate since ranging period now is determined
       * by the minimum expected delivery time.
       */
      taskDelay = MIN(taskDelay, table->nextExpectedDeliveryTime - curTime);
      /* Bound the dynamic task delay between RANGING_PERIOD_MIN and RANGING_PERIOD */
      taskDelay = MAX(RANGING_PERIOD_MIN, taskDelay);
      #endif
      rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
      /* It is possible that latestReceived is not the newest timestamp, because the newest may be in rxQueue
       * waiting to be handled.
       */
      rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->latestReceived;
      bodyUnitNumber++;
      rangingMessage->header.filter |= 1 << (table->neighborAddress % 16);
      rangingTableOnEvent(table, RANGING_EVENT_TX_Tf);
    }
  }
  #ifdef ENABLE_BUS_BOARDING_SCHEME
  rangingTableSetRearrange(&rangingTableSet, COMPARE_BY_ADDRESS);
  #endif
  /* Generate message header */
  rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
  rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
  rangingMessage->header.msgSequence = curSeqNumber;
  getLatestNTxTimestamps(rangingMessage->header.lastTxTimestamps, MAX_Tr_UNIT);
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  rangingMessage->header.velocity = (short) (velocity * 100);
  //  printRangingMessage(rangingMessage);
  return taskDelay;
}

static void uwbRangingTxTask(void *parameters) {
  systemWaitStart();

  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  UWB_Packet_t txPacketCache;
  txPacketCache.header.srcAddress = uwbGetAddress();
  txPacketCache.header.destAddress = UWB_DEST_ANY;
  txPacketCache.header.type = UWB_RANGING_MESSAGE;
  txPacketCache.header.length = 0;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *) &txPacketCache.payload;

  while (true) {
    xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);

    Time_t taskDelay = generateRangingMessage(rangingMessage);
    txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
    uwbSendPacketBlock(&txPacketCache);
//    printRangingTableSet(&rangingTableSet);

    xSemaphoreGive(rangingTableSet.mu);
    vTaskDelay(taskDelay);
  }
}

static void uwbRangingRxTask(void *parameters) {
  systemWaitStart();

  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true) {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
      xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);

      processRangingMessage(&rxPacketCache);

      xSemaphoreGive(rangingTableSet.mu);
    }
    vTaskDelay(M2T(1));
  }
}

void rangingRxCallback(void *parameters) {
  // DEBUG_PRINT("rangingRxCallback \n");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  UWB_Packet_t *packet = (UWB_Packet_t *) parameters;

  dwTime_t rxTime;
  dwt_readrxtimestamp((uint8_t *) &rxTime.raw);
  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;

  xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingTxCallback(void *parameters) {
  UWB_Packet_t *packet = (UWB_Packet_t *) parameters;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;

  dwTime_t txTime;
  dwt_readtxtimestamp((uint8_t *) &txTime.raw);

  Timestamp_Tuple_t timestamp = {.timestamp = txTime, .seqNumber = rangingMessage->header.msgSequence};
  updateTfBuffer(timestamp);
}

void rangingInit() {
  MY_UWB_ADDRESS = uwbGetAddress();
  rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
  rangingTableSetInit(&rangingTableSet);
  rangingTableSetEvictionTimer = xTimerCreate("rangingTableSetCleanTimer",
                                              M2T(RANGING_TABLE_HOLD_TIME / 2),
                                              pdTRUE,
                                              (void *) 0,
                                              rangingTableSetClearExpireTimerCallback);
  xTimerStart(rangingTableSetEvictionTimer, M2T(0));
  TfBufferMutex = xSemaphoreCreateMutex();

  listener.type = UWB_RANGING_MESSAGE;
  listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
  listener.rxCb = rangingRxCallback;
  listener.txCb = rangingTxCallback;
  uwbRegisterListener(&listener);

  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle);
  xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle);
}

LOG_GROUP_START(Ranging)
        LOG_ADD(LOG_INT16, distTo1, distanceTowards + 1)
        LOG_ADD(LOG_INT16, distTo2, distanceTowards + 2)
        LOG_ADD(LOG_INT16, distTo3, distanceTowards + 3)
        LOG_ADD(LOG_INT16, distTo4, distanceTowards + 4)
        LOG_ADD(LOG_INT16, distTo5, distanceTowards + 5)
        LOG_ADD(LOG_INT16, distTo6, distanceTowards + 6)
        LOG_ADD(LOG_INT16, distTo7, distanceTowards + 7)
        LOG_ADD(LOG_INT16, distTo8, distanceTowards + 8)
        LOG_ADD(LOG_INT16, distTo9, distanceTowards + 9)
        LOG_ADD(LOG_INT16, distTo10, distanceTowards + 10)
LOG_GROUP_STOP(Ranging)
