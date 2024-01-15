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

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static int rangingSeqNumber = 1;

static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;

int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};

int16_t getDistance(uint16_t neighborAddress) {
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  return distanceTowards[neighborAddress];
}

void setDistance(uint16_t neighborAddress, int16_t distance) {
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  distanceTowards[neighborAddress] = distance;
}

static void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  set_index_t neighborIndex = findInRangingTableSet(&rangingTableSet, neighborAddress);

  /* Handle new neighbor */
  if (neighborIndex == -1) {
    if (rangingTableSet.freeQueueEntry == -1) {
      /* Ranging table set is full, ignore this ranging message. */
      DEBUG_PRINT("Ranging table is full, cannot handle new neighbor %d\n", neighborAddress);
      return;
    }
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress);
    neighborIndex = rangingTableSetInsert(&rangingTableSet, &table);
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.setData[neighborIndex].data;
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
  if (rangingMessage->header.filter & (1 << (getUWBAddress() % 16))) {
    /* Retrieve body unit from received ranging message. */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++) {
      if (rangingMessage->bodyUnits[i].address == getUWBAddress()) {
        neighborRf = rangingMessage->bodyUnits[i].timestamp;
        break;
      }
    }
  }

  /* Trigger event handler according to Rf */
  if (neighborRf.timestamp.full) {
    neighborRangingTable->Rf = neighborRf;
    rangingTableOnEvent(neighborRangingTable, RX_Rf);
  } else {
    rangingTableOnEvent(neighborRangingTable, RX_NO_Rf);
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
#ifdef ENABLE_BUS_BOARDING_SCHEME
  sortRangingTableSet(&rangingTableSet);
#endif
  rangingTableSetClearExpire(&rangingTableSet);
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  Time_t curTime = xTaskGetTickCount();
  /* Using the default RANGING_PERIOD when DYNAMIC_RANGING_PERIOD is not enabled. */
  Time_t taskDelay = M2T(RANGING_PERIOD);
  /* Generate message body */
  for (set_index_t index = rangingTableSet.fullQueueEntry; index != -1;
       index = rangingTableSet.setData[index].next) {
    Ranging_Table_t *table = &rangingTableSet.setData[index].data;
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
      rangingTableOnEvent(table, TX_Tf);
    }
  }
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
  txPacketCache.header.type = RANGING;
  Ranging_Message_t *rangingMessage = &txPacketCache.payload;
//  txPacketCache.header.mac = ? TODO init mac header
  while (true) {
    int taskDelay = generateRangingMessage(rangingMessage);
    txPacketCache.header.length = sizeof(Packet_Header_t) + rangingMessage->header.msgLength;
    uwbSendPacketBlock(&txPacketCache);
    vTaskDelay(taskDelay);
  }
}

static void uwbRangingRxTask(void *parameters) {
  systemWaitStart();

  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true) {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
//      DEBUG_PRINT("uwbRangingRxTask: received ranging message \n");
      processRangingMessage(&rxPacketCache);
    }
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
  dwTime_t txTime;
  dwt_readtxtimestamp((uint8_t *) &txTime.raw);
  Timestamp_Tuple_t timestamp = {.timestamp = txTime, .seqNumber = rangingSeqNumber};
  updateTfBuffer(timestamp);
}

void rangingInit() {
  MY_UWB_ADDRESS = getUWBAddress();
  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
  rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
  rangingTableSetInit(&rangingTableSet);

  listener.type = RANGING;
  listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
  listener.rxCb = rangingRxCallback;
  listener.txCb = rangingTxCallback;
  uwbRegisterListener(&listener);

  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle); // TODO optimize STACK SIZE
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
  uint64_t rightBound = Tf.timestamp.full % MAX_TIMESTAMP;
  Ranging_Table_Tr_Rr_Candidate_t candidate = {.Rr.timestamp.full = 0, .Tr.timestamp.full = 0};

  for (int count = 0; count < Tr_Rr_BUFFER_POOL_SIZE; count++) {
    if (rangingTableBuffer->candidates[index].Rr.timestamp.full &&
        rangingTableBuffer->candidates[index].Rr.timestamp.full % MAX_TIMESTAMP < rightBound) {
      candidate.Tr = rangingTableBuffer->candidates[index].Tr;
      candidate.Rr = rangingTableBuffer->candidates[index].Rr;
      break;
    }
    index = (index - 1 + Tr_Rr_BUFFER_POOL_SIZE) % Tr_Rr_BUFFER_POOL_SIZE;
  }

  return candidate;
}

static int TfBufferIndex = 0;
static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};

void updateTfBuffer(Timestamp_Tuple_t timestamp) {
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex] = timestamp;
//  DEBUG_PRINT("updateTfBuffer: time = %llu, seq = %d\n", TfBuffer[TfBufferIndex].timestamp.full, TfBuffer[TfBufferIndex].seqNumber);
}

Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber) {
  Timestamp_Tuple_t Tf = {.timestamp.full = 0, .seqNumber = 0};
  int startIndex = TfBufferIndex;
  /* Backward search */
  for (int i = startIndex; i >= 0; i--) {
    if (TfBuffer[i].seqNumber == seqNumber) {
      Tf = TfBuffer[i];
      return Tf;
    }
  }
  /* Forward search */
  for (int i = startIndex + 1; i < Tf_BUFFER_POOL_SIZE; i++) {
    if (TfBuffer[i].seqNumber == seqNumber) {
      Tf = TfBuffer[i];
      break;
    }
  }
  return Tf;
}

Timestamp_Tuple_t getLatestTxTimestamp() {
  return TfBuffer[TfBufferIndex];
}

void getLatestNTxTimestamps(Timestamp_Tuple_t *timestamps, int n) {
  ASSERT(n <= Tf_BUFFER_POOL_SIZE);
  int startIndex = (TfBufferIndex + 1 - n + Tf_BUFFER_POOL_SIZE) % Tf_BUFFER_POOL_SIZE;
  for (int i = n - 1; i >= 0; i--) {
    timestamps[i] = TfBuffer[startIndex];
    startIndex = (startIndex + 1) % Tf_BUFFER_POOL_SIZE;
  }
}

void rangingTableInit(Ranging_Table_t *rangingTable, address_t address) {
  memset(rangingTable, 0, sizeof(Ranging_Table_t));
  rangingTable->state = S1;
  rangingTable->neighborAddress = address;
  rangingTable->period = RANGING_PERIOD;
  rangingTable->nextExpectedDeliveryTime = xTaskGetTickCount() + rangingTable->period;
  rangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
  rangingTableBufferInit(&rangingTable->TrRrBuffer); // Can be safely removed this line since memset() is called
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

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, tprop_ctn;
  tRound1 = (Rr.timestamp.full - Tp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply1 = (Tr.timestamp.full - Rp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tRound2 = (Rf.timestamp.full - Tr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply2 = (Tf.timestamp.full - Rr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  tprop_ctn = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t distance = (int16_t) tprop_ctn * 0.4691763978616;

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
  rangingTable->state = S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S1_Tf: S%d -> S%d\n", prevState, curState);
}

static void S1_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = S1;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
//  DEBUG_PRINT("S1_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S1_RX_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = S1;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
//  DEBUG_PRINT("S1_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S2_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S2_Tf: S%d -> S%d\n", prevState, curState);
}

static void S2_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
//  DEBUG_PRINT("S2_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S2_RX_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Find corresponding Tf in TfBuffer, it is possible that can not find corresponding Tf. */
  rangingTable->Tf = findTfBySeqNumber(rangingTable->Rf.seqNumber);

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

  rangingTable->state = S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S2_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S3_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = S4;

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

  rangingTable->state = S3;

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

  rangingTable->state = S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S3_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S4_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = S4;

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

  rangingTable->state = S4;

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
  rangingTable->state = S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
//  DEBUG_PRINT("S4_RX_Rf: S%d -> S%d\n", prevState, curState);
}

/* Don't call this handler function. */
static void S5_Tf(Ranging_Table_t *rangingTable) {
//  DEBUG_PRINT("S5_Tf: invalid handler invocation of temporary state S5\n");
}

/* Don't call this handler function. */
static void S5_RX_NO_Rf(Ranging_Table_t *rangingTable) {
//  DEBUG_PRINT("S5_RX_NO_Rf: invalid handler invocation of temporary state S5\n");
}

/* Don't call this handler function. */
static void S5_RX_Rf(Ranging_Table_t *rangingTable) {
//  DEBUG_PRINT("S5_RX_Rf: invalid handler invocation of temporary state S5\n");
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
    /* S5 is effectively a temporary state for distance calculation, never been invoked */
    {S5_Tf, S5_RX_NO_Rf, S5_RX_Rf}
};

void rangingTableOnEvent(Ranging_Table_t *rangingTable, RANGING_TABLE_EVENT event) {
  ASSERT(rangingTable->state < RANGING_TABLE_STATE_COUNT);
  ASSERT(event < RANGING_TABLE_EVENT_COUNT);
  EVENT_HANDLER[rangingTable->state][event](rangingTable);
}

static set_index_t rangingTableSetMalloc(
    Ranging_Table_Set_t *rangingTableSet) {
  if (rangingTableSet->freeQueueEntry == -1) {
    DEBUG_PRINT("Ranging Table Set is FULL, malloc failed.\n");
    return -1;
  } else {
    set_index_t candidate = rangingTableSet->freeQueueEntry;
    rangingTableSet->freeQueueEntry =
        rangingTableSet->setData[candidate].next;
    // insert to full queue
    set_index_t temp = rangingTableSet->fullQueueEntry;
    rangingTableSet->fullQueueEntry = candidate;
    rangingTableSet->setData[candidate].next = temp;
    return candidate;
  }
}

static bool rangingTableSetFree(Ranging_Table_Set_t *rangingTableSet,
                                set_index_t item_index) {
  if (-1 == item_index) {
    return true;
  }
  // delete from full queue
  set_index_t pre = rangingTableSet->fullQueueEntry;
  if (item_index == pre) {
    rangingTableSet->fullQueueEntry = rangingTableSet->setData[pre].next;
    // insert into empty queue
    rangingTableSet->setData[item_index].next =
        rangingTableSet->freeQueueEntry;
    rangingTableSet->freeQueueEntry = item_index;
    rangingTableSet->size = rangingTableSet->size - 1;
    return true;
  } else {
    while (pre != -1) {
      if (rangingTableSet->setData[pre].next == item_index) {
        rangingTableSet->setData[pre].next =
            rangingTableSet->setData[item_index].next;
        // insert into empty queue
        rangingTableSet->setData[item_index].next =
            rangingTableSet->freeQueueEntry;
        rangingTableSet->freeQueueEntry = item_index;
        rangingTableSet->size = rangingTableSet->size - 1;
        return true;
      }
      pre = rangingTableSet->setData[pre].next;
    }
  }
  return false;
}

void rangingTableSetInit(Ranging_Table_Set_t *rangingTableSet) {
  set_index_t i;
  for (i = 0; i < RANGING_TABLE_SIZE - 1; i++) {
    rangingTableSet->setData[i].next = i + 1;
  }
  rangingTableSet->setData[i].next = -1;
  rangingTableSet->freeQueueEntry = 0;
  rangingTableSet->fullQueueEntry = -1;
  rangingTableSet->size = 0;
}

set_index_t rangingTableSetInsert(Ranging_Table_Set_t *rangingTableSet,
                                  Ranging_Table_t *table) {
  set_index_t candidate = rangingTableSetMalloc(rangingTableSet);
  if (candidate != -1) {
    memcpy(&rangingTableSet->setData[candidate].data, table,
           sizeof(Ranging_Table_t));
    rangingTableSet->size++;
  }
  return candidate;
}

set_index_t findInRangingTableSet(Ranging_Table_Set_t *rangingTableSet,
                                  address_t addr) {
  set_index_t iter = rangingTableSet->fullQueueEntry;
  while (iter != -1) {
    Ranging_Table_Set_Item_t cur = rangingTableSet->setData[iter];
    if (cur.data.neighborAddress == addr) {
      break;
    }
    iter = cur.next;
  }
  return iter;
}

bool deleteRangingTableByIndex(Ranging_Table_Set_t *rangingTableSet,
                               set_index_t index) {
  return rangingTableSetFree(rangingTableSet, index);
}

bool rangingTableSetClearExpire(Ranging_Table_Set_t *rangingTableSet) {
  set_index_t candidate = rangingTableSet->fullQueueEntry;
  Time_t now = xTaskGetTickCount();
  bool has_changed = false;
  while (candidate != -1) {
    Ranging_Table_Set_Item_t temp = rangingTableSet->setData[candidate];
    if (temp.data.expirationTime < now) {
      set_index_t next_index = temp.next;
      rangingTableSetFree(rangingTableSet, candidate);
      setDistance(temp.data.neighborAddress, -1);
      candidate = next_index;
      has_changed = true;
      continue;
    }
    candidate = temp.next;
  }
  return has_changed;
}

void sortRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
  if (rangingTableSet->fullQueueEntry == -1) {
    return;
  }
  set_index_t new_head = rangingTableSet->fullQueueEntry;
  set_index_t cur = rangingTableSet->setData[new_head].next;
  rangingTableSet->setData[new_head].next = -1;
  set_index_t next = -1;
  while (cur != -1) {
    next = rangingTableSet->setData[cur].next;
    if (rangingTableSet->setData[cur].data.nextExpectedDeliveryTime <=
        rangingTableSet->setData[new_head].data.nextExpectedDeliveryTime) {
      rangingTableSet->setData[cur].next = new_head;
      new_head = cur;
    } else {
      set_index_t start = rangingTableSet->setData[new_head].next;
      set_index_t pre = new_head;
      while (start != -1 &&
          rangingTableSet->setData[cur].data.nextExpectedDeliveryTime >
              rangingTableSet->setData[start].data.nextExpectedDeliveryTime) {
        pre = start;
        start = rangingTableSet->setData[start].next;
      }
      rangingTableSet->setData[cur].next = start;
      rangingTableSet->setData[pre].next = cur;
    }
    cur = next;
  }
  rangingTableSet->fullQueueEntry = new_head;
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
//  DEBUG_PRINT("Rp = %2x%8lx, Tr = %2x%8lx, Rf = %2x%8lx, \n",
//              table->Rp.timestamp.high8,
//              table->Rp.timestamp.low32,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.timestamp.high8,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.timestamp.low32,
//              table->Rf.timestamp.high8,
//              table->Rf.timestamp.low32);
//  DEBUG_PRINT("Tp = %2x%8lx, Rr = %2x%8lx, Tf = %2x%8lx, Re = %2x%8lx, \n",
//              table->Tp.timestamp.high8,
//              table->Tp.timestamp.low32,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.timestamp.high8,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.timestamp.low32,
//              table->Tf.timestamp.high8,
//              table->Tf.timestamp.low32,
//              table->Re.timestamp.high8,
//              table->Re.timestamp.low32);
//  DEBUG_PRINT("====\n");
//  DEBUG_PRINT("Rp = %llu, Tr = %llu, Rf = %llu, \n",
//              table->Rp.timestamp.full,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.seqNumber,
//              table->Rf.timestamp.full);
//  DEBUG_PRINT("Tp = %llu, Rr = %llu, Tf = %llu, Re = %llu, \n",
//              table->Tp.timestamp.full,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.seqNumber,
//              table->Tf.timestamp.full,
//              table->Re.timestamp.full);
//  DEBUG_PRINT("====\n");
}

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
  for (set_index_t index = rangingTableSet->fullQueueEntry; index != -1;
       index = rangingTableSet->setData[index].next) {
    printRangingTable(&rangingTableSet->setData[index].data);
  }
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
