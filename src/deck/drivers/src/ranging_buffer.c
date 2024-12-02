#include "FreeRTOS.h"
#include "debug.h"
#include "dwTypes.h"

#include "ranging_buffer.h"
#include "ranging_table.h"
#include "nullValue.h"
#include "adhocdeck.h"


void initRangingBuffer(RangingBuffer *buffer){
    buffer->topSendBuffer = NULL_INDEX;
    buffer->topReceiveBuffer = NULL_INDEX;
    buffer->receiveLength = 0;
    buffer->sendLength = 0;
    for (int i = 0; i < MAX_RANGING_BUFFER_SIZE; i++)
    {
        initRangingBufferNode(&buffer->sendBuffer[i]);
        initRangingBufferNode(&buffer->receiveBuffer[i]);
    }
}
void initRangingBufferNode(RangingBufferNode *node){
    node->sendTx = nullTimeStamp;
    node->sendRx = nullTimeStamp;
    node->receiveTx = nullTimeStamp;
    node->receiveRx = nullTimeStamp;
    node->T1 = NULL_TF;
    node->T2 = NULL_TF;
    node->sumTof = NULL_TF;
    node->localSeq = NULL_SEQ;
    node->preLocalSeq = NULL_SEQ;
    
    #ifdef ENABLE_RECORD_COORDINATE
        node->sendTxCoordinate = nullCoordinate;
        node->sendRxCoordinate = nullCoordinate;
        node->receiveTxCoordinate = nullCoordinate;
        node->receiveRxCoordinate = nullCoordinate;
    #endif

}
void addRangingBuffer(RangingBuffer *buffer, RangingBufferNode *node, StatusType status){
    if (status == SENDER)
    {
        buffer->topSendBuffer = (buffer->topSendBuffer + 1) % MAX_RANGING_BUFFER_SIZE;
        buffer->sendBuffer[buffer->topSendBuffer].sendTx = node->sendTx;
        buffer->sendBuffer[buffer->topSendBuffer].sendRx = node->sendRx;
        buffer->sendBuffer[buffer->topSendBuffer].receiveTx = node->receiveTx;
        buffer->sendBuffer[buffer->topSendBuffer].receiveRx = node->receiveRx;
        #ifdef ENABLE_RECORD_COORDINATE
            buffer.sendBuffer[buffer->topSendBuffer].sendTxCoordinate = node->sendTxCoordinate;
            buffer->sendBuffer[buffer->topSendBuffer].sendRxCoordinate = node->sendRxCoordinate;
            buffer->sendBuffer[buffer->topSendBuffer].receiveTxCoordinate = node->receiveTxCoordinate;
            buffer->sendBuffer[buffer->topSendBuffer].receiveRxCoordinate = node->receiveRxCoordinate;
        #endif
        buffer->sendBuffer[buffer->topSendBuffer].localSeq = node->localSeq;
        buffer->sendBuffer[buffer->topSendBuffer].preLocalSeq = node->preLocalSeq;
        buffer->sendBuffer[buffer->topSendBuffer].sumTof = node->sumTof;
        buffer->sendBuffer[buffer->topSendBuffer].T1 = node->T1;
        buffer->sendBuffer[buffer->topSendBuffer].T2 = node->T2;
        DEBUG_PRINT("[SENDER]RTX:%llu,RRX:%llu,TTX:%llu,TRX:%llu,T1:%lld,T2:%lld,localSeq:%d,preLocalSeq:%d\n"
            ,node->receiveTx.full,node->receiveRx.full,node->sendTx.full,node->sendRx.full
            ,node->T1,node->T2,node->localSeq,node->preLocalSeq);
        if(buffer->sendLength < MAX_RANGING_BUFFER_SIZE){
            buffer->sendLength++;
        }
        //打印所有数据
        // table_index_t i = buffer->topSendBuffer;
        // for (int j = 0; j < buffer->sendLength; j++)
        // {
        //     DEBUG_PRINT("[SENDER]RTX:%llu,RRX:%llu,TTX:%llu,TRX:%llu,T1:%lld,T2:%lld,localSeq:%d,preLocalSeq:%d\n"
        //         ,buffer->sendBuffer[i].receiveTx.full,buffer->sendBuffer[i].receiveRx.full,buffer->sendBuffer[i].sendTx.full,buffer->sendBuffer[i].sendRx.full
        //         ,buffer->sendBuffer[i].T1,buffer->sendBuffer[i].T2,buffer->sendBuffer[i].localSeq,buffer->sendBuffer[i].preLocalSeq);
        //     i = (i - 1 + MAX_RANGING_BUFFER_SIZE) % MAX_RANGING_BUFFER_SIZE;
        // }
    }
    else if (status == RECEIVER)
    {
        buffer->topReceiveBuffer = (buffer->topReceiveBuffer + 1) % MAX_RANGING_BUFFER_SIZE;
        buffer->receiveBuffer[buffer->topReceiveBuffer].sendTx = node->sendTx;
        buffer->receiveBuffer[buffer->topReceiveBuffer].sendRx = node->sendRx;
        buffer->receiveBuffer[buffer->topReceiveBuffer].receiveTx = node->receiveTx;
        buffer->receiveBuffer[buffer->topReceiveBuffer].receiveRx = node->receiveRx;
        #ifdef ENABLE_RECORD_COORDINATE
            buffer.receiveBuffer[buffer->topReceiveBuffer].sendTxCoordinate = node->sendTxCoordinate;
            buffer->receiveBuffer[buffer->topReceiveBuffer].sendRxCoordinate = node->sendRxCoordinate;
            buffer->receiveBuffer[buffer->topReceiveBuffer].receiveTxCoordinate = node->receiveTxCoordinate;
            buffer->receiveBuffer[buffer->topReceiveBuffer].receiveRxCoordinate = node->receiveRxCoordinate;
        #endif
        buffer->receiveBuffer[buffer->topReceiveBuffer].localSeq = node->localSeq;
        buffer->receiveBuffer[buffer->topReceiveBuffer].preLocalSeq = node->preLocalSeq;
        buffer->receiveBuffer[buffer->topReceiveBuffer].sumTof = node->sumTof;
        buffer->receiveBuffer[buffer->topReceiveBuffer].T1 = node->T1;
        buffer->receiveBuffer[buffer->topReceiveBuffer].T2 = node->T2;
        DEBUG_PRINT("[RECEIVER]TTX:%llu,TRX:%llu,RTX:%llu,RRX:%llu,T1:%lld,T2:%lld,localSeq:%d,preLocalSeq:%d\n"
            ,node->sendTx.full,node->sendRx.full,node->receiveTx.full,node->receiveRx.full
            ,node->T1,node->T2,node->localSeq,node->preLocalSeq);
        if(buffer->receiveLength < MAX_RANGING_BUFFER_SIZE){
            buffer->receiveLength++;
        }
        //打印所有数据
        // table_index_t i = buffer->topReceiveBuffer;
        // for (int j = 0; j < buffer->receiveLength; j++)
        // {
        //     DEBUG_PRINT("[RECEIVER]TTX:%llu,TRX:%llu,RTX:%llu,RRX:%llu,T1:%lld,T2:%lld,localSeq:%d,preLocalSeq:%d\n"
        //         ,buffer->receiveBuffer[i].sendTx.full,buffer->receiveBuffer[i].sendRx.full,buffer->receiveBuffer[i].receiveTx.full,buffer->receiveBuffer[i].receiveRx.full
        //         ,buffer->receiveBuffer[i].T1,buffer->receiveBuffer[i].T2,buffer->receiveBuffer[i].localSeq,buffer->receiveBuffer[i].preLocalSeq);
        //     i = (i - 1 + MAX_RANGING_BUFFER_SIZE) % MAX_RANGING_BUFFER_SIZE;
        // }
    }
    DEBUG_PRINT("Add a new record to ranging buffer,current topSendBuffer:%d,topReceiveBuffer:%d.\n",buffer->topSendBuffer,buffer->topReceiveBuffer);
}
table_index_t searchRangingBuffer(RangingBuffer *buffer, uint16_t localSeq, StatusType status){
    table_index_t ans = NULL_INDEX;
    if(status == SENDER){
        table_index_t i = buffer->topReceiveBuffer;
        for (int j = 0; j < buffer->receiveLength; j++)
        {
            if(buffer->receiveBuffer[i].localSeq < localSeq){
                if(ans == NULL_INDEX || buffer->receiveBuffer[i].localSeq > buffer->receiveBuffer[ans].localSeq){
                    ans = i;
                }
            }
            i = (i - 1 + MAX_RANGING_BUFFER_SIZE) % MAX_RANGING_BUFFER_SIZE;
        }
    }
    else if(status == RECEIVER){
        table_index_t i = buffer->topSendBuffer;
        for (int j = 0; j < buffer->sendLength; j++)
        {
            if(buffer->sendBuffer[i].localSeq < localSeq){
                if(ans == NULL_INDEX || buffer->sendBuffer[i].localSeq > buffer->sendBuffer[ans].localSeq){
                    ans = i;
                }
            }
            i = (i - 1 + MAX_RANGING_BUFFER_SIZE) % MAX_RANGING_BUFFER_SIZE;
        }
    }
    return ans;
}
bool calculateTof(RangingBuffer *buffer, dwTime_t Tx, dwTime_t Rx, uint16_t localSeq, uint16_t checkLocalSeq, StatusType status, bool flag){
    RangingBufferNode* node = NULL;
    table_index_t index = searchRangingBuffer(buffer, checkLocalSeq, status);
    if(index == NULL_INDEX){
        DEBUG_PRINT("Warning: Cannot find the record with localSeq:%d.\n",checkLocalSeq);
        return false;
    }
    if(status == SENDER){
        node = &buffer->receiveBuffer[index];
    }
    else if(status == RECEIVER){
        node = &buffer->sendBuffer[index];
    }
    DEBUG_PRINT("[CalculateTof]nodePreLocalSeq:%d,nodelocalSeq:%d,localSeq:%d\n",node->preLocalSeq,node->localSeq,localSeq);
    uint64_t Ra=0,Rb=0,Da=0,Db=0;
    /*
        本次通信是发送方,通信次序从左到右如下:
            TRX RTX         RX
        TTX         RRX TX
    */
    if(status == SENDER){
        Ra = (node->receiveRx.full-node->sendTx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        Db = (node->receiveTx.full-node->sendRx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        Rb = (Rx.full - node->receiveTx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        Da = (Tx.full - node->receiveRx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    }
    /*
        本次通信是接收方,通信次序从左到右如下:
            RTX         TRX TX
                RRX TTX         RX
    */
    else if(status == RECEIVER){
        Ra = (node->sendRx.full - node->receiveTx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        Db = (node->sendTx.full - node->receiveRx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        Rb = (Rx.full - node->sendTx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        Da = (Tx.full - node->sendRx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    }
    int64_t T12 = node->sumTof,T23 = 0;
    // DEBUG_PRINT("[CalculateTof]status: %d,T12: %lld,Ra: %llu,Rb: %llu,Da: %llu,Db: %llu\n", status,T12,Ra, Rb, Da, Db);
    float RaDa = (float)Ra/Da;
    float RbDb = (float)Rb/Db;
    int64_t diffA = Ra - Da;
    int64_t diffB = Rb - Db;

    DEBUG_PRINT("[CalculateTof]RaDa: %f,RbDb: %f\n", RaDa, RbDb);
    if(RaDa < CONVERGENCE_THRESHOLD || RbDb < CONVERGENCE_THRESHOLD){
        if(RaDa < RbDb){
            T23 = ((diffA * Rb + diffA * Db + diffB * Ra + diffB * Da)/2 - Ra*T12)/Da;
        }else{
            T23 = ((diffA * Rb + diffA * Db + diffB * Ra + diffB * Da)/2 - Rb*T12)/Db;
        }
    }else{
        DEBUG_PRINT("Warning: Ra/Da and Rb/Db are both greater than CONVERGENCE_THRESHOLD(%f).\n",CONVERGENCE_THRESHOLD);
        #ifdef ENABLE_CLASSIC_TOLERANCE
            T23 = ((Rb + Db) * (Ra - Da) + (Rb - Db) * (Ra + Da)) / (Ra + Db + Rb + Da);
            DEBUG_PRINT("Warning: use traditional method to calculate T23.\n");
        #else
            if(flag){
                DEBUG_PRINT("Warning: The latest record in the ranging buffer fails, and an attempt is made to recalculate the Tof using the next most recent valid record.\n");
                return calculateTof(buffer, Tx, Rx, localSeq, node->localSeq, status, false);
            }
            else{
                DEBUG_PRINT("Warning: The latest record in the ranging buffer failed, and an attempt to recalculate the Tof using the next most recent valid record failed.\n");
                return false;
            }
        #endif
    }
    float d = T23 * 0.4691763978616 / 2;
    if(d < 0 || d > 1000){
        DEBUG_PRINT("Warning: d is out of range(0,1000),d:%f\n",d);
        if(flag){
            DEBUG_PRINT("Warning: The latest record in the ranging buffer fails, and an attempt is made to recalculate the Tof using the next most recent valid record.\n");
            return calculateTof(buffer, Tx, Rx, localSeq, node->localSeq, status, false);
        }
        else{
            DEBUG_PRINT("Warning: The latest record in the ranging buffer failed, and an attempt to recalculate the Tof using the next most recent valid record failed.\n");
            return false;
        }
    }
    int64_t classicTof = ((Rb + Db) * (Ra - Da) + (Rb - Db) * (Ra + Da)) / (2*(Ra + Db + Rb + Da));
    float classicD = classicTof * 0.4691763978616;
    DEBUG_PRINT("[CalculateTof Finished]T23: %lld,T3: %lld,classicTof: %lld,d: %f,d3: %f,classicD: %f\n", T23, (T23-node->T2), classicTof, d, (T23-node->T2)* 0.4691763978616, classicD);
    //缓存新有效记录
    RangingBufferNode newNode;
    if(status == SENDER){
        /*
            本次通信是发送方,通信次序从左到右如下:
                TRX RTX         RX
            TTX         RRX TX
        */
        newNode.sendTx = Tx;
        newNode.sendRx = Rx;
        newNode.receiveTx = node->receiveTx;
        newNode.receiveRx = node->receiveRx;
        #ifdef ENABLE_RECORD_COORDINATE
            newNode.receiveTxCoordinate = node->receiveTxCoordinate;
            newNode.receiveRxCoordinate = node->receiveRxCoordinate;
        #endif
        newNode.localSeq = localSeq;
        newNode.preLocalSeq = node->localSeq;
        newNode.sumTof = T23;
        newNode.T1 = node->T2;
        newNode.T2 = T23 - newNode.T1;
        addRangingBuffer(buffer, &newNode, status);
    }
    else if(status == RECEIVER){
        /*
            本次通信是接收方,通信次序从左到右如下:
                RTX         TRX TX
                    RRX TTX         RX
        */
        newNode.sendTx = node->sendTx;
        newNode.sendRx = node->sendRx;
        newNode.receiveTx = Tx;
        newNode.receiveRx = Rx;
        #ifdef ENABLE_RECORD_COORDINATE
            newNode.sendTxCoordinate = node->sendTxCoordinate;
            newNode.sendRxCoordinate = node->sendRxCoordinate;
        #endif
        newNode.localSeq = localSeq;
        newNode.preLocalSeq = node->localSeq;
        newNode.sumTof = T23;
        newNode.T1 = node->T2;
        newNode.T2 = T23 - newNode.T1;
        addRangingBuffer(buffer, &newNode, status);
    }
    return true;
}

// 更新Tof记录
bool firstRecordBuffer(TableLinkedList_t *listA, TableLinkedList_t *listB, table_index_t firstIndex, RangingBuffer* rangingBuffer, StatusType status)
{
    // 依次从A、B、A中取出数据
    table_index_t indexA1 = firstIndex;
    // DEBUG_PRINT("indexA1:%d,Tx:%llu,Rx:%llu,Tf:%lld\n",indexA1,listA->tableBuffer[indexA1].Tx.full,listA->tableBuffer[indexA1].Rx.full,listA->tableBuffer[indexA1].Tf);
    if (indexA1 == NULL_INDEX || listA->tableBuffer[indexA1].Tx.full == NULL_TIMESTAMP || listA->tableBuffer[indexA1].Rx.full == NULL_TIMESTAMP || listA->tableBuffer[indexA1].Tf != NULL_TF)
    {
        DEBUG_PRINT("the lastest record in listA is invalid or the record has owned Tof\n");
        return false;
    }
    table_index_t indexB2 = findMaxSeqIndex(listB, listA->tableBuffer[indexA1].localSeq, listA->tableBuffer[indexA1].remoteSeq);
    if (indexB2 == NULL_INDEX)
    {
        DEBUG_PRINT("No valid record in listB\n");
        return false;
    }
    table_index_t indexA3 = findMaxSeqIndex(listA, listB->tableBuffer[indexB2].localSeq, listB->tableBuffer[indexB2].remoteSeq);
    if (indexA3 == NULL_INDEX)
    {
        DEBUG_PRINT("No valid record in listA\n");
        return false;
    }
    // 计算Tof
    /*
    A3.Tx          <- Ra ->          B2.Rx      <- Da ->      A1.Tx

            A3.Rx  <- Db ->  B2.Tx              <- Rb ->              A1.Rx
    */
    // DEBUG_PRINT("indexA1: %d,seq: %d,tx: %llu,rx:%llu\n", indexA1, listA->tableBuffer[indexA1].localSeq, listA->tableBuffer[indexA1].Tx.full, listA->tableBuffer[indexA1].Rx.full);
    // DEBUG_PRINT("indexB2: %d,seq: %d,tx: %llu,rx:%llu\n", indexB2, listB->tableBuffer[indexB2].localSeq, listB->tableBuffer[indexB2].Tx.full, listB->tableBuffer[indexB2].Rx.full);
    // DEBUG_PRINT("indexA3: %d,seq: %d,tx: %llu,rx:%llu\n", indexA3, listA->tableBuffer[indexA3].localSeq, listA->tableBuffer[indexA3].Tx.full, listA->tableBuffer[indexA3].Rx.full);
    int64_t Ra = (listB->tableBuffer[indexB2].Rx.full - listA->tableBuffer[indexA3].Tx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db = (listB->tableBuffer[indexB2].Tx.full - listA->tableBuffer[indexA3].Rx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb = (listA->tableBuffer[indexA1].Rx.full - listB->tableBuffer[indexB2].Tx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da = (listA->tableBuffer[indexA1].Tx.full - listB->tableBuffer[indexB2].Rx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    int64_t diffA = Ra - Da;
    int64_t diffB = Rb - Db;
    int64_t sumAB = Ra + Db + Rb + Da;
    // int64_t Tof = ((Rb + Db) * (Ra - Da) + (Rb - Db) * (Ra + Da)) / (2*);
    int64_t Tof = (diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / (2*sumAB);
    if(Tof < 0){
        DEBUG_PRINT("Tof is less than 0\n");
        return false;
    }
    RangingBufferNode newNode1,newNode2;
    if(status == SENDER){
        /*
                A3.Rx  B2.Tx            A1.Rx
            A3.Tx           B2.Rx  A1.Tx
        */
        newNode1.sendTx = listA->tableBuffer[indexA3].Tx;
        newNode1.sendRx = listA->tableBuffer[indexA3].Rx;
        newNode1.receiveTx = listB->tableBuffer[indexB2].Tx;
        newNode1.receiveRx = listB->tableBuffer[indexB2].Rx;
        newNode1.localSeq = listB->tableBuffer[indexB2].localSeq;
        newNode1.preLocalSeq = listA->tableBuffer[indexA3].localSeq;
        newNode1.sumTof = Tof*2;
        newNode1.T1 = Tof;
        newNode1.T2 = Tof;
        #ifdef ENABLE_RECORD_COORDINATE
            newNode1.sendTxCoordinate = listA->tableBuffer[indexA3].TxCoordinate;
            newNode1.sendRxCoordinate = listA->tableBuffer[indexA3].RxCoordinate;
            newNode1.receiveTxCoordinate = listB->tableBuffer[indexB2].TxCoordinate;
            newNode1.receiveRxCoordinate = listB->tableBuffer[indexB2].RxCoordinate;
        #endif
        DEBUG_PRINT("Tof:%lld, T1:%lld, T2:%lld\n",Tof,newNode1.T1,newNode1.T2);
        addRangingBuffer(rangingBuffer, &newNode1, RECEIVER);

        newNode2.sendTx = listA->tableBuffer[indexA1].Tx;
        newNode2.sendRx = listA->tableBuffer[indexA1].Rx;
        newNode2.receiveTx = listB->tableBuffer[indexB2].Tx;
        newNode2.receiveRx = listB->tableBuffer[indexB2].Rx;
        newNode2.localSeq = listA->tableBuffer[indexA1].localSeq;
        newNode2.preLocalSeq = listB->tableBuffer[indexB2].localSeq;
        newNode2.sumTof = Tof*2;
        newNode2.T1 = Tof;
        newNode2.T2 = Tof;
        #ifdef ENABLE_RECORD_COORDINATE
            newNode2.sendTxCoordinate = listA->tableBuffer[indexA1].TxCoordinate;
            newNode2.sendRxCoordinate = listA->tableBuffer[indexA1].RxCoordinate;
            newNode2.receiveTxCoordinate = listB->tableBuffer[indexB2].TxCoordinate;
            newNode2.receiveRxCoordinate = listB->tableBuffer[indexB2].RxCoordinate;
        #endif
        addRangingBuffer(rangingBuffer, &newNode2, SENDER);
    }
    else if(status == RECEIVER){
        /*
            A3.Tx           B2.Rx   A1.Tx
                A3.Rx  B2.Tx            A1.Rx
        */
        newNode1.receiveTx = listA->tableBuffer[indexA3].Tx;
        newNode1.receiveRx = listA->tableBuffer[indexA3].Rx;
        newNode1.sendTx = listB->tableBuffer[indexB2].Tx;
        newNode1.sendRx = listB->tableBuffer[indexB2].Rx;
        newNode1.localSeq = listB->tableBuffer[indexB2].localSeq;
        newNode1.preLocalSeq = listA->tableBuffer[indexA3].localSeq;
        newNode1.sumTof = Tof*2;
        newNode1.T1 = Tof;
        newNode1.T2 = Tof;
        #ifdef ENABLE_RECORD_COORDINATE
            newNode1.sendTxCoordinate = listB->tableBuffer[indexB2].TxCoordinate;
            newNode1.sendRxCoordinate = listB->tableBuffer[indexB2].RxCoordinate;
            newNode1.receiveTxCoordinate = listA->tableBuffer[indexA3].TxCoordinate;
            newNode1.receiveRxCoordinate = listA->tableBuffer[indexA3].RxCoordinate;
        #endif
        
        addRangingBuffer(rangingBuffer, &newNode1, SENDER);

        newNode2.receiveTx = listA->tableBuffer[indexA1].Tx;
        newNode2.receiveRx = listA->tableBuffer[indexA1].Rx;
        newNode2.sendTx = listB->tableBuffer[indexB2].Tx;
        newNode2.sendRx = listB->tableBuffer[indexB2].Rx;
        newNode2.localSeq = listA->tableBuffer[indexA1].localSeq;
        newNode2.preLocalSeq = listB->tableBuffer[indexB2].localSeq;
        newNode2.sumTof = Tof*2;
        newNode2.T1 = Tof;
        newNode2.T2 = Tof;
        #ifdef ENABLE_RECORD_COORDINATE
            newNode2.sendTxCoordinate = listB->tableBuffer[indexB2].TxCoordinate;
            newNode2.sendRxCoordinate = listB->tableBuffer[indexB2].RxCoordinate;
            newNode2.receiveTxCoordinate = listA->tableBuffer[indexA1].TxCoordinate;
            newNode2.receiveRxCoordinate = listA->tableBuffer[indexA1].RxCoordinate;
        #endif
        DEBUG_PRINT("Tof:%lld, T1:%lld, T2:%lld\n",Tof,newNode2.T1,newNode2.T2);
        addRangingBuffer(rangingBuffer, &newNode2, RECEIVER);
    }
}


void printRangingBuffer(RangingBuffer *buffer){
    int i = buffer->topReceiveBuffer;
    for (int j = 0; j < MAX_RANGING_BUFFER_SIZE; j++)
    {
        if(i == NULL_INDEX){
            break;
        }
        DEBUG_PRINT("ReceiveBuffer[%d]:\n",i);
        DEBUG_PRINT("sendTx:%llu,sendRx:%llu\n",buffer->receiveBuffer[i].sendTx.full,buffer->receiveBuffer[i].sendRx.full);
        DEBUG_PRINT("receiveTx:%llu,receiveRx:%llu\n",buffer->receiveBuffer[i].receiveTx.full,buffer->receiveBuffer[i].receiveRx.full);
        DEBUG_PRINT("sumTof:%lld\n",buffer->receiveBuffer[i].sumTof);
        #ifdef ENABLE_RECORD_COORDINATE
            DEBUG_PRINT("sendCoordinate:(%d,%d,%d)\n",buffer->receiveBuffer[i].sendRxCoordinate.x,buffer->receiveBuffer[i].sendRxCoordinate.y,buffer->topReceiveBuffer[i].sendRxCoordinate.z);
            DEBUG_PRINT("receiveCoordinate:(%d,%d,%d)\n",buffer->receiveBuffer[i].receiveRxCoordinate.x,buffer->receiveBuffer[i].receiveRxCoordinate.y,buffer->topReceiveBuffer[i].receiveRxCoordinate.z);
        #endif
        i = (i - 1 + MAX_RANGING_BUFFER_SIZE) % MAX_RANGING_BUFFER_SIZE;
    }
    i = buffer->topSendBuffer;
    for(int j = 0; j < MAX_RANGING_BUFFER_SIZE; j++){
        if(i == NULL_INDEX){
            break;
        }
        DEBUG_PRINT("SendBuffer[%d]:\n",i);
        DEBUG_PRINT("receiveTx:%llu,receiveRx:%llu\n",buffer->sendBuffer[i].receiveTx.full,buffer->sendBuffer[i].receiveRx.full);
        DEBUG_PRINT("sendTx:%llu,sendRx:%llu\n",buffer->sendBuffer[i].sendTx.full,buffer->sendBuffer[i].sendRx.full);
        DEBUG_PRINT("sumTof:%lld\n",buffer->sendBuffer[i].sumTof);
        #ifdef ENABLE_RECORD_COORDINATE
            DEBUG_PRINT("receiveCoordinate:(%d,%d,%d)\n",buffer->sendBuffer[i].receiveRxCoordinate.x,buffer->sendBuffer[i].receiveRxCoordinate.y,buffer->topSendBuffer[i].receiveRxCoordinate.z);
            DEBUG_PRINT("sendCoordinate:(%d,%d,%d)\n",buffer->sendBuffer[i].sendRxCoordinate.x,buffer->sendBuffer[i].sendRxCoordinate.y,buffer->topSendBuffer[i].sendRxCoordinate.z);
        #endif
        i = (i - 1 + MAX_RANGING_BUFFER_SIZE) % MAX_RANGING_BUFFER_SIZE;
    }
}