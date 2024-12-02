#ifndef NULL_VALUE_H
#define NULL_VALUE_H

#include "ranging_table.h"

#define NULL_TIMESTAMP 0
#define NULL_TF NULL_TIMESTAMP
#define NULL_SEQ 0
#define NULL_INDEX -1

static const dwTime_t nullTimeStamp = {.full = NULL_TIMESTAMP};
static const Coordinate16_Tuple_t nullCoordinate = {.x = -1, .y = -1, .z = -1};

static inline dwTime_t create_null_timestamp() {
    dwTime_t ts = {.full = NULL_TIMESTAMP};
    return ts;
}

static inline Coordinate16_Tuple_t create_null_coordinate() {
    Coordinate16_Tuple_t coord = {.x = -1, .y = -1, .z = -1};
    return coord;
}

#endif