#ifndef UKFSTRUCTURE_H
#define UKFSTRUCTURE_H

#include "MatrixTool.h"

typedef struct coordinate
{
    Matrix_t *coordinateXYZ;
} Coordinate;

typedef struct velocity
{
    Matrix_t *velocityXYZ;
} Velocity;

typedef struct posture
{
    Matrix_t *posturePRY;
} Posture;

typedef struct state
{
    Coordinate *coordinate;
    Velocity *velocity;
    Posture *posture;
} State;

typedef struct measurement
{
    Matrix_t *measurement;
} Measurement;

Coordinate *createCoordinate(double x, double y, double z);
Velocity *createVelocity(double vx, double vy, double vz);
Posture *createPosture(double pitch, double roll, double yaw);
State *createState(Coordinate *coordinate, Velocity *velocity, Posture *posture);
Measurement *createMeasurement(double row,double col);

void deleteCoordinate(Coordinate* point);
void deleteVelocity(Velocity* velocity);
void deletePosture(Posture* posture);
void deleteState(State* state);

// set函数
void setCoordinate(Coordinate *coordinate, double x, double y, double z);
void setVelocity(Velocity *velocity, double vx, double vy, double vz);
void setPosture(Posture *posture, double pitch, double roll, double yaw);
void setState(State *state, Coordinate *coordinate, Velocity *velocity, Posture *posture);
void setMeasurement(Measurement *measurement, int index, double val);

#endif