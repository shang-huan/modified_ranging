#include "FreeRTOS.h"
#include "debug.h"

#include "UKFstructure.h"
#include "MatrixTool.h"
#include "stdlib.h"
#include "stdio.h"

Coordinate *createCoordinate(double x, double y, double z)
{
    Coordinate *coordinate = (Coordinate *)malloc(sizeof(Coordinate));
    if(coordinate == NULL)
    {
        DEBUG_PRINT("createCoordinate malloc failed\n");
        return NULL;
    }
    coordinate->coordinateXYZ = createMatrix_t(3, 1);
    coordinate->coordinateXYZ->data[0][0] = x;
    coordinate->coordinateXYZ->data[1][0] = y;
    coordinate->coordinateXYZ->data[2][0] = z;
    return coordinate;
}

void deleteCoordinate(Coordinate* point){
    Matrix_t_delete(point->coordinateXYZ);
    free(point);
}

Velocity *createVelocity(double vx, double vy, double vz)
{
    Velocity *velocity = (Velocity *)malloc(sizeof(Velocity));
    if(velocity == NULL)
    {
        DEBUG_PRINT("createVelocity malloc failed\n");
        return NULL;
    }
    velocity->velocityXYZ = createMatrix_t(3, 1);
    velocity->velocityXYZ->data[0][0] = vx;
    velocity->velocityXYZ->data[1][0] = vy;
    velocity->velocityXYZ->data[2][0] = vz;
    return velocity;
}

void deleteVelocity(Velocity* velocity){
    Matrix_t_delete(velocity->velocityXYZ);
    free(velocity);
}

Posture *createPosture(double pitch, double roll, double yaw)
{
    Posture *posture = (Posture *)malloc(sizeof(Posture));
    if(posture == NULL)
    {
        DEBUG_PRINT("createPosture malloc failed\n");
        return NULL;
    }
    posture->posturePRY = createMatrix_t(3, 1);
    posture->posturePRY->data[0][0] = pitch;
    posture->posturePRY->data[1][0] = roll;
    posture->posturePRY->data[2][0] = yaw;
    return posture;
}

void deletePosture(Posture* posture){
    Matrix_t_delete(posture->posturePRY);
    free(posture);
}

State *createState(Coordinate *coordinate, Velocity *velocity, Posture *posture)
{
    State *state = (State *)malloc(sizeof(State));
    state->coordinate = coordinate;
    state->velocity = velocity;
    state->posture = posture;
    return state;
}

void deleteState(State* state){
    deleteCoordinate(state->coordinate);
    deletePosture(state->posture);
    deleteVelocity(state->velocity);
    free(state);
}

Measurement *createMeasurement(double row,double col){
    Measurement *measurement = (Measurement *)malloc(sizeof(Measurement));
    if(measurement == NULL)
    {
        DEBUG_PRINT("createMeasurement malloc failed\n");
        return NULL;
    }
    measurement->measurement = createMatrix_t(row, col);
    return measurement;
}

void deleteMeasurement(Measurement* measurement){
    Matrix_t_delete(measurement->measurement);
    free(measurement);
}

// set函数
void setCoordinate(Coordinate *coordinate, double x, double y, double z)
{
    coordinate->coordinateXYZ->data[0][0] = x;
    coordinate->coordinateXYZ->data[1][0] = y;
    coordinate->coordinateXYZ->data[2][0] = z;
}
void setVelocity(Velocity *velocity, double vx, double vy, double vz)
{
    velocity->velocityXYZ->data[0][0] = vx;
    velocity->velocityXYZ->data[1][0] = vy;
    velocity->velocityXYZ->data[2][0] = vz;
}
void setPosture(Posture *posture, double pitch, double roll, double yaw)
{
    posture->posturePRY->data[0][0] = pitch;
    posture->posturePRY->data[1][0] = roll;
    posture->posturePRY->data[2][0] = yaw;
}
void setState(State *state, Coordinate *coordinate, Velocity *velocity, Posture *posture)
{
    state->coordinate = coordinate;
    state->velocity = velocity;
    state->posture = posture;
}
