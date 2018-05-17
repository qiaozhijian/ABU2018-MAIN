#ifndef __MOTION_H
#define __MOTION_H

#define COURCE_MOTOR_ID       6
#define PITCH_MOTOR_ID				5
#define UP_STEER_MOTOR_ID     7
#define DOWN_STEER_MOTOR_ID   8



void PitchAngleMotion(float angle);

void CourseAngleMotion(float angle);

void GasMotion(float value);

void MotionRead(void);
void MotionExecute(void);
void MotionStatusUpdate(void);
void TalkToCamera(uint32_t command);
void GasEnable(void);
void GasIF(void);
#endif

