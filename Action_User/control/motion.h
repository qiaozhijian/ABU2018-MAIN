#ifndef __MOTION_H
#define __MOTION_H

void PitchAngleMotion(float angle);

void CourseAngleMotion(float angle);

void GasMotion(float value);

void MotionRead(void);
void MotionExecute(void);
void MotionStatusUpdate(void);
#endif

