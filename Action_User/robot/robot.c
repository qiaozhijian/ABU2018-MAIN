#include "robot.h"
#include "stdint.h"
#include "task.h"



extern Robot_t gRobot;



/*机器人状态量标志位开关*/
void SetMotionFlag(uint32_t status){
  
  switch(status){
  case AT_CLAW_STATUS_OPEN:
    gRobot.sDta.AT_motionFlag|=AT_CLAW_STATUS_OPEN;
    break;
  case ~AT_CLAW_STATUS_OPEN:
    gRobot.sDta.AT_motionFlag&=~AT_CLAW_STATUS_OPEN;
    break;
  case AT_STEER_READY:
    gRobot.sDta.AT_motionFlag|=AT_STEER_READY;
    break;
  case ~AT_STEER_READY:
    gRobot.sDta.AT_motionFlag&=~AT_STEER_READY;
    break;
  case AT_SHOOT_BIG_ENABLE:
    gRobot.sDta.AT_motionFlag|=AT_SHOOT_BIG_ENABLE;
    break;
  case ~AT_SHOOT_BIG_ENABLE:
    gRobot.sDta.AT_motionFlag&=~AT_SHOOT_BIG_ENABLE;
    break;
  case AT_SHOOT_SMALL_ENABLE:
    gRobot.sDta.AT_motionFlag|=AT_SHOOT_SMALL_ENABLE;
    break;
  case ~AT_SHOOT_SMALL_ENABLE:
    gRobot.sDta.AT_motionFlag&=~AT_SHOOT_SMALL_ENABLE;
    break;
  case AT_HOLD_BALL_1_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_HOLD_BALL_1_SUCCESS;
    break;
  case ~AT_HOLD_BALL_1_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_HOLD_BALL_1_SUCCESS;
    break;
  case AT_HOLD_BALL_2_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_HOLD_BALL_2_SUCCESS;
    break;
  case ~AT_HOLD_BALL_2_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_HOLD_BALL_2_SUCCESS;
    break;
  case AT_HOLD_BALL_1_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_HOLD_BALL_1_RESPONSE_SUCCESS;
    break;
  case ~AT_HOLD_BALL_1_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_HOLD_BALL_1_RESPONSE_SUCCESS;
    break;
  case AT_HOLD_BALL_2_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_HOLD_BALL_2_RESPONSE_SUCCESS;
    break;
  case ~AT_HOLD_BALL_2_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_HOLD_BALL_2_RESPONSE_SUCCESS;
    break;
  case AT_COURSE_READ_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_COURSE_READ_SUCCESS;
    break;
  case ~AT_COURSE_READ_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_COURSE_READ_SUCCESS;
    break;
  case AT_PITCH_READ_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_PITCH_READ_SUCCESS;
    break;
  case ~AT_PITCH_READ_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_PITCH_READ_SUCCESS;
    break;
  case AT_COURSE_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_COURSE_SUCCESS;
    break;
  case ~AT_COURSE_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_COURSE_SUCCESS;
    break;
  case AT_PITCH_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_PITCH_SUCCESS;
    break;
  case ~AT_PITCH_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_PITCH_SUCCESS;
    break;
  case AT_GAS_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_GAS_SUCCESS;
    break;
  case ~AT_GAS_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_GAS_SUCCESS;
    break;
  case AT_CAMERA_TALK_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_CAMERA_TALK_SUCCESS;
    break;
  case ~AT_CAMERA_TALK_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_CAMERA_TALK_SUCCESS;
    break;
  case AT_CAMERA_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_CAMERA_RESPONSE_SUCCESS;
    break;
  case ~AT_CAMERA_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_CAMERA_RESPONSE_SUCCESS;
    break;
  case AT_PREPARE_READY:
    gRobot.sDta.AT_motionFlag|=AT_PREPARE_READY;
    break;
  case ~AT_PREPARE_READY:
    gRobot.sDta.AT_motionFlag&=~AT_PREPARE_READY;
    break;
  case AT_IS_SEND_DEBUG_DATA:
    gRobot.sDta.AT_motionFlag|=AT_IS_SEND_DEBUG_DATA;
    break;
  case ~AT_IS_SEND_DEBUG_DATA:
    gRobot.sDta.AT_motionFlag&=~AT_IS_SEND_DEBUG_DATA;
    break;
  case AT_REACH_FIRST_PLACE:
    gRobot.sDta.AT_motionFlag|=AT_REACH_FIRST_PLACE;
    break;
  case ~AT_REACH_FIRST_PLACE:
    gRobot.sDta.AT_motionFlag&=~AT_REACH_FIRST_PLACE;
    break;
  case AT_REACH_SECOND_PLACE:
    gRobot.sDta.AT_motionFlag|=AT_REACH_SECOND_PLACE;
    break;
  case ~AT_REACH_SECOND_PLACE:
    gRobot.sDta.AT_motionFlag&=~AT_REACH_SECOND_PLACE;
    break;
  case AT_REACH_THIRD_PLACE:
    gRobot.sDta.AT_motionFlag|=AT_REACH_THIRD_PLACE;
    break;
  case ~AT_REACH_THIRD_PLACE:
    gRobot.sDta.AT_motionFlag&=~AT_REACH_THIRD_PLACE;
    break;
	case AT_THE_WHEEL_SELFTEST_OVER:
		  gRobot.sDta.AT_motionFlag|=AT_THE_WHEEL_SELFTEST_OVER;
	break;
	
	case ~AT_THE_WHEEL_SELFTEST_OVER:
    gRobot.sDta.AT_motionFlag&=~AT_THE_WHEEL_SELFTEST_OVER;
    break;
	
	case AT_THE_DUCT_SELFTEST_OVER:
		gRobot.sDta.AT_motionFlag|=AT_THE_DUCT_SELFTEST_OVER;
	break;
	
	case ~AT_THE_DUCT_SELFTEST_OVER:
		gRobot.sDta.AT_motionFlag&=~AT_THE_DUCT_SELFTEST_OVER;
	break;
	
	case AT_RESET_THE_ROBOT:
		gRobot.sDta.AT_motionFlag|=AT_RESET_THE_ROBOT;
	break;
	
	case ~AT_RESET_THE_ROBOT:
		gRobot.sDta.AT_motionFlag&=~AT_RESET_THE_ROBOT;
	break;
	
  }
}


