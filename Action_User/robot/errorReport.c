#include "task.h"

extern Robot_t gRobot;

void ErrorReport(void)
{
  for(int i=0;i<gRobot.errorTime;i++)
  {
    switch(gRobot.error[i][0])
    {
    case HOLD_BALL1_ENABLE_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL1_ENABLE_FAIL\t");
      break;
    case HOLD_BALL1_ROTATE_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL1_ROTATE_FAIL\t");
      break;
    case HOLD_BALL2_ENABLE_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL2_ENABLE_FAIL\t");
      break;
    case HOLD_BALL2_ROTATE_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL2_ROTATE_FAIL\t");
      break;
    case CAN1_FAIL:
      USART_OUT(DEBUG_USART,"CAN1_FAIL\t");
      break;
    case CAN2_FAIL:
      USART_OUT(DEBUG_USART,"CAN2_FAIL\t");
      break;
    }
    switch(gRobot.error[i][1])
    {
    case TO_START:
      USART_OUT(DEBUG_USART,"TO_START");
      USART_Enter();
      break;
    case TO_GET_BALL_1:
      USART_OUT(DEBUG_USART,"TO_GET_BALL_1");
      USART_Enter();
      break;
    case TO_THE_AREA_1:
      USART_OUT(DEBUG_USART,"TO_THE_AREA_1");
      USART_Enter();
      break;
    case TO_THROW_BALL_1:
      USART_OUT(DEBUG_USART,"TO_THROW_BALL_1");
      USART_Enter();
      break;
    case TO_GET_BALL_2:
      USART_OUT(DEBUG_USART,"TO_GET_BALL_2");
      USART_Enter();
      break;
    case TO_THE_AREA_2:
      USART_OUT(DEBUG_USART,"TO_THE_AREA_2");
      USART_Enter();
      break;
    case TO_THROW_BALL_2:
      USART_OUT(DEBUG_USART,"TO_THROW_BALL_2");
      USART_Enter();
      break;
    case TO_GET_BALL_3:
      USART_OUT(DEBUG_USART,"TO_GET_BALL_3");
      USART_Enter();
      break;
    case TO_THE_AREA_3:
      USART_OUT(DEBUG_USART,"TO_THE_AREA_3");
      USART_Enter();
      break;
    case TO_THROW_BALL_3:
      USART_OUT(DEBUG_USART,"TO_THROW_BALL_3");
      USART_Enter();
      break;
    case END_COMPETE:
      USART_OUT(DEBUG_USART,"END_COMPETE");
      USART_Enter();
      break;
    default:
      USART_OUT(DEBUG_USART,"%d",gRobot.error[i][1]);
      USART_Enter();
      break;
    }
  }
}

void ErrorRecord(char type)
{
  int i=0;
  while(gRobot.error[i][0]!=type||gRobot.error[i][1]!=gRobot.process)
  {
    i++;
    if(i==ERROR_TIME)
    {
      gRobot.error[gRobot.errorTime][0]=type;
      if(gRobot.error[gRobot.errorTime][0]==CAN1_FAIL)
        gRobot.error[gRobot.errorTime][1]=CAN_GetLastErrorCode(CAN1);
      else if(gRobot.error[gRobot.errorTime][0]==CAN2_FAIL)
        gRobot.error[gRobot.errorTime][1]=CAN_GetLastErrorCode(CAN2);
      else
        gRobot.error[gRobot.errorTime][1]=gRobot.process;
      gRobot.errorTime++;
      break;
    }
  }
}




