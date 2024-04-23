//============================================================================
// Name        : Epos4_test.cpp
// Author      : YongJae Lee
// Version     : 0.1
// Copyright   : maxon motor ag 2014-2021
// Description : Epos controller test in C++
//============================================================================



#include <iostream>
#include "kangsan_gripper/Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>

#include <signal.h>

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
using namespace std::chrono_literals;

typedef void* HANDLE;
typedef int BOOL;

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

using namespace std;



void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;

unsigned short m_NormCurrent = 2730;//mA
unsigned short m_MaxCurrent = 3000;//mA
//EAppMode g_eAppMode = AM_DEMO;

const string g_programName = "Epo4_test";

//keyboard int 
void     INThandler(int);

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void LogInfo(string message);
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int  EposSetMotorType();
int  EposSetMotorParameter();
int EposHaltPositionMovement(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int SetGripperHomePosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);


//example code start
void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}
void LogInfo(string message)
{
	cout << message << endl;
}

void SetDefaultParameters()
{
	//USB
	g_usNodeId = 1;
	g_deviceName = "EPOS4"; 
	g_protocolStackName = "MAXON SERIAL V2"; 
	g_interfaceName = "USB"; 
	g_portName = "USB0"; 
	g_baudrate = 1000000; 
}

int OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
            LogInfo("Device Successfully Opened!");
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;
  

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int PrepareDemo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}
//example code end

//user code begin
int EposSetMotorType()
{
  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;
  if(VCS_SetMotorType(g_pKeyHandle, g_usNodeId, MT_EC_BLOCK_COMMUTATED_MOTOR, &ulErrorCode))
    lResult = MMC_SUCCESS;
  return lResult;
}

int EposProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
  else
	{
		list<long> positionList;

		positionList.push_back(5000);
		positionList.push_back(-10000);
		positionList.push_back(5000);

		for(list<long>::iterator it = positionList.begin(); it !=positionList.end(); it++)
		{
			long targetPosition = (*it);
			stringstream msg;
			msg << "move to position = " << targetPosition << ", node = " << p_usNodeId;
			LogInfo(msg.str());

			if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 0, 1, &p_rlErrorCode) == 0)
			{
				LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
				break;
			}

			sleep(1);
		}

		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt position movement");

			if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}
		}
	}

	return lResult;
}

void  INThandler(int sig)
{
    char  c;
    int lResult = MMC_FAILED;
    unsigned int ulErrorCode = 0;
    unsigned int lErrorCode = 0;
     signal(sig, SIG_IGN);
     printf("Do you really want to quit? [y/n] ");
     c = getchar();
     if (c == 'y' || c == 'Y')
     {
      EposHaltPositionMovement(g_pKeyHandle, g_usNodeId, lErrorCode);
      if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
        {
          LogError("VCS_SetDisableState", lResult, lErrorCode);
          lResult = MMC_FAILED;
        }
      if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
        {
          LogError("Closing Device and shutting down", lResult, ulErrorCode);
        }
      exit(0);
     }
          
     else
          signal(SIGINT, INThandler);
     getchar(); // Get new line character
}

int EposMoveto(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long targetPosition)
{
  int lResult = MMC_SUCCESS;
  if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 1, 1, &p_rlErrorCode) == 0)
			{
				LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}
  sleep(1);
  return lResult;
}

int EposGoalCurrent(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, int targetCurrent)
{
  int lResult = MMC_SUCCESS;
  if(VCS_SetCurrentMustEx(p_DeviceHandle, p_usNodeId, targetCurrent, &p_rlErrorCode) == 0)
			{
				LogError("VCS_SetCurrentMustEx", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}
  sleep(1);
  return lResult;
}

int EposHaltPositionMovement(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
  int lResult = MMC_SUCCESS;
  if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}
    return lResult;
}

int EposSetMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
  int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile current mode, node = " << p_usNodeId;
	LogInfo(msg.str());

//	if(VCS_ActivateCurrentMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
//	{
//		LogError("VCS_ActivateCurrentMode", lResult, p_rlErrorCode);
//		lResult = MMC_FAILED;
//	}
  if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}

  return lResult;
}

int SetGripperHomePosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
  int lResult = MMC_SUCCESS;
	stringstream msg;
  unsigned int l_HomingAcceleration = 1000;
  unsigned int l_SpeedSwitch = 150;
  unsigned int l_SpeedIndex = 150;
  unsigned int l_HomeOffset = 0;
  unsigned short l_CurrentThreshold = 1000;
  int l_HomePosition = 0;
  int l_Timeout = 5000;

  signed char l_HomingMethod = 18; //HM_POSITIVE_LIMIT_SWITCH


	msg << "set gripper home position, node = " << p_usNodeId;
	LogInfo(msg.str());
  if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}

  if(VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, l_HomingAcceleration, l_SpeedSwitch, l_SpeedIndex, l_HomeOffset, l_CurrentThreshold, l_HomePosition,  &p_rlErrorCode) == 0)
	{
		LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
  if(VCS_FindHome(p_DeviceHandle, p_usNodeId, l_HomingMethod, &p_rlErrorCode) == 0)
	{
		LogError("VCS_FindHome", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
  if(VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, l_Timeout, &p_rlErrorCode) == 0)
	{
		LogError("VCS_WaitForHomingAttained", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
  return lResult;
}

int EposPositionFeedback(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int* p_Positionals , unsigned int & p_rlErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, p_Positionals, &p_rlErrorCode)==0)
  {
    msg<<"pos : "<< p_Positionals;
    LogInfo(msg.str());
    lResult = MMC_FAILED;
  }
  return lResult;
}

int EposMovementStatus(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int* l_status , unsigned int & p_rlErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;
  if(VCS_GetMovementState(p_DeviceHandle, p_usNodeId, l_status, &p_rlErrorCode) == 0)
	{
		LogError("VCS_GetMovementState", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}

  return lResult;
}

//user code end


using std::placeholders::_1;
int keyInput = 0;
void subscribe_topic_message(const std_msgs::msg::Int8::SharedPtr msg)
{
    keyInput = msg->data;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%d'", msg->data);
}
bool flag = false;
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kangsan_gripper");
  
  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;
  unsigned int lErrorCode = 0;
    SetDefaultParameters();
  int m_Position = 0;
  int m_status = 0;

  signal(SIGINT, INThandler);

  if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
      LogError("OpenDevice", lResult, ulErrorCode);
      return lResult;
    }
  if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("PrepareDemo", lResult, ulErrorCode);
				return lResult;
			}
  if((lResult = SetGripperHomePosition(g_pKeyHandle, g_usNodeId, lErrorCode))!=MMC_SUCCESS)
			{
				LogError("SetGripperHome", lResult, ulErrorCode);
				return lResult;
			}
  if((lResult = EposSetMode(g_pKeyHandle, g_usNodeId, lErrorCode))!=MMC_SUCCESS)
			{
				LogError("Set Mode", lResult, ulErrorCode);
				return lResult;
			}
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripper_sub;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr gripper_end;
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  gripper_end = node->create_publisher<std_msgs::msg::Int8>(
    "gripper_end", qos_profile);
  gripper_sub = node->create_subscription<std_msgs::msg::Int8>(
    "gripper_sub",
    qos_profile,
    subscribe_topic_message);
  
  // if((lResult = EposProfilePositionMode(g_pKeyHandle, g_usNodeId, lErrorCode)) != MMC_SUCCESS)
	// 	{
	// 		LogError("DemoProfilePositionMode", lResult, lErrorCode);
	// 	}
	// 	else
	// 	{
	// 		if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
	// 		{
	// 			LogError("VCS_SetDisableState", lResult, lErrorCode);
	// 			lResult = MMC_FAILED;
	// 		}
	// 	}
  while(rclcpp::ok())
  {
    
    if(keyInput == 1) //open
    {
	  flag = true;
      EposMoveto(g_pKeyHandle, g_usNodeId, lErrorCode, 0);
	  m_status = 0;
      //EposGoalCurrent(g_pKeyHandle, g_usNodeId, lErrorCode, 500);
      keyInput = 0;
    }
    else if(keyInput == 2) //close
    {
	  flag = true;
      EposMoveto(g_pKeyHandle, g_usNodeId, lErrorCode, -170);
	  m_status = 0;
      //EposGoalCurrent(g_pKeyHandle, g_usNodeId, lErrorCode, -1000);
      keyInput = 0;
    }
    else if(keyInput == 3) //apply zero force
    {
      EposMoveto(g_pKeyHandle, g_usNodeId, lErrorCode, 0);
      //EposGoalCurrent(g_pKeyHandle, g_usNodeId, lErrorCode, 0);
      keyInput = 0;
    }
    else{
      keyInput = 0;
    }
    

    do {
      stringstream msg;
      EposMovementStatus(g_pKeyHandle, g_usNodeId, &m_status , lErrorCode);                     //if gripper reaches goal position, m_status returns 1.
      if(m_status)
      {
        msg << "Goal Reached" <<endl;
        std_msgs::msg::Int8 msg;
		if (flag)
		{
        	msg.data = 1;
			flag = false;
		}
		else
			msg.data = 0;
        gripper_end->publish(msg);
      }
      else
      {
        msg << "Moving.." <<endl;
      }
      LogInfo(msg.str());
      rclcpp::spin_some(node);
      this_thread::sleep_for(0.05s);
    }while(m_status==0 && rclcpp::ok());

    rclcpp::spin_some(node);
    this_thread::sleep_for(0.05s);
  }

}