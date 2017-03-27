/*
//		Module to move the robots head
//		Set the Yawa and Pitch values
//
*/

#include <Behaviors/2016/skills/MoveHead.h>
#include <math.h>
// Define the logging constants
#define COMPONENT BEHAVIORS
//#define CLASS_LOG_LEVEL LOG_LEVEL_INFO
#include "Log/LogSettings.h"

namespace RoboCup2016 {

	MoveHead::MoveHead(ConfigFile &configFile, Log &_log): 
	BehaviorBase(), 
	log(_log),
	fsm(_log),
	TargetYaw(0), 
	TargetPitch(0),
	prev_Yaw(0),
	prev_Pitch(0),
	atTarget(true),
	MIN_PAN_DIFF(configFile.getRadiansFromDegrees(
          "behaviors/2016/scan/minPanDiff", 3)),
    MIN_TILT_DIFF(configFile.getRadiansFromDegrees(
          "behaviors/2016/scan/minTiltDiff", 3)),
    TURN_ANGLE_THRESHOLD(configFile.getRadiansFromDegrees(
          "behaviors/2016/scan/turnAngleThreshold", 3)),
    TURN_WAIT_TIME(configFile.getMillisecondsFromSeconds(
          "behaviors/2016/scan/turnWaitTime", 5)),
    MAX_TILT(configFile.getRadiansFromDegrees("behaviors/2016/scan/maxTilt",
                                                -30.5)),
    MIN_TILT(configFile.getRadiansFromDegrees("behaviors/2016/scan/minTilt",
                                                25.5)),
    MAX_PAN(configFile.getRadiansFromDegrees("behaviors/2016/scan/maxPan",
                                               100.5)) 
    {
    	init(_log.getTimestamp());
    }

	MoveHead::~MoveHead() {}

	bool MoveHead::setTarget(const double &Yaw, const double &Pitch)
	{
		if (fsm.inState(TurnHead))
		{
			LOG_WARN("Cannot set target while tracking!\n");
			return true;
		}
		if (fsm.inState(Initial)) 
		{
			TargetPitch = Pitch;
			TargetYaw = Yaw;
			if (fabs(TargetPitch - prev_Pitch) < MIN_TILT_DIFF && fabs(TargetYaw - prev_Yaw) < MIN_PAN_DIFF)
			{
				atTarget = true;
			}
			else
			{
				atTarget = false;
			}
			return false;
		}
	}

	bool MoveHead::run(BEHAVIOR_PARAMS,const double &Yaw, const double &Pitch)
	{
		bool o = setTarget(Yaw,Pitch);
		if (!o)
		{
			return run(BEHAVIOR_PARAMS);
		}
		else
		{
			return true;
		}
	}

	bool MoveHead::run(BEHAVIOR_PARAMS)
	{
		unsigned long timestamp = robotState.getTimestamp();
		try
		{
			fsm.startLoop(timestamp);
			while(fsm.isRunning())
			{
				if (fsm.inState(Initial))
				{
					if (!atTarget)
					{
						FSM_TRANS(TurnHead,"Turning to target\n");
					}
					else
					{
						fsm.endLoop();
						continue;	
					}
				}
				if (fsm.inState(TurnHead))
				{
					p = robotState.getHeadTilt();
					y = robotState.getHeadPan();
					p_delta = fabs(p - TargetPitch);
					y_delta = fabs(y - TargetYaw);	
					prev_Pitch = p;
					prev_Yaw = y;		
					if (p_delta < MIN_TILT_DIFF && y_delta < MIN_PAN_DIFF)
					{
						atTarget = true;
						FSM_TRANS(Initial,"Target Reached\n");
					}
					else
					{
						// Set the next location to make the head approach target
						command.getMotionCommand().headAngles(TargetYaw,TargetPitch,10);
						command.useBottomCamera(true);
					}
				}
			}
			return false;
		}
		catch (int e)
		{
			if (fsm.isRunning())
			{
				fsm.endLoop();
			}
		}
	}

	void MoveHead::init(unsigned long timestamp) 
	{
  		// Initialize the fsm
  		fsm.reset();
  		FSM_ADDSTATE(fsm, Initial);
  		FSM_ADDSTATE(fsm, TurnHead);
  		fsm.finishInit(Initial, timestamp);
	}




}