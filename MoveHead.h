/*
//		Module to move the camera in a desired orientation
//		Sets the Head Pitch and Yaw
*/

#include "Behaviors/shared/BehaviorBase.h"
#include "Behaviors/shared/FSM.h"
#include "shared/UsefulMacros.h"
#include "shared/Vector/Vector2D.h"

class ConfigFile;
class Log;

namespace RoboCup2016 {


class MoveHead : public BehaviorBase {

private:

	double TargetYaw;
	double TargetPitch;
	double prev_Yaw;
	double prev_Pitch;
	bool atTarget;
	Log &log;
	FSM fsm;
	const float MIN_PAN_DIFF, MIN_TILT_DIFF;
  	const float MAX_TILT, MIN_TILT, MAX_PAN;
  // What is the angle threshold to consider that the head hasn't moved?
  	const float TURN_ANGLE_THRESHOLD;
  // How long should we wait if the head hasn't moved while facing a target
  	const float TURN_WAIT_TIME;
  	void init(unsigned long timestamp);
public:

	enum State 
	{
    Initial,
    TurnHead
  	};

	MoveHead(ConfigFile &configFile, Log &log);
	
	virtual ~MoveHead();
	
	virtual bool run(BEHAVIOR_PARAMS);

  	virtual bool run(BEHAVIOR_PARAMS, const double &Yaw, const double &Pitch);

  	bool setTarget(const double &Yaw, const double &Pitch);

};

}
