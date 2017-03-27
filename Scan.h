#ifndef SCAN2016_H
#define SCAN2016_H

#include <Behaviors/shared/BehaviorBase.h>
#include <Behaviors/shared/FSM.h>
#include <shared/UsefulMacros.h>
#include <vector>

class ConfigFile;
class Log;

namespace RoboCup2016 {

class Scan : public BehaviorBase {
 public:
  enum State {
    Initial,
    TurnHead,
    Stare,
  };

  Scan(ConfigFile &configFile, Log &log);

  virtual ~Scan();

  virtual bool run(BEHAVIOR_PARAMS);

  /** Resets entire scan behavior.
   *
   * This will remove all scan targets
   */
  void resetAll();

  /** Resets the current scanning target and chooses the appropriate
   * one from current head pan/tilt.
   *
   */
  void reset();

  /** Reset the current scanning target to the given one.
   *
   */
  void reset(int targetID);

  /**
   * Resets the number of iterations to 0.
   */
  void resetIterations();

  /**
   * Returns the number of complete scanning cycles that have completed.
   */
  int getNumIterations() const;

  /**
   * Adds a scanning target.
   *
   * @param pan the desired pan angle (in radians)
   * @param tilt the desired tilt angle (in radians)
   * @param useBottomCamera whether the bottom camera should be used (true =
   * bottom, false = top)
   * @param stareDuration how long the behavior should stay at the target (in
   * milliseconds)
   * @param headSpeed what speed the head should move at
   */
  void addTarget(float pan, float tilt, bool useBottomCamera,
                 unsigned long stareDuration, float headSpeed = 0.8f);

  /*
   * Clears all targets
   */
  void clearTargets();

  int getCurrentTargetID() const;

 private:
  PREVENT_COPY_AND_ASSIGNMENT(Scan);

  void init(unsigned long timestamp);

  void chooseInitialTarget(const RobotState &robotState);

  typedef struct ScanTarget_t {
    float pan, tilt;
    bool useBottomCamera;
    unsigned long stareDuration;
    float headSpeed;
  } ScanTarget;

  // How accurately do we have to face the target?
  const float MIN_PAN_DIFF, MIN_TILT_DIFF;
  const float MAX_TILT, MIN_TILT, MAX_PAN;

  // What is the angle threshold to consider that the head hasn't moved?
  const float TURN_ANGLE_THRESHOLD;
  // How long should we wait if the head hasn't moved while facing a target
  const float TURN_WAIT_TIME;

  Log &log;

  FSM fsm;

  float prevPan, prevTilt;
  unsigned long noTurnTimestamp;

  std::vector<ScanTarget> scanTargets;
  int initialTargetIndex;
  int currentTargetIndex;
  int numIterations;
};
}

#endif /* SCAN2016_H */
