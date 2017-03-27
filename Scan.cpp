#include <Agent/Command/Command.h>
#include <Agent/Command/MotionCommand/MotionCommand.h>
#include <Agent/RobotState.h>
#include <Behaviors/2016/skills/Scan.h>
#include <GameController/GameState.h>
#include <shared/ConfigFile/ConfigFile.h>

// Logging components
#define COMPONENT BEHAVIORS
#define CLASS_LOG_LEVEL LOG_LEVEL_INFO
#include "Log/LogSettings.h"

namespace RoboCup2016 {
Scan::Scan(ConfigFile& configFile, Log& _log)
    : BehaviorBase(),
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
                                               100.5)),
      log(_log),
      fsm(_log),
      prevPan(0),
      prevTilt(0),
      noTurnTimestamp(0),
      scanTargets(),
      initialTargetIndex(-1),
      currentTargetIndex(-1),
      numIterations(0) {
  init(_log.getTimestamp());
}

Scan::~Scan() {}

void Scan::init(unsigned long timestamp) {
  // Initialize the fsm
  fsm.reset();
  FSM_ADDSTATE(fsm, Initial);
  FSM_ADDSTATE(fsm, TurnHead);
  FSM_ADDSTATE(fsm, Stare);
  fsm.finishInit(Initial, timestamp);

  scanTargets.clear();
  initialTargetIndex = -1;
  currentTargetIndex = -1;
  numIterations = 0;
}

void Scan::resetAll() { init(log.getTimestamp()); }

void Scan::reset() {
  initialTargetIndex = -1;
  currentTargetIndex = -1;
  numIterations = 0;
  fsm.setState(Initial, "Reset scanning");
}

void Scan::reset(int targetID) {
  if ((targetID >= 0) && (targetID < (int)scanTargets.size())) {
    LOG_INFO("Reset target to %d.", targetID);
    initialTargetIndex = targetID;
    currentTargetIndex = targetID;
    numIterations = 0;
    fsm.setState(TurnHead, "Reset scanning");
  } else {
    LOG_WARN("Target ID %d does not exist. Resetting to default target.",
             targetID);
    reset();
  }
}

void Scan::resetIterations() { numIterations = 0; }

int Scan::getCurrentTargetID() const { return currentTargetIndex; }

bool Scan::run(BEHAVIOR_PARAMS) {
  unsigned long timestamp = robotState.getTimestamp();

  try {
    fsm.startLoop(timestamp);
    while (fsm.isRunning()) {
      if (fsm.inState(Initial)) {
        // Choose a target
        if (scanTargets.size() > 0) {
          if (currentTargetIndex == -1) {
            chooseInitialTarget(robotState);
          }
          if (currentTargetIndex >= 0) {
            FSM_TRANS(TurnHead, "Turn to target");
          }
        }

        fsm.endLoop();
        continue;
      } else if (fsm.inState(TurnHead)) {
        float pan = robotState.getHeadPan();
        float tilt = robotState.getHeadTilt();

        // Are we looking at the target?
        float panDelta = fabs(pan - scanTargets[currentTargetIndex].pan);
        float tiltDelta = fabs(tilt - scanTargets[currentTargetIndex].tilt);

        if (fsm.isNewState()) {
          prevPan = pan;
          prevTilt = tilt;
          noTurnTimestamp = timestamp;
          LOG_DEBUG("Turn head to target id: %d", currentTargetIndex);
        } else if ((panDelta <= MIN_PAN_DIFF) && (tiltDelta <= MIN_TILT_DIFF)) {
          // Only transition the second time this state is reached
          FSM_TRANS(Stare, "At target");
        }

        // Has the head moved?
        float angleDelta = Vector2D(prevPan - pan, prevTilt - tilt).length();
        if (angleDelta > TURN_ANGLE_THRESHOLD) {
          noTurnTimestamp = timestamp;
          prevPan = pan;
          prevTilt = tilt;
        }
        // Check if we can't reach the target
        // if the game state is {Ready, Set, or Playing}.
        // These are the only three game states where the head
        // is allowed to be moving.
        if ((gameState.getStateOfGame() == GameState::ready ||
             gameState.getStateOfGame() == GameState::set ||
             gameState.getStateOfGame() == GameState::playing) &&
            (timestamp - noTurnTimestamp > TURN_WAIT_TIME)) {
          LOG_WARN(
              "Couldn't reach target %d: (pan: %.1f deg (%.3f rad), tilt: %.1f "
              "deg (%.3f rad))",
              currentTargetIndex, TO_DEG(scanTargets[currentTargetIndex].pan),
              scanTargets[currentTargetIndex].pan,
              TO_DEG(scanTargets[currentTargetIndex].tilt),
              scanTargets[currentTargetIndex].tilt);

          command.getMotionCommand().noHeadCommand();
          FSM_TRANS(Stare, "Couldn't reach target");
        } else {
          // Turn the head
          command.getMotionCommand().headAngles(
              scanTargets[currentTargetIndex].pan,
              scanTargets[currentTargetIndex].tilt,
              scanTargets[currentTargetIndex].headSpeed);
          // Set the camera
          command.useBottomCamera(
              scanTargets[currentTargetIndex].useBottomCamera);
        }
        fsm.endLoop();
        continue;
      } else if (fsm.inState(Stare)) {
        // Have we stared long enough?
        if (fsm.timeInState() >=
            scanTargets[currentTargetIndex].stareDuration) {
          int targetIndex = currentTargetIndex;
          currentTargetIndex++;
          if ((unsigned int)currentTargetIndex >= scanTargets.size()) {
            currentTargetIndex = 0;
          }

          if (currentTargetIndex == initialTargetIndex) {
            numIterations++;
            LOG_DEBUG("Iterations: %d", numIterations);
          }

          if (currentTargetIndex != targetIndex) {
            FSM_TRANS(TurnHead, "Look at next target");
          } else {
            // Only one target, so just keep staring
            FSM_TRANS(Stare, "Only one target");
          }
        }

        if (fsm.isNewState()) {
          LOG_DEBUG("Stare at target id: %d", currentTargetIndex);
        }

        // Continue looking at the target
        command.getMotionCommand().headAngles(
            scanTargets[currentTargetIndex].pan,
            scanTargets[currentTargetIndex].tilt,
            scanTargets[currentTargetIndex].headSpeed);
        // Set the camera
        command.useBottomCamera(
            scanTargets[currentTargetIndex].useBottomCamera);

        fsm.endLoop();
        continue;
      } else {
        FSM_TRANS(Initial, "Invalid state");
      }
    }
  } catch (int e) {
    if (fsm.isRunning()) {
      fsm.endLoop();
    }
  }
  fsm.printTransitions();

  return false;
}

int Scan::getNumIterations() const { return numIterations; }

void Scan::addTarget(float pan, float tilt, bool useBottomCamera,
                     unsigned long stareDuration, float headSpeed) {
  LOG_DEBUG(
      "Adding scan target pan: %f, tilt: %f, stareDuration: %lu, headSpeed: %f",
      pan, tilt, stareDuration, headSpeed);
  Scan::ScanTarget target;
  target.pan = pan;
  target.tilt = tilt;
  target.useBottomCamera = useBottomCamera;
  target.stareDuration = stareDuration;
  target.headSpeed = headSpeed;

  if (target.pan > MAX_PAN) {
    target.pan = MAX_PAN;
  } else if (target.pan < -MAX_PAN) {
    target.pan = -MAX_PAN;
  }

  if (target.tilt > MAX_TILT) {
    target.tilt = MAX_TILT;
  } else if (target.tilt < MIN_TILT) {
    target.tilt = MIN_TILT;
  }

  // Add to list of targets
  scanTargets.push_back(target);
}

// Chooses the closest target as the initial target
void Scan::chooseInitialTarget(const RobotState& robotState) {
  float currentPan = robotState.getHeadPan();
  float currentTilt = robotState.getHeadTilt();

  int bestIndex = -1;
  float bestDist = -1;
  int numTargets = scanTargets.size();

  for (int i = 0; i < numTargets; i++) {
    float dist = Vector2D(scanTargets[i].pan - currentPan,
                          scanTargets[i].tilt - currentTilt)
                     .length();

    if ((bestDist == -1) || (dist < bestDist)) {
      bestDist = dist;
      bestIndex = i;
    }
  }

  initialTargetIndex = bestIndex;
  currentTargetIndex = bestIndex;
  numIterations = 0;
}

void Scan::clearTargets() { scanTargets.clear(); }
}
