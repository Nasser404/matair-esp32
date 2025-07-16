#include <ArduinoWebsockets.h>
#include "MotionController.h"
#include "hardware_pins.h" 
#include "config.h"


extern websockets::WebsocketsClient client; 
extern Board board;


// --- Constructor ---
MotionController::MotionController() :
    // Initialize Stepper objects with their type, step pin, and direction pin
    stepper1(AccelStepper::DRIVER, CAPTURE_STEP_PIN, CAPTURE_DIR_PIN),
    stepper2(AccelStepper::DRIVER, CART_STEP_PIN, CART_DIR_PIN),
    stepper3(AccelStepper::DRIVER, ORB_STEP_PIN, ORB_DIR_PIN),

    // Initialize state variables
    currentState(MOTION_IDLE),
    previousState(MOTION_IDLE),
    stateStartTime(0),
    idleEntryTime(0),
    servoDisabled(false),
    resetProgress(0),
    // Initialize target data
    targetFromLoc(""),
    targetToLoc(""),
    targetOrb1(0), targetCart1(0), targetCapt1(0),
    targetOrb2(0), targetCart2(0), targetCapt2(0),
    targetRot1(0), targetRot2(0),
    locType1(LOC_INVALID), 
    locType2(LOC_INVALID),
    isCaptureMove(false),
    targetCaptureSlotIndex(-1),
    targetCaptureSlotPos(0),
    

    // Initialize homing flags
    homeCaptureHomed(false),
    homeCartHomed(false),
    homeOrbHomed(false),

    // Initialize Reset State Tracking variables
    resetBoardIterator(0),
    resetCZIterator(0),
    reset_currentBoardAlg(""),
    reset_targetCZSlotIndex(-1),
    reset_targetCZSlotPos(0),
    reset_targetHomeSquareAlg(""),
    retractRetryCount(0),


    stateToReturnToAfterSubSequence(MOTION_IDLE),
    subSequenceIsActive(false),
    lastMoveWasResetSubMoveFlag(false) 


{
    // Initialize pair members 
    reset_currentBoardCoords = {-1, -1};
    reset_targetHomeSquareCoords = {-1, -1};
    resetSubMoveFromCoords = {-1, -1}; 
    resetSubMoveToCoords = {-1, -1};   
    Serial.println("MotionController object created.");
}
bool MotionController::getResetSubMoveDetails(std::pair<int, int>& from, std::pair<int, int>& to) {
    if (lastMoveWasResetSubMoveFlag) { 
        from = resetSubMoveFromCoords;
        to = resetSubMoveToCoords;
        return true;
    }
    return false;
}

void MotionController::setup() {
    Serial.println("MotionController::setup() initializing hardware...");

    pinMode(ENDSTOP_CAPTURE_PIN, INPUT_PULLUP);
    stepper1.setMaxSpeed(STEPPER_SPEED);
    stepper1.setAcceleration(STEPPER_ACCEL);

    pinMode(ENDSTOP_CART_PIN, INPUT_PULLUP);
    stepper2.setMaxSpeed(STEPPER_SPEED);
    stepper2.setAcceleration(STEPPER_ACCEL);

    pinMode(ENDSTOP_ORB_PIN, INPUT_PULLUP);
    stepper3.setMaxSpeed(STEPPER_SPEED);
    stepper3.setAcceleration(STEPPER_ACCEL);

    servo1.attach(ROTATION_SERVO_PIN, 500, 2500);
    servo2.attach(GRIPPER_SERVO_PIN, 500, 2500);
    servo1.write(GRIPPER_ROT_BOARD);
    servo2.write(GripperOpen);

    pinMode(ACTUATOR_IN1_PIN, OUTPUT);
    pinMode(ACTUATOR_IN2_PIN, OUTPUT);
    digitalWrite(ACTUATOR_IN1_PIN, LOW);
    digitalWrite(ACTUATOR_IN2_PIN, LOW);
    pinMode(ACTUATOR_RETRACTED_SENSE_PIN, INPUT); 

    pinMode(BUTTON_PIN_1, INPUT);
    pinMode(BUTTON_PIN_2, INPUT);

    currentState  = MOTION_IDLE; 
    previousState = ERROR_STATE;
    Serial.println("MotionController::setup() complete. Waiting for Homing/Commands.");
    initializeCaptureZone(); // Initialize capture zone 
    startHomingSequence();
}

// --- Coordinate/Location Helper Implementations ---
bool MotionController::getTargetsForSquare(String square, long &orbTarget, long &cartTarget) {
  square.trim();
  square.toLowerCase();
  if (square.length() != 2) {
    Serial.println("Error: Square format incorrect (e.g., 'e4')");
    return false;
  }

  char fileChar = square.charAt(0);
  char rankChar = square.charAt(1);

  if (fileChar < 'a' || fileChar > 'h') {
    Serial.print("Error: Invalid file: ");
    Serial.println(fileChar);
    return false;
  }

  if (rankChar < '1' || rankChar > '8') {
    Serial.print("Error: Invalid rank: ");
    Serial.println(rankChar);
    return false;
  }

  int fileIndex = fileChar - 'a';
  int rankIndex = rankChar - '1';

  orbTarget = orbTargets[fileIndex];
  cartTarget = cartTargets[rankIndex];
  return true;
}

bool MotionController::getTargetForCapture(int captureSlot, long &captureTarget) {
  if (captureSlot < 1 || captureSlot > 32) {
    Serial.print("Error: Invalid capture slot: ");
    Serial.println(captureSlot);
    return false;
  }

  captureTarget = captureTargets[captureSlot - 1];
  return true;
}


// ==========================================================================
// ===              HELPER FUNCTIONS =========================
// ==========================================================================
bool MotionController::startManualJog(ManualActuator actuator, bool positiveDirection) {
    if (currentState != MOTION_IDLE) {
        Serial.println("MC_Jog: Cannot start jog, controller busy.");
        return false;
    }

    // No need for currentJoggingActuator or jogDirectionPositive if we are not using a state machine for jogging
    // This function will be called on "Button Down" from Nextion

    Serial.print("MC_Jog: Starting jog for ");
    float speed = 0;

    switch (actuator) {
        case ManualActuator::STEPPER_CART: {
            Serial.print("CART, Dir: "); Serial.println(positiveDirection ? "+" : "-");
            // === Cart Software Limit Check ===
            if (positiveDirection && stepper2.currentPosition() >= CART_MAX_POS) {
                Serial.println("  Cart at MAX limit. Not moving.");
                return false; 
            }
            if (!positiveDirection && stepper2.currentPosition() <= CART_MIN_POS) {
                Serial.println("  Cart at MIN limit (Home). Not moving.");
                return false;
            }

            // === Cart SAFETY Checks ===
            if (!positiveDirection) { 
                // --- Gripper Rotation Safety Check (< 2250) ---
                if (stepper2.currentPosition() < CART_SAFETY_THRESHOLD) {
                    if (servo1.read() != GRIPPER_ROT_BOARD) {
                         Serial.println("!!! SAFETY JOG STOP !!! Cart is in safety zone (<2250) but gripper is rotated!");
                         Serial.println("  Rotate gripper to 0 before jogging Cart further towards home.");
                         return false; // ABORT THE JOG COMMAND
                    }
                }
                // --- Capture Homing Safety Check (< 800) ---
                if (stepper2.currentPosition() < CART_CAPTURE_HOME_THRESHOLD) {
                    if (stepper1.currentPosition() != 0) {
                         Serial.println("!!! SAFETY JOG STOP !!! Cart is in critical zone (<800) but Capture motor is not home!");
                         Serial.println("  Home the Capture motor before jogging Cart further.");
                         return false; // ABORT THE JOG COMMAND
                    }
                }
            }
            stepper2.enableOutputs();
            speed = positiveDirection ? MANUAL_JOG_CART_SPEED : -MANUAL_JOG_CART_SPEED;
            stepper2.setSpeed(speed);
            stepper2.runSpeed();
            break;
        }

        case ManualActuator::STEPPER_ORB: {
            Serial.print("ORB, Dir: "); Serial.println(positiveDirection ? "+" : "-");

            // === Orb Software Limit Check ===
            if (positiveDirection && stepper3.currentPosition() >= ORB_MAX_POS) {
                Serial.println("  Orb at MAX limit. Not moving.");
                return false; 
            }
            if (!positiveDirection && stepper3.currentPosition() <= ORB_MIN_POS) {
                Serial.println("  Orb at MIN limit (Home). Not moving.");
                return false;
            }

            stepper3.enableOutputs();
            speed = positiveDirection ? MANUAL_JOG_ORB_SPEED : -MANUAL_JOG_ORB_SPEED;
            stepper3.setSpeed(speed);
            stepper3.runSpeed();
            break;
        }

        case ManualActuator::STEPPER_CAPTURE: {
            Serial.print("CAPTURE, Dir: "); Serial.println(positiveDirection ? "+" : "-");

            // === Capture Software Limit Check ===
            if (positiveDirection && stepper1.currentPosition() >= CAPTURE_MAX_POS) {
                Serial.println("  Capture at MAX limit. Not moving.");
                return false;
            }
            if (!positiveDirection && stepper1.currentPosition() <= CAPTURE_MIN_POS) {
                Serial.println("  Capture at MIN limit (Home). Not moving.");
                return false;
            }

            // === Capture SAFETY Check ===
            // Cannot move Capture away from home if Cart is in the critical zone
            if (stepper2.currentPosition() < CART_CAPTURE_HOME_THRESHOLD && stepper1.currentPosition() == 0 && positiveDirection) {
                 Serial.println("!!! SAFETY JOG STOP !!! Cannot move Capture away from home while Cart is in critical zone (<800).");
                 return false; 
            }

            stepper1.enableOutputs();
            speed = positiveDirection ? MANUAL_JOG_CAPTURE_SPEED : -MANUAL_JOG_CAPTURE_SPEED;
            stepper1.setSpeed(speed);
            stepper1.runSpeed();
            break;
        }

        case ManualActuator::GRIPPER_ROTATION: { // Servo1
            Serial.print("GRIPPER_ROT, Dir: "); Serial.println(positiveDirection ? "+" : "-");

            // === Gripper Rotation SAFETY Check (related to Cart position) ===
            // Cannot rotate gripper away from 0 if Cart is in the safety zone
            if (stepper2.currentPosition() < CART_SAFETY_THRESHOLD) {
                if (positiveDirection) { // Trying to rotate away from 0
                    Serial.println("!!! SAFETY JOG STOP !!! Cannot rotate gripper while Cart is in safety zone (<2250).");
                    Serial.println("  Move Cart past 2250 first.");
                    return false; // ABORT
                }
            }

            int currentAngle1 = servo1.read();
            int newAngle1 = currentAngle1 + (positiveDirection ? MANUAL_JOG_SERVO_INCREMENT : -MANUAL_JOG_SERVO_INCREMENT);
            servo1.write(constrain(newAngle1, GRIPPER_ROT_CAPTURE, GRIPPER_ROT_BOARD));
            // For servos, this is a single action, not a continuous jog start
            // The Nextion "Press" event will move it one increment.
            // If you want continuous, the Nextion needs to send repeated press events,
            // or we need a state machine here. Let's assume one increment per press.
            return true;
        }

        case ManualActuator::GRIPPER_OPEN_CLOSE: { // Servo2
            Serial.print("GRIPPER_OC, Dir: "); Serial.println(positiveDirection ? "+" : "-");
            int currentAngle2 = servo2.read();
            int newAngle2 = currentAngle2 + (positiveDirection ? MANUAL_JOG_SERVO_INCREMENT : -MANUAL_JOG_SERVO_INCREMENT);
            servo2.write(constrain(newAngle2, GripperClose, GripperOpen)); // Use defined limits
            return true;
        }

        case ManualActuator::LINEAR_ACTUATOR: {
            Serial.print("LIN_ACT, Dir: "); Serial.println(positiveDirection ? "+" : "-");
            if (positiveDirection) { // Extend
                commandExtendActuator();
            } else { // Retract
                commandRetractActuator();
            }
            break;
        }
    } 
    return true; 
}


bool MotionController::stopManualJog(ManualActuator actuator) {

    Serial.print("MC_Jog: Stopping jog for ");
    switch (actuator) {
        case ManualActuator::STEPPER_CART:
            Serial.println("CART");
            stepper2.setSpeed(0);
            stepper2.disableOutputs();
            break;
        case ManualActuator::STEPPER_ORB:
            Serial.println("ORB");
            stepper3.setSpeed(0);
            stepper3.disableOutputs();
            break;
        case ManualActuator::STEPPER_CAPTURE:
            Serial.println("CAPTURE");
            stepper1.setSpeed(0);
            stepper1.disableOutputs();
            break;
        case ManualActuator::GRIPPER_ROTATION:
             Serial.println("GRIPPER_ROT (no continuous stop needed)");
            break; 
        case ManualActuator::GRIPPER_OPEN_CLOSE:
             Serial.println("GRIPPER_OC (no continuous stop needed)");
            break; 
        case ManualActuator::LINEAR_ACTUATOR:
            Serial.println("LIN_ACT");
            commandStopActuator();
            break;
    }
    return true;
}

bool MotionController::stopAllManualJogs() {
    Serial.println("MC_Jog: Stopping ALL manual jogs.");
    stepper1.setSpeed(0);
    stepper2.setSpeed(0);
    stepper3.setSpeed(0);
    commandStopActuator();
    return true;
}

// --- Helper to get ALL potential home squares for a piece type/color ---

std::vector<std::pair<int, int>> MotionController::getPotentialHomeSquares(PieceType type, PieceColor color) {
    std::vector<std::pair<int, int>> homes;
    if (color == PieceColor::WHITE) { // White pieces (Rank 7 is 0-indexed y=7)
        switch (type) {
            case PieceType::PAWN:
                for (int i = 0; i < 8; ++i) homes.push_back({i, 6}); // a2-h2
                break;
            case PieceType::ROOK:
                homes.push_back({0, 7}); homes.push_back({7, 7}); // a1, h1
                break;
            case PieceType::KNIGHT:
                homes.push_back({1, 7}); homes.push_back({6, 7}); // b1, g1
                break;
            case PieceType::BISHOP:
                homes.push_back({2, 7}); homes.push_back({5, 7}); // c1, f1
                break;
            case PieceType::QUEEN:
                homes.push_back({3, 7}); // d1
                break;
            case PieceType::KING:
                homes.push_back({4, 7}); // e1
                break;
        }
    } else { // PieceColor::BLACK (Rank 0 is 0-indexed y=0)
        switch (type) {
            case PieceType::PAWN:
                for (int i = 0; i < 8; ++i) homes.push_back({i, 1}); // a7-h7
                break;
            case PieceType::ROOK:
                homes.push_back({0, 0}); homes.push_back({7, 0}); // a8, h8
                break;
            case PieceType::KNIGHT:
                homes.push_back({1, 0}); homes.push_back({6, 0}); // b8, g8
                break;
            case PieceType::BISHOP:
                homes.push_back({2, 0}); homes.push_back({5, 0}); // c8, f8
                break;
            case PieceType::QUEEN:
                homes.push_back({3, 0}); // d8
                break;
            case PieceType::KING:
                homes.push_back({4, 0}); // e8
                break;
        }
    }
    return homes;
} 

// --- Start Board Reset Sequence ---
bool MotionController::startBoardResetSequence() {
    if (currentState != MOTION_IDLE) {
       Serial.println("MotionController Error: Cannot start reset, busy!");
       return false;
   }
   stateToReturnToAfterSubSequence = MOTION_IDLE; 
   subSequenceIsActive = false;
   Serial.println("MotionController: Starting FULL Physical Board Reset Sequence...");
   resetP3_subMoveInProgress = false; // Initialize flag
   currentState = RESET_START; // Initial state for the whole reset
   stateStartTime = millis();
   return true;
}

// --- Capture Zone Initialization ---
void MotionController::initializeCaptureZone() {
    Serial.println("Initializing Capture Zone array...");
    for (int i = 0; i < 32; ++i) {
        capture_zone[i].occupied = false;
    }
}

// --- Reset Internal Capture Zone State ---
void MotionController::resetInternalCaptureZoneState() {
    Serial.println("[MC] Resetting Internal Capture Zone Array...");
    for (int i = 0; i < 32; ++i) {
        capture_zone[i].occupied = false;
    }

    Serial.println("[MC] Internal Capture Zone Array Reset.");
}



// --- Find Free Capture Slot ---
int MotionController::findFreeCaptureSlot(PieceColor color) {
    for (int i = 0; i < 32; ++i) {
        if (!capture_zone[i].occupied) {
            return i; 
        }
    }
    Serial.println("!!! ERROR: Capture Zone Full !!!");
    return -1; // Indicate zone is full
}


bool MotionController::startHomingSequence() {
    if (currentState != MOTION_IDLE) {
        Serial.println("MotionController Error: Cannot start homing, already busy!");
        return false;
    }
    Serial.println("MotionController: Starting Homing Sequence...");
    currentState = HOMING_START;
    stateStartTime = millis();
    // Reset homing flags
    homeCaptureHomed = false;
    homeCartHomed = false;
    homeOrbHomed = false;
    return true;
}


LocationType MotionController::parseLocation(String locStr, long &orbTarget, long &cartTarget, long &captureTarget, int &gripperRotTarget) {
    locStr.trim(); locStr.toLowerCase();
    orbTarget = stepper3.currentPosition(); cartTarget = stepper2.currentPosition();
    captureTarget = stepper1.currentPosition(); gripperRotTarget = servo1.read();

    if (locStr.startsWith("capt")) {
        if (locStr.length() <= 4) { Serial.println("Error: Missing num after 'capt'."); return LOC_INVALID; }
        int slot = locStr.substring(4).toInt();
        long specificCaptureTarget;

        if (getTargetForCapture(slot, specificCaptureTarget)) {
            captureTarget = specificCaptureTarget; cartTarget = CART_CAPTURE_POS;
            gripperRotTarget = GRIPPER_ROT_CAPTURE; 
            return LOC_CAPTURE;
        } else { return LOC_INVALID; }

    } else if (locStr.length() == 2) {

        long specificOrbTarget, specificCartTarget;

        if (getTargetsForSquare(locStr, specificOrbTarget, specificCartTarget)) {
            orbTarget = specificOrbTarget; cartTarget = specificCartTarget;
            gripperRotTarget = GRIPPER_ROT_BOARD; 
            return LOC_BOARD;
        } else { return LOC_INVALID; }

    } else {
        Serial.print("Error: Invalid loc format: '"); Serial.print(locStr); Serial.println("'");
        return LOC_INVALID;
    }
}

// --- Safety Check Implementations ---

void MotionController::enforceCartSafetyRotation(long targetCartPos) {
  if (targetCartPos < CART_SAFETY_THRESHOLD) {
    int currentAngle = servo1.read();
    if (currentAngle != GRIPPER_ROT_BOARD) {
      Serial.print("!!! SAFETY WARN !!! Cart "); Serial.print(targetCartPos); Serial.print("<Thresh. Gripper ");
      Serial.print(currentAngle); Serial.println("!=0. Forcing rot!");
      commandRotateGripper(GRIPPER_ROT_BOARD); delay(500);
      Serial.println("!!! SAFETY OK !!! Gripper rotated.");
    }
  }
}

void MotionController::enforceCaptureHomedForLowCart(long targetCartPos) {
    const unsigned long SAFETY_HOMING_TIMEOUT_MS = 15000; // Max time for safety homing
    const unsigned long WS_POLL_INTERVAL_MS = 100;     // How often to poll WebSocket (ms)
  
    if (targetCartPos < CART_CAPTURE_HOME_THRESHOLD) {
      if (stepper1.currentPosition() != 0) {
        Serial.print("!!! CRITICAL WARN !!! Cart "); Serial.print(targetCartPos); Serial.print("<CritThresh. Capture not home (");
        Serial.print(stepper1.currentPosition()); Serial.println("). Forcing home!");
  
        float o_sp = stepper1.maxSpeed(); float o_ac = stepper1.acceleration();
        stepper1.setMaxSpeed(abs(HOMING_SPEED_CAPTURE)); stepper1.setAcceleration(HOMING_ACCEL);
        stepper1.enableOutputs(); stepper1.move(-30000); // Large distance
  
        unsigned long startT = millis();
        unsigned long lastPollTime = millis(); // Timer for WebSocket polling
        bool homed = false;
        Serial.println("[SAFETY HOME] Moving...");
  
        // --- Blocking Homing Loop with Polling ---
        while (!homed && (millis() - startT < SAFETY_HOMING_TIMEOUT_MS)) {
          // Check for Endstop Hit
          if (digitalRead(ENDSTOP_CAPTURE_PIN) == LOW) {
            stepper1.stop();
            stepper1.setCurrentPosition(0);
            homed = true;
            Serial.println("[SAFETY HOME] Homed!");
          } else {
            // Keep Motor Running
            stepper1.run();
  
            // --- Poll WebSocket Client Periodically ---
            if (millis() - lastPollTime > WS_POLL_INTERVAL_MS) {
               if (client.available()) { 
                  client.poll(); 
                
               }
               lastPollTime = millis(); // Reset poll timer
            }
            // -----------------------------------------
  
            delay(1); // Yield CPU briefly
          }
        } 
  
        if (!homed) {
          Serial.println("!!! SAFETY HOME FAILED (Timeout) !!!");
          stepper1.stop();
        }
        stepper1.setMaxSpeed(o_sp); stepper1.setAcceleration(o_ac);
        stepper1.runToPosition(); // Ensure it settles at the final position (0 or stopped position)
        Serial.println(homed ? "!!! CRITICAL OK !!! Capture homed." : "!!! CRITICAL FAILED !!!");
      }
    
    }
  }


// --- Low-level Action Implementations ---

void MotionController::commandExtendActuator() {
    digitalWrite(ACTUATOR_IN1_PIN, LOW);
    digitalWrite(ACTUATOR_IN2_PIN, HIGH);
}
void MotionController::commandRetractActuator() {
    digitalWrite(ACTUATOR_IN1_PIN, HIGH);
    digitalWrite(ACTUATOR_IN2_PIN, LOW);
}
void MotionController::commandStopActuator() {
    digitalWrite(ACTUATOR_IN1_PIN, LOW);
    digitalWrite(ACTUATOR_IN2_PIN, LOW);
}
void MotionController::commandGripperOpen() { servo2.write(GripperOpen); }
void MotionController::commandGripperClose() { servo2.write(GripperClose); }
void MotionController::commandRotateGripper(int angle) { servo1.write(constrain(angle, 0, 180)); }

String MotionController::coordsToAlgebraic(int x, int y) {
    if (x < 0 || x > 7 || y < 0 || y > 7) return "??";
    char file = 'a' + x;
    char rank = '8' - y;
    String r = "";
    r += file;
    r += rank;
    return r;
}

// ==========================================================================
// === PUBLIC FUNCTIONS =====================================================
// ==========================================================================

void MotionController::update() {
    stepper1.run(); stepper2.run(); stepper3.run();

    executeStateMachine();

}

bool MotionController::isBusy() {
    return currentState != MOTION_IDLE; 
}

bool MotionController::startMoveSequence(String fromLoc, String toLoc, bool isSubSequenceCall) {

    if (!isSubSequenceCall && currentState != MOTION_IDLE) {
        Serial.println("MotionController Error: Busy (called externally when not IDLE)!");
        return false;
    }


    if (subSequenceIsActive && isSubSequenceCall) {
        Serial.println("MotionController Error: Cannot start a new sub-sequence while another is active!");
        return false;
    }


    Serial.print("MotionController: Starting move '"); Serial.print(fromLoc);
    Serial.print("' -> '"); Serial.print(toLoc);
    if(isSubSequenceCall) Serial.print("' (as sub-sequence)");
    Serial.println("'");

    targetFromLoc = fromLoc; targetToLoc = toLoc;

    locType1 = parseLocation(targetFromLoc, targetOrb1, targetCart1, targetCapt1, targetRot1);
    locType2 = parseLocation(targetToLoc, targetOrb2, targetCart2, targetCapt2, targetRot2);

    if (locType1 == LOC_INVALID || locType2 == LOC_INVALID) {
        Serial.println("MotionController Error: Invalid location(s).");
        return false;
    }

    isCaptureMove = false; // Reset flag
    if (locType2 == LOC_BOARD) {
        int toX = targetToLoc.charAt(0) - 'a';
        int toY = 7 - (targetToLoc.charAt(1) - '1');
        if (toX >= 0 && toX < 8 && toY >= 0 && toY < 8) {
             if (board.grid[toX][toY] != nullptr) {
                 isCaptureMove = true;
                 pieceBeingCaptured.type = board.grid[toX][toY]->getType();
                 pieceBeingCaptured.color = board.grid[toX][toY]->getColor();
            }
        }
    }

    if (isSubSequenceCall && currentState == RESET_P3_INITIATE_MOVE_TO_HOME) {
        this->lastMoveWasResetSubMoveFlag = true;
        // Store the actual board coordinates for this P3 sub-move
        this->resetSubMoveFromCoords = reset_currentBoardCoords; // From P3's iteration
        this->resetSubMoveToCoords = reset_targetHomeSquareCoords; // Target home from P3
    } else {
        this->lastMoveWasResetSubMoveFlag = false;
        this->resetSubMoveFromCoords = {-1,-1}; // Clear them
        this->resetSubMoveToCoords = {-1,-1};
    }
    // If called as a sub-sequence, the calling state will set 'stateToReturnToAfterSubSequence' BEFORE this call.
    // We just mark that a sub-sequence is now active.
    if (isSubSequenceCall) {
        subSequenceIsActive = true;
        // stateToReturnToAfterSubSequence should have been set by the caller
    } else {
        subSequenceIsActive = false; // Not a sub-sequence
        stateToReturnToAfterSubSequence = MOTION_IDLE; // Default return is IDLE
    }


    if (isCaptureMove) {
        currentState = DO_CAPTURE_START;
    } else {
        currentState = DO_START;
    }
    stateStartTime = millis();
    return true;
}

// ==========================================================================
// === STATE MACHINE ========================================================
// ==========================================================================

void MotionController::executeStateMachine()
{
    unsigned long timeInState = millis() - stateStartTime;

    switch (currentState)
    {

    case MOTION_IDLE: 
      if (currentState != previousState) {
        idleEntryTime = millis();
        servoDisabled = false; // Reset flag so it runs again on next entry
      }

      // Wait 1 second after entering IDLE to disable servo
      if (!servoDisabled && millis() - idleEntryTime >= 1000) {
        digitalWrite(GRIPPER_SERVO_PIN, LOW); // PWM PIN SET AT LOW TO DIABLE SERVO
        servoDisabled = true;
      }
    break;

    // =================== HOMING SEQUENCE ===================
    case HOMING_START:
        Serial.println("[MC State] HOMING_START -> HOMING_CAPTURE_START_BACKUP_MOVE");
        currentState = HOMING_CAPTURE_START_BACKUP_MOVE; 
        stateStartTime = millis();
        homeCaptureHomed = false; 
        homeCartHomed = false;
        homeOrbHomed = false;
        // Fall through

    case HOMING_CAPTURE_START_BACKUP_MOVE:
        Serial.println("[MC State] HOMING_CAPTURE_START_BACKUP_MOVE: Backing up Capture stepper...");
        stepper1.setMaxSpeed(abs(HOMING_SPEED_CAPTURE));
        stepper1.setAcceleration(HOMING_ACCEL);
        stepper1.enableOutputs();
        stepper1.move(CAPTURE_HOME_BACKUP_STEPS); // Positive relative move
        currentState = HOMING_CAPTURE_WAIT_BACKUP;
        stateStartTime = millis();
        break;

    case HOMING_CAPTURE_WAIT_BACKUP:
        if (stepper1.distanceToGo() == 0)
        {
            Serial.println("  Capture backup complete -> HOMING_CAPTURE_START_HOME_MOVE");
            currentState = HOMING_CAPTURE_START_HOME_MOVE; // Now move towards home
            stateStartTime = millis();
        }
        break;

    case HOMING_CAPTURE_START_HOME_MOVE:
        Serial.println("[MC State] HOMING_CAPTURE_START_HOME_MOVE: Commanding Capture home move...");
        stepper1.setMaxSpeed(abs(HOMING_SPEED_CAPTURE)); 
        stepper1.setAcceleration(HOMING_ACCEL);
        stepper1.enableOutputs();
        stepper1.move(-30000); // Move towards the endstop
        currentState = HOMING_CAPTURE_WAIT_HIT;
        stateStartTime = millis();
        break;

    case HOMING_CAPTURE_WAIT_HIT:
        if (digitalRead(ENDSTOP_CAPTURE_PIN) == LOW)
        { // Active LOW
            Serial.println("[MC State] HOMING_CAPTURE_WAIT_HIT: Endstop HIT!");
            stepper1.stop();
            stepper1.setCurrentPosition(0);
            homeCaptureHomed = true;
            stepper1.setMaxSpeed(STEPPER_SPEED); // Restore normal speed
            stepper1.setAcceleration(STEPPER_ACCEL);
            Serial.println("  Capture Homing Complete -> HOMING_CART_ORB_START_MOVE");
            currentState = HOMING_CART_ORB_START_MOVE;
            stateStartTime = millis();
        }
        break;

    case HOMING_CART_ORB_START_MOVE:
        Serial.println("[MC State] HOMING_CART_ORB_START_MOVE: Commanding Cart & Orb home moves...");
        //  Home Cart & Orb Steppers Simultaneously
        stepper2.setMaxSpeed(abs(HOMING_SPEED_CART_ORB));
        stepper2.setAcceleration(HOMING_ACCEL);
        stepper2.enableOutputs();
        stepper2.move(-30000); // Large distance towards endstop

        stepper3.setMaxSpeed(abs(HOMING_SPEED_CART_ORB));
        stepper3.setAcceleration(HOMING_ACCEL);
        stepper3.enableOutputs();
        stepper3.move(-30000); // Large distance towards endstop

        currentState = HOMING_CART_ORB_WAIT_HIT;
        stateStartTime = millis();
        homeCartHomed = false; // Reset flags for this phase
        homeOrbHomed = false;
        break; // Wait for hits

    case HOMING_CART_ORB_WAIT_HIT:
        // Check Cart Endstop
        if (!homeCartHomed && digitalRead(ENDSTOP_CART_PIN) == LOW)
        {
            Serial.println("  Cart Endstop HIT!");
            stepper2.stop();
            stepper2.setCurrentPosition(0);
            homeCartHomed = true;
            // Restore speed/accel
            stepper2.setMaxSpeed(STEPPER_SPEED);
            stepper2.setAcceleration(STEPPER_ACCEL);
        }

        // Check Orb Endstop
        if (!homeOrbHomed && digitalRead(ENDSTOP_ORB_PIN) == LOW)
        {
            Serial.println("  Orb Endstop HIT!");
            stepper3.stop();
            stepper3.setCurrentPosition(0);
            homeOrbHomed = true;
            // Restore speed/accel
            stepper3.setMaxSpeed(STEPPER_SPEED);
            stepper3.setAcceleration(STEPPER_ACCEL);
            // Optional: Trigger Orb backoff state
        }

        // Check if both are done
        if (homeCartHomed && homeOrbHomed)
        {
            Serial.println("[MC State] HOMING_CART_ORB_WAIT_HIT: Both homed -> HOMING_COMPLETE");
            currentState = HOMING_COMPLETE;
            stateStartTime = millis();
        }
        else if (timeInState > 20000)
        { // Timeout safety for this phase
            Serial.println("!!! ERROR: Homing Cart/Orb Timeout !!!");
            if (!homeCartHomed)
                stepper2.stop();
            if (!homeOrbHomed)
                stepper3.stop();
            // Restore speeds anyway
            stepper2.setMaxSpeed(STEPPER_SPEED);
            stepper2.setAcceleration(STEPPER_ACCEL);
            stepper3.setMaxSpeed(STEPPER_SPEED);
            stepper3.setAcceleration(STEPPER_ACCEL);
            currentState = ERROR_STATE; 
            stateStartTime = millis();
        }
        break;



    case HOMING_COMPLETE:
        Serial.println("[MC State] HOMING_COMPLETE: Homing Finished -> MOTION_IDLE");
        currentState = MOTION_IDLE;
        stateStartTime = millis();
        break;
    // =================== FULL PHYSICAL BOARD RESET SEQUENCE ==================
    case RESET_START:
        Serial.println("[MC Reset] RESET_START -> Phase 1: Clear Misplaced from Board");
        resetBoardIterator = 0;
        currentState = RESET_P1_ITERATE_BOARD;
        stateStartTime = millis();
        break;

    // === Phase 1: Clear ALL Pieces from Board to Capture Zone ===
    case RESET_P1_ITERATE_BOARD: {
        reset_currentBoardCoords.first = resetBoardIterator % 8;
        reset_currentBoardCoords.second = resetBoardIterator / 8;
        reset_currentBoardAlg = coordsToAlgebraic(reset_currentBoardCoords.first, reset_currentBoardCoords.second);

        Serial.print("[MC Reset P1] ITERATE_BOARD: Square ");
        Serial.print(resetBoardIterator);
        Serial.print(" (");
        Serial.print(reset_currentBoardAlg);
        Serial.println(")");
        
        resetProgress = resetBoardIterator/64;
        
        if (resetBoardIterator >= 64)
        {
            Serial.println("  Phase 1 (Clear Misplaced Board Pieces) Complete -> RESET_P2_START");
            currentState = RESET_P2_START;
            resetCZIterator = 0; // Initialize for Phase 2
            stateStartTime = millis();
        }
        else
        {
            currentState = RESET_P1_CHECK_SQUARE_FOR_CLEAR;
            stateStartTime = millis();
        }
    }
        break;

    case RESET_P1_CHECK_SQUARE_FOR_CLEAR: {
    {
        Piece *pieceOnSquareP1 = board.grid[reset_currentBoardCoords.first][reset_currentBoardCoords.second];
        bool pieceToClearFromBoard = false;

        if (pieceOnSquareP1 != nullptr)
        {
            PieceType type = pieceOnSquareP1->getType();
            PieceColor color = pieceOnSquareP1->getColor();
            std::vector<std::pair<int, int>> potentialHomes = getPotentialHomeSquares(type, color);
            bool isAlreadyOnAHomeSquare = false;

            for (const auto &home : potentialHomes)
            {
                if (reset_currentBoardCoords.first == home.first && reset_currentBoardCoords.second == home.second)
                {
                    isAlreadyOnAHomeSquare = true;
                    break;
                }
            }

            if (!isAlreadyOnAHomeSquare)
            {
                Serial.print("  Found MISPLACED piece to clear: ");
                Serial.print(pieceOnSquareP1->getSymbol());
                Serial.print(" at ");
                Serial.println(reset_currentBoardAlg);
                reset_pieceBeingMoved.type = type;
                reset_pieceBeingMoved.color = color;
                reset_pieceBeingMoved.occupied = true;
                pieceToClearFromBoard = true;
            }
            else
            {
                Serial.print("  Piece ");
                Serial.print(pieceOnSquareP1->getSymbol());
                Serial.print(" at ");
                Serial.print(reset_currentBoardAlg);
                Serial.println(" is already on a valid home square. Skipping clear.");
            }
        }
        else
        {
            Serial.print("  Square ");
            Serial.print(reset_currentBoardAlg);
            Serial.println(" is empty.");
        }

        if (pieceToClearFromBoard)
        {
            currentState = RESET_P1_MOVE_TO_GRAB_FROM_BOARD;
        }
        else
        {
            resetBoardIterator++;
            currentState = RESET_P1_ITERATE_BOARD;
        }
        stateStartTime = millis();
    }
}
    break;

    case RESET_P1_MOVE_TO_GRAB_FROM_BOARD:
        Serial.print("[MC Reset P1] MOVE_TO_GRAB_FROM_BOARD: Moving to ");
        Serial.println(reset_currentBoardAlg);
        getTargetsForSquare(reset_currentBoardAlg, targetOrb1, targetCart1);
        enforceCaptureHomedForLowCart(targetCart1);
        enforceCartSafetyRotation(targetCart1);
        stepper1.moveTo(0);
        stepper2.moveTo(targetCart1);
        stepper3.moveTo(targetOrb1);
        stepper1.enableOutputs();
        stepper2.enableOutputs();
        stepper3.enableOutputs();
        currentState = RESET_P1_WAIT_GRAB_POS_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_GRAB_POS_BOARD:
        if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0)
        {
            Serial.println("  Arrived -> RESET_P1_ROTATE_FOR_GRAB_BOARD");
            currentState = RESET_P1_ROTATE_FOR_GRAB_BOARD;
            stateStartTime = millis();
        }
        break;

    case RESET_P1_ROTATE_FOR_GRAB_BOARD:
        Serial.println("[MC Reset P1] ROTATE_FOR_GRAB_BOARD");
        commandRotateGripper(GRIPPER_ROT_BOARD);
        currentState = RESET_P1_WAIT_ROTATE_GRAB_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_ROTATE_GRAB_BOARD:
        if (timeInState >= 450)
        {
            Serial.println("  Rotated -> RESET_P1_GRAB_EXTEND_BOARD");
            currentState = RESET_P1_GRAB_EXTEND_BOARD;
            stateStartTime = millis();
        }
        break;

    // --- "Take" sub-sequence for P1 with Robust Retract ---
    case RESET_P1_GRAB_EXTEND_BOARD:
        Serial.println("[MC Reset P1] GRAB_EXTEND_BOARD");
        commandGripperOpen();
        commandExtendActuator();
        currentState = RESET_P1_WAIT_GRAB_EXTEND_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_GRAB_EXTEND_BOARD:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            commandStopActuator();
            currentState = RESET_P1_GRAB_CLOSE_BOARD;
            stateStartTime = millis();
        }
        break;

    case RESET_P1_GRAB_CLOSE_BOARD:
        Serial.println("[MC Reset P1] GRAB_CLOSE_BOARD");
        commandGripperClose();
        currentState = RESET_P1_WAIT_GRAB_CLOSE_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_GRAB_CLOSE_BOARD:
        if (timeInState >= 300)
        { // Gripper close time
            Serial.println("  Grab done -> Starting Robust Retract for P1 Grab");
            commandRetractActuator();                   // Start retracting motor
            retractRetryCount = 0;                      // Reset retry counter
            currentState = RESET_P1_GRAB_RETRACT_BOARD; // Go to the waiting/checking state
            stateStartTime = millis();
        }
        break;

    case RESET_P1_GRAB_RETRACT_BOARD: // timed wait and sensor check with recovery
        // Initial timed retract + first sensor check
        if (retractRetryCount == 0)
        {
            if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150)
            {
                Serial.print("  P1 Initial Retract time done. Checking sensor... ");
                if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH)
                { // Active HIGH
                    Serial.println("Sensor Active! -> RESET_P1_GRAB_CONFIRMED_BOARD");
                    commandStopActuator();
                    currentState = RESET_P1_GRAB_CONFIRMED_BOARD;
                }
                else
                {
                    Serial.println("Sensor Inactive. -> P1 Recovery: Rotate +5");
                    retractRetryCount = 1; // Move to next recovery stage
                    commandStopActuator(); // Stop retract before rotating
                    commandRotateGripper(constrain(servo1.read() - 5, 0, 180));
                }
                stateStartTime = millis(); // Reset timer for next check/action
            }
            // Stage 1: After rotation, retry retract + second sensor check
        }
        else if (retractRetryCount == 1)
        {
            if (timeInState >= 450 && timeInState < 450 + (ACTUATOR_TRAVEL_TIME_MS + 150))
            {   // Wait for rotation, then for retract
                // Check if this is the first time after rotation

                Serial.println("  P1 Rotated. Retrying retract...");
                commandRetractActuator(); // Start retract again
            }
            if (timeInState >= 450 + ACTUATOR_TRAVEL_TIME_MS + 150)
            { // After rotation + retry retract time
                Serial.print("  P1 Retry Retract time done. Checking sensor... ");
                if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH)
                { // Active HIGH
                    Serial.println("Sensor Active after retry! -> RESET_P1_GRAB_CONFIRMED_BOARD");
                    commandStopActuator();
                    currentState = RESET_P1_GRAB_CONFIRMED_BOARD;
                }
                else
                {
                    Serial.println("Sensor STILL Inactive after P1 retry! -> RESET_P1_GRAB_FAILED_BOARD");
                    commandStopActuator();
                    currentState = RESET_P1_GRAB_FAILED_BOARD;
                }
                stateStartTime = millis();
            }
        }
        break; // Keep in this state until resolved or failed

    case RESET_P1_GRAB_CONFIRMED_BOARD:
        Serial.println("[MC Reset P1] GRAB_CONFIRMED_BOARD -> RESET_P1_ROTATE_AWAY_BOARD_TO_CZ");
        currentState = RESET_P1_ROTATE_AWAY_BOARD_TO_CZ;
        stateStartTime = millis();
        break;

    case RESET_P1_GRAB_FAILED_BOARD:
        Serial.println("!!! [MC Reset P1] GRAB_FAILED_BOARD. Skipping piece.");
        resetBoardIterator++;
        currentState = RESET_P1_ITERATE_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P1_ROTATE_AWAY_BOARD_TO_CZ:
        Serial.println("[MC Reset P1] ROTATE_AWAY_BOARD_TO_CZ");
        commandRotateGripper(GRIPPER_ROT_BOARD); // Safe angle
        currentState = RESET_P1_WAIT_ROTATE_AWAY_BOARD_TO_CZ;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_ROTATE_AWAY_BOARD_TO_CZ:
        if (timeInState >= 450)
        {
            Serial.println("  Rotated -> RESET_P1_MOVE_TO_CZ_DROPOFF");
            currentState = RESET_P1_MOVE_TO_CZ_DROPOFF;
            stateStartTime = millis();
        }
        break;

    case RESET_P1_MOVE_TO_CZ_DROPOFF:
        Serial.println("[MC Reset P1] MOVE_TO_CZ_DROPOFF");
        enforceCaptureHomedForLowCart(CART_CAPTURE_POS);
        stepper1.moveTo(0);
        stepper2.moveTo(CART_CAPTURE_POS);
        stepper3.moveTo(stepper3.currentPosition());
        stepper2.enableOutputs();
        currentState = RESET_P1_WAIT_CZ_DROPOFF;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_CZ_DROPOFF:
        if (stepper2.distanceToGo() == 0)
        {
            Serial.println("  Arrived at CZ dropoff -> RESET_P1_FIND_AVAILABLE_CZ_SLOT");
            currentState = RESET_P1_FIND_AVAILABLE_CZ_SLOT;
            stateStartTime = millis();
        }
        break;

    case RESET_P1_FIND_AVAILABLE_CZ_SLOT:
        Serial.println("[MC Reset P1] FIND_AVAILABLE_CZ_SLOT");
        reset_targetCZSlotIndex = findFreeCaptureSlot(reset_pieceBeingMoved.color);
        if (reset_targetCZSlotIndex == -1)
        {
            Serial.println("!!! ERROR: CZ Full! Cannot place piece. Skipping drop.");
            resetBoardIterator++;
            currentState = RESET_P1_ITERATE_BOARD;
        }
        else
        {
            if (getTargetForCapture(reset_targetCZSlotIndex + 1, reset_targetCZSlotPos))
            {
                Serial.print("  Found free CZ slot: ");
                Serial.print(reset_targetCZSlotIndex);
                Serial.print(" -> Stepper Pos: ");
                Serial.println(reset_targetCZSlotPos);
                currentState = RESET_P1_MOVE_CZ_MOTOR_TO_SLOT;
            }
            else
            { /* Error, skip */
                resetBoardIterator++;
                currentState = RESET_P1_ITERATE_BOARD;
            }
        }
        stateStartTime = millis();
        break;

    case RESET_P1_MOVE_CZ_MOTOR_TO_SLOT:
        Serial.println("[MC Reset P1] MOVE_CZ_MOTOR_TO_SLOT");
        stepper1.moveTo(reset_targetCZSlotPos);
        stepper1.enableOutputs();
        currentState = RESET_P1_WAIT_CZ_MOTOR;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_CZ_MOTOR:
        if (stepper1.distanceToGo() == 0)
        {
            Serial.println("  CZ Motor at slot -> RESET_P1_ROTATE_FOR_CZ_RELEASE");
            currentState = RESET_P1_ROTATE_FOR_CZ_RELEASE;
            stateStartTime = millis();
        }
        break;

    case RESET_P1_ROTATE_FOR_CZ_RELEASE:
        Serial.println("[MC Reset P1] ROTATE_FOR_CZ_RELEASE");
        commandRotateGripper(GRIPPER_ROT_CAPTURE);
        currentState = RESET_P1_WAIT_ROTATE_CZ_RELEASE;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_ROTATE_CZ_RELEASE:
        if (timeInState >= 450)
        {
            Serial.println("  Rotated for CZ -> RESET_P1_PERFORM_RELEASE_EXTEND");
            currentState = RESET_P1_RELEASE_EXTEND_IN_CZ;
            stateStartTime = millis();
        }
        break;

    // --- Release sub-sequence into CZ for P1 ---
    case RESET_P1_RELEASE_EXTEND_IN_CZ:
        Serial.println("[MC Reset P1] RELEASE_EXTEND_IN_CZ");
        commandExtendActuator();
        currentState = RESET_P1_WAIT_RELEASE_EXTEND_IN_CZ;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_RELEASE_EXTEND_IN_CZ:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            commandStopActuator();
            currentState = RESET_P1_RELEASE_OPEN_IN_CZ;
            stateStartTime = millis();
        }
        break;

    case RESET_P1_RELEASE_OPEN_IN_CZ:
        Serial.println("[MC Reset P1] RELEASE_OPEN_IN_CZ");
        commandGripperOpen();
        currentState = RESET_P1_WAIT_RELEASE_OPEN_IN_CZ;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_RELEASE_OPEN_IN_CZ:
        if (timeInState >= 300)
        {
            currentState = RESET_P1_RELEASE_RETRACT_IN_CZ;
            stateStartTime = millis();
        }
        break;

    case RESET_P1_RELEASE_RETRACT_IN_CZ:
        Serial.println("[MC Reset P1] RELEASE_RETRACT_IN_CZ (Timed)");
        commandRetractActuator();
        currentState = RESET_P1_WAIT_RELEASE_RETRACT_IN_CZ;
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_RELEASE_RETRACT_IN_CZ:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            commandStopActuator();
            Serial.println("  Release in CZ done -> RESET_P1_UPDATE_LOGIC_AND_CZ_ARRAY");
            currentState = RESET_P1_UPDATE_LOGIC_AND_CZ_ARRAY;
            stateStartTime = millis();
        }
        break;

    // The UPDATE_LOGIC_AND_CZ_ARRAY needs to use reset_pieceBeingMoved correctly.
    case RESET_P1_UPDATE_LOGIC_AND_CZ_ARRAY:
        Serial.print("[MC Reset P1] UPDATE_LOGIC_AND_CZ_ARRAY: Slot ");
        Serial.print(reset_targetCZSlotIndex);
        if (reset_targetCZSlotIndex != -1 && reset_pieceBeingMoved.occupied)
        { // Ensure we have a piece
            capture_zone[reset_targetCZSlotIndex] = reset_pieceBeingMoved;
            Serial.print(" occupied with type ");
            Serial.println((int)reset_pieceBeingMoved.type);
            reset_pieceBeingMoved.occupied = false; // Clear for next potential piece
        }
        if (reset_currentBoardCoords.first != -1)
        {
            Serial.print("  Logically clearing board at: ");
            Serial.println(reset_currentBoardAlg);
            delete board.grid[reset_currentBoardCoords.first][reset_currentBoardCoords.second];
            board.grid[reset_currentBoardCoords.first][reset_currentBoardCoords.second] = nullptr;
            reset_currentBoardCoords = {-1, -1};
        }
        commandRotateGripper(GRIPPER_ROT_BOARD);
        delay(450); // TODO: Non-blocking wait
        currentState = RESET_P1_PIECE_CLEARED_TO_CZ;
        stateStartTime = millis();
        break;

    case RESET_P1_PIECE_CLEARED_TO_CZ:
        Serial.println("[MC Reset P1] PIECE_CLEARED_TO_CZ. Iterating to next board square.");
        resetBoardIterator++;
        currentState = RESET_P1_ITERATE_BOARD;
        stateStartTime = millis();
        break;

    // === Phase 2: Place ALL Pieces from Capture Zone to Board Home ===
    case RESET_P2_START:
        Serial.println("[MC Reset P2] START: Placing ALL from CZ to Home -> RESET_P2_ITERATE_CZ");
        resetCZIterator = 0; // Start iterating CZ from slot 0
        currentState = RESET_P2_ITERATE_CZ;
        stateStartTime = millis();
        break;

    case RESET_P2_ITERATE_CZ: {
        Serial.print("[MC Reset P2] ITERATE_CZ: Slot ");
        Serial.println(resetCZIterator);


        resetProgress = resetCZIterator/32;


        if (resetCZIterator >= 32)
        { // Done checking all CZ slots
            Serial.println("  Phase 2 (Place ALL from CZ) Complete -> RESET_P3_START");
            currentState = RESET_P3_START; // Move to next phase
            resetBoardIterator = 0;        // Reset board iterator if P3 uses it
            stateStartTime = millis();
        }
        else
        {
            currentState = RESET_P2_CHECK_CZ_PIECE; // Check current slot
            stateStartTime = millis();
        }
    }
        break;

    case RESET_P2_CHECK_CZ_PIECE:
    {
        bool pieceToPlaceFromCZ = false;
        reset_targetHomeSquareCoords = {-1, -1}; // Reset target home

        if (capture_zone[resetCZIterator].occupied)
        {
            PieceType type = capture_zone[resetCZIterator].type;
            PieceColor color = capture_zone[resetCZIterator].color;
            Serial.print("  CZ Slot ");
            Serial.print(resetCZIterator);
            Serial.print(" has a ");
            Serial.print(color == PieceColor::WHITE ? "W" : "B");
            Serial.print((int)type); // Basic symbol

            std::vector<std::pair<int, int>> potentialHomes = getPotentialHomeSquares(type, color);

            for (const auto &home : potentialHomes)
            {
                if (board.grid[home.first][home.second] == nullptr)
                { // Check if home square is logically empty
                    reset_targetHomeSquareCoords = home;
                    reset_targetHomeSquareAlg = coordsToAlgebraic(home.first, home.second);
                    Serial.print(". Found available home: ");
                    Serial.println(reset_targetHomeSquareAlg);
                    reset_pieceBeingMoved = capture_zone[resetCZIterator]; // Store info of piece to move
                    pieceToPlaceFromCZ = true;
                    break; // Found a spot, stop checking other potential homes for this piece
                }
            }
            if (!pieceToPlaceFromCZ)
            {
                Serial.println(". No available home square on board. Skipping.");
            }
        }
        else
        {
            Serial.print("  CZ slot ");
            Serial.print(resetCZIterator);
            Serial.println(" is empty. Skipping.");
        }

        if (pieceToPlaceFromCZ)
        {
            if (getTargetForCapture(resetCZIterator + 1, reset_targetCZSlotPos))
            {
                Serial.print("  Moving to grab from CZ slot pos: ");
                Serial.println(reset_targetCZSlotPos);
                currentState = RESET_P2_MOVE_TO_CZ_SLOT;
            }
            else
            {
                Serial.println("!!! ERROR: P2 Could not get stepper pos for CZ slot! Skipping.");
                resetCZIterator++;
                currentState = RESET_P2_ITERATE_CZ;
            }
        }
        else
        {
            resetCZIterator++;
            currentState = RESET_P2_ITERATE_CZ;
        }
        stateStartTime = millis();
    }
    break;

    case RESET_P2_MOVE_TO_CZ_SLOT:
        Serial.println("[MC Reset P2] MOVE_TO_CZ_SLOT");
        enforceCaptureHomedForLowCart(CART_CAPTURE_POS); // Cart goes to CZ interaction pos
        // Gripper should be at board angle 0
        stepper2.moveTo(CART_CAPTURE_POS);
        stepper1.moveTo(reset_targetCZSlotPos);      // Capture motor to slot
        stepper3.moveTo(stepper3.currentPosition()); // Orb stays
        stepper1.enableOutputs();
        stepper2.enableOutputs();
        currentState = RESET_P2_WAIT_CZ_SLOT_POS;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_CZ_SLOT_POS:
        if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0)
        {
            Serial.println("  Arrived at CZ slot -> RESET_P2_ROTATE_FOR_CZ_GRAB");
            currentState = RESET_P2_ROTATE_FOR_CZ_GRAB;
            stateStartTime = millis();
        }
        break;

    case RESET_P2_ROTATE_FOR_CZ_GRAB:
        Serial.println("[MC Reset P2] ROTATE_FOR_CZ_GRAB");
        commandRotateGripper(GRIPPER_ROT_CAPTURE);
        currentState = RESET_P2_WAIT_ROTATE_CZ_GRAB;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_ROTATE_CZ_GRAB:
        if (timeInState >= 450)
        {
            Serial.println("  Rotated -> RESET_P2_PERFORM_CZ_GRAB_EXTEND");
            currentState = RESET_P2_GRAB_EXTEND_FROM_CZ;
            stateStartTime = millis();
        }
        break;

    // --- "Take" sub-sequence from CZ (with Robust Retract) ---
    case RESET_P2_GRAB_EXTEND_FROM_CZ:
        Serial.println("[MC Reset P2] GRAB_EXTEND_FROM_CZ");
        commandGripperOpen();
        commandExtendActuator();
        currentState = RESET_P2_WAIT_GRAB_EXTEND_FROM_CZ;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_GRAB_EXTEND_FROM_CZ:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            commandStopActuator();
            currentState = RESET_P2_GRAB_CLOSE_FROM_CZ;
            stateStartTime = millis();
        }
        break;

    case RESET_P2_GRAB_CLOSE_FROM_CZ:
        Serial.println("[MC Reset P2] GRAB_CLOSE_FROM_CZ");
        commandGripperClose();
        currentState = RESET_P2_WAIT_GRAB_CLOSE_FROM_CZ;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_GRAB_CLOSE_FROM_CZ:
        if (timeInState >= 300)
        {
            Serial.println("  Grab from CZ done -> Starting Robust Retract for P2 Grab");
            commandRetractActuator();
            retractRetryCount = 0;
            currentState = RESET_P2_GRAB_RETRACT_FROM_CZ;
            stateStartTime = millis();
        }
        break;
    case RESET_P2_GRAB_RETRACT_FROM_CZ: // Robust retract with sensor for P2
        if (retractRetryCount == 0)
        {
            if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150)
            {
                Serial.print("  P2 Initial Retract time done. Sensor... ");
                if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH)
                {
                    Serial.println("Active! -> RESET_P2_CZ_GRAB_CONFIRMED");
                    commandStopActuator();
                    currentState = RESET_P2_CZ_GRAB_CONFIRMED;
                }
                else
                {
                    Serial.println("Inactive. -> P2 Recovery: Rotate -5");
                    retractRetryCount = 1;
                    commandStopActuator();
                    commandRotateGripper(constrain(servo1.read() - 5, 0, 180)); 
                }
                stateStartTime = millis();
            }
        }
        else if (retractRetryCount == 1)
        { // Stage 1: After rotation, retry retract
            if (timeInState >= 450)
            { // Wait for rotation, then start retract if not already
                Serial.println("  P2 Rotated. Retrying retract...");
                commandRetractActuator();
            }
            if (timeInState >= 450 + ACTUATOR_TRAVEL_TIME_MS + 150)
            { 
                Serial.print("  P2 Retry Retract time done. Sensor... ");
                if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH)
                {
                    Serial.println("Active! -> RESET_P2_CZ_GRAB_CONFIRMED");
                    commandStopActuator();
                    currentState = RESET_P2_CZ_GRAB_CONFIRMED;
                }
                else
                {
                    Serial.println("STILL Inactive! -> RESET_P2_CZ_GRAB_FAILED");
                    commandStopActuator();
                    currentState = RESET_P2_CZ_GRAB_FAILED;
                }
                stateStartTime = millis();
            }
        }
        break;

    case RESET_P2_CZ_GRAB_CONFIRMED:
        Serial.println("[MC Reset P2] CZ_GRAB_CONFIRMED -> RESET_P2_UPDATE_CZ_ARRAY_AS_FREE");
        currentState = RESET_P2_UPDATE_CZ_ARRAY_AS_FREE;
        stateStartTime = millis();
        break;
    case RESET_P2_CZ_GRAB_FAILED:
        Serial.println("!!! [MC Reset P2] CZ_GRAB_FAILED. Skipping CZ piece.");
        resetCZIterator++;
        currentState = RESET_P2_ITERATE_CZ;
        stateStartTime = millis();
        break;

    case RESET_P2_UPDATE_CZ_ARRAY_AS_FREE:
        Serial.print("[MC Reset P2] UPDATE_CZ_ARRAY_AS_FREE: Marking CZ slot ");
        Serial.print(resetCZIterator);
        Serial.println(" free.");
        if (resetCZIterator >= 0 && resetCZIterator < 32)
        {
            capture_zone[resetCZIterator].occupied = false;
        }
        currentState = RESET_P2_ROTATE_AWAY_CZ_TO_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P2_ROTATE_AWAY_CZ_TO_BOARD:
        Serial.println("[MC Reset P2] ROTATE_AWAY_CZ_TO_BOARD");
        commandRotateGripper(GRIPPER_ROT_BOARD);
        currentState = RESET_P2_WAIT_ROTATE_AWAY_CZ_TO_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_ROTATE_AWAY_CZ_TO_BOARD:
        if (timeInState >= 450)
        {
            Serial.println("  Rotated for board -> RESET_P2_MOVE_TO_HOME_SQUARE");
            currentState = RESET_P2_MOVE_TO_HOME_SQUARE;
            stateStartTime = millis();
        }
        break;

    case RESET_P2_MOVE_TO_HOME_SQUARE:
        stepper1.setMaxSpeed(STEPPER_SPEED);
        stepper1.setAcceleration(STEPPER_ACCEL);
        stepper2.setMaxSpeed(STEPPER_SPEED);
        stepper2.setAcceleration(STEPPER_ACCEL);
        stepper3.setMaxSpeed(STEPPER_SPEED);
        stepper3.setAcceleration(STEPPER_ACCEL);

        Serial.print("[MC Reset P2] MOVE_TO_HOME_SQUARE: Moving piece to ");
        Serial.println(reset_targetHomeSquareAlg);

        getTargetsForSquare(reset_targetHomeSquareAlg, targetOrb1, targetCart1);
        enforceCaptureHomedForLowCart(targetCart1);
        enforceCartSafetyRotation(targetCart1);

        stepper1.moveTo(0);
        stepper2.moveTo(targetCart1);
        stepper3.moveTo(targetOrb1);
        stepper1.enableOutputs();
        stepper2.enableOutputs();
        stepper3.enableOutputs();

        currentState = RESET_P2_WAIT_HOME_SQUARE_POS;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_HOME_SQUARE_POS:
        if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0)
        {
            Serial.println("  Arrived at home square -> RESET_P2_ROTATE_FOR_HOME_RELEASE");
            currentState = RESET_P2_ROTATE_FOR_HOME_RELEASE;
            stateStartTime = millis();
        }
        break;

    case RESET_P2_ROTATE_FOR_HOME_RELEASE:
        Serial.println("[MC Reset P2] ROTATE_FOR_HOME_RELEASE");
        commandRotateGripper(GRIPPER_ROT_BOARD); // Should already be 0
        currentState = RESET_P2_WAIT_ROTATE_HOME_RELEASE;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_ROTATE_HOME_RELEASE:
        if (timeInState >= 450)
        { // Even if no rotation, ensures sequencing
            Serial.println("  Rotated -> RESET_P2_PERFORM_HOME_RELEASE_EXTEND");
            currentState = RESET_P2_RELEASE_EXTEND_ON_BOARD;
            stateStartTime = millis();
        }
        break;

    // --- "Release" sub-sequence onto board for P2 ---
    case RESET_P2_RELEASE_EXTEND_ON_BOARD:
        Serial.println("[MC Reset P2] RELEASE_EXTEND_ON_BOARD");
        commandExtendActuator();
        currentState = RESET_P2_WAIT_RELEASE_EXTEND_ON_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_RELEASE_EXTEND_ON_BOARD:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            commandStopActuator();
            currentState = RESET_P2_RELEASE_OPEN_ON_BOARD;
            stateStartTime = millis();
        }
        break;

    case RESET_P2_RELEASE_OPEN_ON_BOARD:
        Serial.println("[MC Reset P2] RELEASE_OPEN_ON_BOARD");
        commandGripperOpen();
        currentState = RESET_P2_WAIT_RELEASE_OPEN_ON_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_RELEASE_OPEN_ON_BOARD:
        if (timeInState >= 300)
        {
            currentState = RESET_P2_RELEASE_RETRACT_ON_BOARD;
            stateStartTime = millis();
        }
        break;

    case RESET_P2_RELEASE_RETRACT_ON_BOARD:
        Serial.println("[MC Reset P2] RELEASE_RETRACT_ON_BOARD (Timed)");
        commandRetractActuator();
        currentState = RESET_P2_WAIT_RELEASE_RETRACT_ON_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P2_WAIT_RELEASE_RETRACT_ON_BOARD:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            commandStopActuator();
            Serial.println("  Release on home square done -> RESET_P2_UPDATE_LOGICAL_BOARD");
            currentState = RESET_P2_UPDATE_LOGICAL_BOARD;
            stateStartTime = millis();
        }
        break;

    case RESET_P2_UPDATE_LOGICAL_BOARD:
        Serial.println("[MC Reset P2] UPDATE_LOGICAL_BOARD");
        if (reset_targetHomeSquareCoords.first != -1 && reset_pieceBeingMoved.occupied)
        {
            Serial.print("  Logically placing type ");
            Serial.print((int)reset_pieceBeingMoved.type);
            Serial.print(" at ");
            Serial.println(reset_targetHomeSquareAlg);
            // Ensure square is actually empty before adding
            if (board.grid[reset_targetHomeSquareCoords.first][reset_targetHomeSquareCoords.second] == nullptr)
            {
                board.addPiece(reset_pieceBeingMoved.type, reset_pieceBeingMoved.color, reset_targetHomeSquareCoords);
            }
            else
            {
                Serial.println("  !!! WARNING: Target home square was not empty logically before P2 addPiece!");
            }
            reset_pieceBeingMoved.occupied = false; // Mark as processed
        }
        currentState = RESET_P2_PIECE_PLACED_ON_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P2_PIECE_PLACED_ON_BOARD:
        Serial.println("[MC Reset P2] PIECE_PLACED_ON_BOARD. Iterating to next CZ slot.");
        resetCZIterator++;
        currentState = RESET_P2_ITERATE_CZ;
        stateStartTime = millis();
        break;

    // === Phase 3: Place Misplaced Pieces from Board to Home ===
    case RESET_P3_START:
        Serial.println("[MC Reset P3] START: Placing Misplaced Board Pieces to Home -> RESET_P3_ITERATE_BOARD");
        resetBoardIterator = 0; // Start iterating board from square 0
        currentState = RESET_P3_ITERATE_BOARD;
        stateStartTime = millis();
        break;

    case RESET_P3_ITERATE_BOARD: {
        reset_currentBoardCoords.first = resetBoardIterator % 8;
        reset_currentBoardCoords.second = resetBoardIterator / 8;
        reset_currentBoardAlg = coordsToAlgebraic(reset_currentBoardCoords.first, reset_currentBoardCoords.second);

        Serial.print("[MC Reset P3] ITERATE_BOARD: Square ");
        Serial.print(resetBoardIterator);
        Serial.print(" (");
        Serial.print(reset_currentBoardAlg);
        Serial.println(")");

        resetProgress = resetBoardIterator/64;

    

        if (resetBoardIterator >= 64)
        { // Done with all board squares for phase 3
            Serial.println("  Phase 3 (Place Misplaced Board Pieces) Complete -> RESET_P4_HOME_CAPTURE_MOTOR");
            currentState = RESET_P4_HOME_CAPTURE_MOTOR; // Move to final phase
            stateStartTime = millis();
        }
        else
        {
            currentState = RESET_P3_CHECK_BOARD_PIECE;
            stateStartTime = millis();
        }
    }
        break;

    case RESET_P3_CHECK_BOARD_PIECE:
    {
        Piece *pieceOnSquareP3 = board.grid[reset_currentBoardCoords.first][reset_currentBoardCoords.second];
        bool pieceToMoveHome = false;
        reset_targetHomeSquareCoords = {-1, -1}; // Reset target

        if (pieceOnSquareP3 != nullptr)
        {
            PieceType type = pieceOnSquareP3->getType();
            PieceColor color = pieceOnSquareP3->getColor();
            std::vector<std::pair<int, int>> potentialHomes = getPotentialHomeSquares(type, color);
            bool isAlreadyHome = false;

            // Check if piece is already on one of its valid home squares
            for (const auto &home : potentialHomes)
            {
                if (reset_currentBoardCoords.first == home.first && reset_currentBoardCoords.second == home.second)
                {
                    isAlreadyHome = true;
                    break;
                }
            }

            if (!isAlreadyHome)
            {
                // Find the first available empty home square for this piece type/color
                for (const auto &home : potentialHomes)
                {
                    if (board.grid[home.first][home.second] == nullptr)
                    {
                        reset_targetHomeSquareCoords = home;
                        reset_targetHomeSquareAlg = coordsToAlgebraic(home.first, home.second);
                        Serial.print("  Found misplaced piece: ");
                        Serial.print(pieceOnSquareP3->getSymbol());
                        Serial.print(" at ");
                        Serial.print(reset_currentBoardAlg);
                        Serial.print(". Target available home: ");
                        Serial.println(reset_targetHomeSquareAlg);
                        pieceToMoveHome = true;
                        break; // Found a suitable empty home
                    }
                }
                if (!pieceToMoveHome)
                {
                    Serial.print("  Piece ");
                    Serial.print(pieceOnSquareP3->getSymbol());
                    Serial.print(" at ");
                    Serial.print(reset_currentBoardAlg);
                    Serial.println(" is misplaced, but no available home squares found. Skipping.");
                }
            }
            else
            {
                Serial.print("  Piece ");
                Serial.print(pieceOnSquareP3->getSymbol());
                Serial.print(" at ");
                Serial.print(reset_currentBoardAlg);
                Serial.println(" is already home. Skipping.");
            }
        }
        else
        {
            Serial.print("  Square ");
            Serial.print(reset_currentBoardAlg);
            Serial.println(" is empty.");
        }

        if (pieceToMoveHome)
        {
            // Only store targets
            Serial.print("[MC Reset P3] CHECK_BOARD_PIECE: Identified piece ");
            Serial.print(reset_currentBoardAlg);
            Serial.print(" needs to move to ");
            Serial.println(reset_targetHomeSquareAlg);
            currentState = RESET_P3_INITIATE_MOVE_TO_HOME; // Transition to initiate
        }
        else
        {
            resetBoardIterator++; // Move to next square
            currentState = RESET_P3_ITERATE_BOARD;
        }
        stateStartTime = millis();
    }
    break;

    case RESET_P3_INITIATE_MOVE_TO_HOME:
    {
        Serial.print("[MC Reset P3] INITIATE_MOVE_TO_HOME: For ");
        Serial.print(reset_currentBoardAlg);
        Serial.print(" -> ");
        Serial.println(reset_targetHomeSquareAlg);

        // Set the state we want to return to AFTER the DO sequence finishes
        stateToReturnToAfterSubSequence = RESET_P3_PIECE_HOMED;
        // Call startMoveSequence, indicating it's a sub-sequence call
        if (startMoveSequence(reset_currentBoardAlg, reset_targetHomeSquareAlg, true))
        {
            // If successful, startMoveSequence has set currentState to DO_START (or DO_CAPTURE_START)
            // and subSequenceIsActive to true.
            // This state (RESET_P3_INITIATE_MOVE_TO_HOME) has done its job.
            // The DO sequence will now run. DO_COMPLETE will handle returning to RESET_P3_PIECE_HOMED.
            Serial.println("  P3: DO sub-sequence initiated.");
        }
        else
        {
            Serial.println("!!! ERROR: P3 Failed to start sub-move sequence! Skipping piece.");
            // If it failed// then go to next iteration.
            stateToReturnToAfterSubSequence = MOTION_IDLE; // Clear return state
            subSequenceIsActive = false;                   // Clear flag
            resetBoardIterator++;
            currentState = RESET_P3_ITERATE_BOARD;
            stateStartTime = millis();
        }
        // No matter if it succeeded or failed to start, this state's action is done for this cycle.
        // If it succeeded, the next state will be DO_START.
        // If it failed, the next state is RESET_P3_ITERATE_BOARD.
    }
    break; 

    case RESET_P3_PIECE_HOMED:
        Serial.println("[MC Reset P3] PIECE_HOMED. Iterating to next board square.");
        resetBoardIterator++;
        currentState = RESET_P3_ITERATE_BOARD;
        stateStartTime = millis();
        break;

    // === Phase 4: Finish ===
    case RESET_P4_HOME_CAPTURE_MOTOR: {
      
        
        Serial.println("[MC Reset P4] HOME_CAPTURE_MOTOR");
        stepper1.moveTo(0);
        stepper1.enableOutputs();
        currentState = RESET_P4_WAIT_CAPTURE_MOTOR;
        stateStartTime = millis();
    }
        break;

    case RESET_P4_WAIT_CAPTURE_MOTOR:
        if (stepper1.distanceToGo() == 0)
        {
            Serial.println("  Capture motor homed -> RESET_P4_LOGICAL_BOARD_RESET");
            currentState = RESET_P4_LOGICAL_BOARD_RESET;
            stateStartTime = millis();
        }
        break;

    case RESET_P4_LOGICAL_BOARD_RESET:
        Serial.println("[MC Reset P4] LOGICAL_BOARD_RESET");
        board.resetBoard();              // Reset logical board state
        resetInternalCaptureZoneState(); // RESET CAPTURE ZONE ARRAY
        board.printBoard();
        currentState = RESET_P4_UPDATE_NEXTION;
        stateStartTime = millis();
        break;

    case RESET_P4_UPDATE_NEXTION:
        Serial.println("[MC Reset P4] UPDATE_NEXTION (Full Redraw)");
        // TO DO (NEXTION REFRESH)
        currentState = RESET_COMPLETE;
        stateStartTime = millis();
        break;

    case RESET_COMPLETE:
        Serial.println("[MC Reset] RESET_COMPLETE -> MOTION_IDLE");
        currentState = MOTION_IDLE;
        stateStartTime = millis();
        break;

    // =================== CAPTURE SEQUENCE ==================
    case DO_CAPTURE_START:
        Serial.println("[MC State] DO_CAPTURE_START -> DO_CAPTURE_SAFETY_CHECKS_TARGET");
        currentState = DO_CAPTURE_SAFETY_CHECKS_TARGET;
        stateStartTime = millis();
    // Fall through

    case DO_CAPTURE_SAFETY_CHECKS_TARGET:
        Serial.println("[MC State] DO_CAPTURE_SAFETY_CHECKS_TARGET");
        // Safety checks before moving to TARGET square (where piece to be captured is)
        enforceCaptureHomedForLowCart(targetCart2); // Use targetCart2 (destination cart pos)
        enforceCartSafetyRotation(targetCart2);
        Serial.println("  Safety checks for capture target complete.");
        currentState = DO_CAPTURE_MOVE_TO_TARGET;
        stateStartTime = millis();
        // Fall through

    case DO_CAPTURE_MOVE_TO_TARGET:
        Serial.println("[MC State] DO_CAPTURE_MOVE_TO_TARGET: Commanding steppers...");
        // Move Cart/Orb to the TARGET square ('to' location)
        stepper1.stop(); // Ensure capture stepper stays put (should be home)
        stepper2.moveTo(targetCart2);
        stepper3.moveTo(targetOrb2);
        stepper2.enableOutputs();
        stepper3.enableOutputs();
        currentState = DO_CAPTURE_WAIT_TARGET;
        stateStartTime = millis();
        break; 

    case DO_CAPTURE_WAIT_TARGET:
        if (stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0)
        {
            Serial.println("[MC State] DO_CAPTURE_WAIT_TARGET: Arrived -> DO_CAPTURE_ROTATE_TARGET");
            currentState = DO_CAPTURE_ROTATE_TARGET;
            stateStartTime = millis();
        }
        break; 

    case DO_CAPTURE_ROTATE_TARGET:
        Serial.println("[MC State] DO_CAPTURE_ROTATE_TARGET: Commanding rotation...");
        commandRotateGripper(targetRot2); // Rotate gripper for board interaction
        currentState = DO_CAPTURE_WAIT_ROTATE_TARGET;
        stateStartTime = millis();
        break; 

    case DO_CAPTURE_WAIT_ROTATE_TARGET:
        if (timeInState >= 450)
        {
            Serial.println("[MC State] DO_CAPTURE_WAIT_ROTATE_TARGET: Rotated -> DO_CAPTURE_PERFORM_TAKE_EXTEND");
            currentState = DO_CAPTURE_PERFORM_TAKE_EXTEND;
            stateStartTime = millis();
        }
        break;

    // --- Grab the piece being captured  ---
    case DO_CAPTURE_PERFORM_TAKE_EXTEND:
        Serial.println("[MC State] DO_CAPTURE_PERFORM_TAKE_EXTEND: Opening & Extending...");
        commandGripperOpen();
        commandExtendActuator();
        currentState = DO_CAPTURE_WAIT_TAKE_EXTEND;
        stateStartTime = millis();
        break;

    case DO_CAPTURE_WAIT_TAKE_EXTEND:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            Serial.println("  Extend complete.");
            commandStopActuator();
            currentState = DO_CAPTURE_PERFORM_TAKE_GRAB;
            stateStartTime = millis();
        }
        break;

    case DO_CAPTURE_PERFORM_TAKE_GRAB:
        Serial.println("[MC State] DO_CAPTURE_PERFORM_TAKE_GRAB: Closing gripper...");
        commandGripperClose();
        currentState = DO_CAPTURE_WAIT_TAKE_GRAB;
        stateStartTime = millis();
        break;

    case DO_CAPTURE_WAIT_TAKE_GRAB:
        if (timeInState >= 300)
        {
            Serial.println("  Grab complete.");
            currentState = DO_CAPTURE_PERFORM_TAKE_RETRACT;
            stateStartTime = millis();
        }
        break;

    case DO_CAPTURE_PERFORM_TAKE_RETRACT:
        Serial.println("[MC State] DO_CAPTURE_PERFORM_TAKE_RETRACT: Retracting...");
        commandRetractActuator();
        currentState = DO_CAPTURE_WAIT_TAKE_RETRACT;
        stateStartTime = millis();
        break;

    case DO_CAPTURE_WAIT_TAKE_RETRACT: 
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            Serial.println("  Retract complete (Captured piece held).");
            commandStopActuator();
            // move towards capture zone dropoff point (Cart only first)
            currentState = DO_CAPTURE_MOVE_TO_DROPOFF; 
            stateStartTime = millis();
        }
        break;

    // --- Move to Capture Zone Dropoff ---
    case DO_CAPTURE_MOVE_TO_DROPOFF:
        Serial.println("[MC State] DO_CAPTURE_MOVE_TO_DROPOFF: Commanding Cart to dropoff...");
        // Move ONLY the Cart to the predefined capture interaction position first
        enforceCaptureHomedForLowCart(CART_CAPTURE_POS);
        enforceCartSafetyRotation(CART_CAPTURE_POS);
        stepper1.stop();                             // Capture stays home for now
        stepper3.moveTo(stepper3.currentPosition()); // Orb stays put
        stepper2.moveTo(CART_CAPTURE_POS);
        stepper2.enableOutputs();
        currentState = DO_CAPTURE_WAIT_DROPOFF; // Wait for Cart to arrive
        stateStartTime = millis();
        break;

    case DO_CAPTURE_WAIT_DROPOFF:
        if (stepper2.distanceToGo() == 0)
        {
            Serial.println("[MC State] DO_CAPTURE_WAIT_DROPOFF: Cart Arrived -> DO_CAPTURE_FIND_SLOT");
            currentState = DO_CAPTURE_FIND_SLOT;
            stateStartTime = millis();
        }
        break;

    case DO_CAPTURE_FIND_SLOT:
        Serial.println("[MC State] DO_CAPTURE_FIND_SLOT: Finding free slot...");
        targetCaptureSlotIndex = findFreeCaptureSlot(pieceBeingCaptured.color);
        if (targetCaptureSlotIndex == -1)
        {
            Serial.println("!!! ERROR: Capture zone full! Cannot place piece. !!!");
            currentState = ERROR_STATE;
        }
        else
        {
            if (getTargetForCapture(targetCaptureSlotIndex + 1, targetCaptureSlotPos))
            { // +1 for 1-32 index
                Serial.print("  Found free slot: ");
                Serial.print(targetCaptureSlotIndex);
                Serial.print(" -> Stepper Pos: ");
                Serial.println(targetCaptureSlotPos);
                // NOW that we have the target position, move the Capture Stepper
                currentState = DO_CAPTURE_MOVE_CAPTURE_STEPPER;
            }
            else
            {
                Serial.println("!!! ERROR: Could not get stepper pos for found capture slot!");
                currentState = ERROR_STATE;
            }
        }
        stateStartTime = millis();
        break; 

    // --- Move Capture Stepper & Rotate Gripper ---
    case DO_CAPTURE_MOVE_CAPTURE_STEPPER: 
        Serial.println("[MC State] DO_CAPTURE_MOVE_CAPTURE_STEPPER: Moving Capture stepper...");
        stepper1.moveTo(targetCaptureSlotPos); // Move capture stepper to calculated slot pos
        stepper1.enableOutputs();
        currentState = DO_CAPTURE_WAIT_CAPTURE_STEPPER; 
        stateStartTime = millis();
        break;

    case DO_CAPTURE_WAIT_CAPTURE_STEPPER: 
        if (stepper1.distanceToGo() == 0)
        {
            Serial.println("[MC State] DO_CAPTURE_WAIT_CAPTURE_STEPPER: Arrived -> DO_CAPTURE_ROTATE_GRIPPER_CAPZONE");
            // NOW Rotate the Gripper
            currentState = DO_CAPTURE_ROTATE_GRIPPER_CAPZONE; // <<< Rotate Gripper LAST before release
            stateStartTime = millis();
        }
        break;

        // --- Rotate Gripper ---
    case DO_CAPTURE_ROTATE_GRIPPER_CAPZONE: 
        Serial.println("[MC State] DO_CAPTURE_ROTATE_GRIPPER_CAPZONE: Rotating for capture zone...");
        commandRotateGripper(GRIPPER_ROT_CAPTURE);     // Rotate to capture zone angle
        currentState = DO_CAPTURE_WAIT_ROTATE_CAPZONE; 
        stateStartTime = millis();
        break;

    case DO_CAPTURE_WAIT_ROTATE_CAPZONE:
        if (timeInState >= 450)
        {
            Serial.println("  Rotated for capture zone -> DO_CAPTURE_PERFORM_RELEASE_EXTEND");
            // NOW proceed with releasing the piece
            currentState = DO_CAPTURE_PERFORM_RELEASE_EXTEND; 
            stateStartTime = millis();
        }
        break;

    // --- Release the captured piece into the slot  ---
    case DO_CAPTURE_PERFORM_RELEASE_EXTEND:
        Serial.println("[MC State] DO_CAPTURE_PERFORM_RELEASE_EXTEND: Extending...");
        commandExtendActuator();
        currentState = DO_CAPTURE_WAIT_RELEASE_EXTEND;
        stateStartTime = millis();
        break;
    case DO_CAPTURE_WAIT_RELEASE_EXTEND:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            Serial.println("  Extend complete.");
            commandStopActuator();
            currentState = DO_CAPTURE_PERFORM_RELEASE_OPEN;
            stateStartTime = millis();
        }
        break;
    case DO_CAPTURE_PERFORM_RELEASE_OPEN:
        Serial.println("[MC State] DO_CAPTURE_PERFORM_RELEASE_OPEN: Opening gripper...");
        commandGripperOpen();
        currentState = DO_CAPTURE_WAIT_RELEASE_OPEN;
        stateStartTime = millis();
        break;
    case DO_CAPTURE_WAIT_RELEASE_OPEN:
        if (timeInState >= 300)
        {
            Serial.println("  Open complete.");
            // Update capture zone array NOW that piece is placed
            if (targetCaptureSlotIndex != -1)
            {
                capture_zone[targetCaptureSlotIndex] = pieceBeingCaptured; // Store piece info
                capture_zone[targetCaptureSlotIndex].occupied = true;
                Serial.print("  Updated capture_zone[");
                Serial.print(targetCaptureSlotIndex);
                Serial.println("]");
            }
            currentState = DO_CAPTURE_PERFORM_RELEASE_RETRACT;
            stateStartTime = millis();
        }
        break;
    case DO_CAPTURE_PERFORM_RELEASE_RETRACT:
        Serial.println("[MC State] DO_CAPTURE_PERFORM_RELEASE_RETRACT: Retracting...");
        commandRetractActuator();
        currentState = DO_CAPTURE_WAIT_RELEASE_RETRACT;
        stateStartTime = millis();
        break;
    case DO_CAPTURE_WAIT_RELEASE_RETRACT:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            Serial.println("  Retract complete (Captured piece released).");
            commandStopActuator();
            // Finished handling the captured piece. Now proceed to move the original piece.
            currentState = DO_CAPTURE_COMPLETE;
            stateStartTime = millis();
        }
        break;

    case DO_CAPTURE_COMPLETE:
        Serial.println("[MC State] DO_CAPTURE_COMPLETE -> DO_START (to move original piece)");
        // Need to rotate gripper back to board angle BEFORE moving steppers away
        commandRotateGripper(GRIPPER_ROT_BOARD);
        // Small delay 
        delay(450);
        // Now we proceed as if starting the move from the beginning, but without the capture check
        // Go back to the safety checks for the *source* location of the piece we actually want to move
        currentState = DO_SAFETY_CHECKS_SOURCE;
        stateStartTime = millis();
        break; //

    case DO_START:
        Serial.println("[MC State] DO_START (Non-Capture) -> DO_SAFETY_CHECKS_SOURCE");
        currentState = DO_SAFETY_CHECKS_SOURCE;
        stateStartTime = millis();

    case DO_SAFETY_CHECKS_SOURCE:
        Serial.println("[MC State] DO_SAFETY_CHECKS_SOURCE");
        enforceCaptureHomedForLowCart(targetCart1);
        enforceCartSafetyRotation(targetCart1);
        Serial.println("  Safety checks source complete.");
        currentState = DO_MOVE_STEPPERS_TO_SOURCE;
        stateStartTime = millis();
        // Fall through

    case DO_MOVE_STEPPERS_TO_SOURCE:
        Serial.println("[MC State] DO_MOVE_STEPPERS_TO_SOURCE: Commanding...");
        stepper1.setMaxSpeed(STEPPER_SPEED);
        stepper1.setAcceleration(STEPPER_ACCEL);
        stepper2.setMaxSpeed(STEPPER_SPEED);
        stepper2.setAcceleration(STEPPER_ACCEL);
        stepper3.setMaxSpeed(STEPPER_SPEED);
        stepper3.setAcceleration(STEPPER_ACCEL); // <<< ENSURE FOR ORB

        stepper1.moveTo(targetCapt1);
        stepper2.moveTo(targetCart1);
        stepper3.moveTo(targetOrb1);
        stepper1.enableOutputs();
        stepper2.enableOutputs();
        stepper3.enableOutputs();
        currentState = DO_WAIT_STEPPERS_SOURCE;
        stateStartTime = millis();
        break;

    case DO_WAIT_STEPPERS_SOURCE:
    {
        bool cartDone = (stepper2.distanceToGo() == 0);
        bool orbDone = (locType1 != LOC_BOARD) || (stepper3.distanceToGo() == 0);
        bool captDone = (locType1 != LOC_CAPTURE) || (stepper1.distanceToGo() == 0);
        if (cartDone && orbDone && captDone)
        {
            Serial.println("[MC State] DO_WAIT_STEPPERS_SOURCE: Done -> DO_ROTATE_GRIPPER_SOURCE");
            currentState = DO_ROTATE_GRIPPER_SOURCE;
            stateStartTime = millis();
        }
    }
    break; 

    case DO_ROTATE_GRIPPER_SOURCE:
    {
        Serial.println("[MC State] DO_ROTATE_GRIPPER_SOURCE: Commanding...");
        commandRotateGripper(targetRot1);
        currentState = DO_WAIT_GRIPPER_SOURCE;
        stateStartTime = millis();
    }
    break; 

    case DO_WAIT_GRIPPER_SOURCE:
    {
        if (timeInState >= 450)
        {
            Serial.println("[MC State] DO_WAIT_GRIPPER_SOURCE: Done -> DO_PERFORM_TAKE_EXTEND");
            currentState = DO_PERFORM_TAKE_EXTEND;
            stateStartTime = millis();
        }
    }
    break; 

    // --- REGULAR TAKE SEQUENCE  ---
    case DO_PERFORM_TAKE_EXTEND:
    {
        Serial.println("[MC State] DO_PERFORM_TAKE_EXTEND: Commanding...");
        commandGripperOpen();
        commandExtendActuator();
        currentState = DO_WAIT_TAKE_EXTEND;
        stateStartTime = millis();
    }
    break;

    case DO_WAIT_TAKE_EXTEND:
    {
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            Serial.println("[MC State] DO_WAIT_TAKE_EXTEND: Done -> DO_PERFORM_TAKE_GRAB");
            commandStopActuator();
            currentState = DO_PERFORM_TAKE_GRAB;
            stateStartTime = millis();
        }
    }
    break; 

    case DO_PERFORM_TAKE_GRAB:
        Serial.println("[MC State] DO_PERFORM_TAKE_GRAB: Commanding...");
        commandGripperClose();
        currentState = DO_WAIT_TAKE_GRAB;
        stateStartTime = millis();
        break; 

    case DO_WAIT_TAKE_GRAB: 
        if (currentState == DO_WAIT_TAKE_GRAB && timeInState >= 300)
        {
            Serial.println("[MC State] DO_WAIT_TAKE_GRAB: Done -> DO_PERFORM_TAKE_RETRACT_INITIAL");
            currentState = DO_PERFORM_TAKE_RETRACT_INITIAL; // <<< Start retract sequence
            stateStartTime = millis();
        }
        break;

    // --- New Retract Sequence with Sensor Check ---
    case DO_PERFORM_TAKE_RETRACT_INITIAL:
        Serial.println("[MC State] DO_PERFORM_TAKE_RETRACT_INITIAL: Starting timed retract...");
        commandRetractActuator(); // Start retracting motor
        retractRetryCount = 0;    // Reset retry counter
        currentState = DO_WAIT_TAKE_RETRACT_INITIAL;
        stateStartTime = millis();
        break;

    case DO_WAIT_TAKE_RETRACT_INITIAL:
        // Wait slightly longer than the expected travel time to ensure it *should* hit the limit
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150)
        {
            Serial.println("  Initial retract time elapsed -> DO_CHECK_TAKE_RETRACT_SENSOR");
            // *** DO NOT STOP THE ACTUATOR YET *** - Keep power applied for sensor check
            currentState = DO_CHECK_TAKE_RETRACT_SENSOR;
            stateStartTime = millis(); // Reset timer for the check phase
        }
        break;

    case DO_CHECK_TAKE_RETRACT_SENSOR:
        // Give a brief moment for the sensor signal to stabilize after potentially hitting limit
        if (timeInState >= 100)
        { // Wait 100ms before checking sensor
            Serial.print("  Checking retract sensor (Pin ");
            Serial.print(ACTUATOR_RETRACTED_SENSE_PIN);
            Serial.print(")... ");
            // *** CHECK THE SENSOR PIN *** (Adjust LOW/HIGH based on your circuit)
            if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH)
            {
                Serial.println("Sensor Active (Retracted!) -> DO_TAKE_RETRACT_CONFIRMED");
                commandStopActuator(); // Successfully retracted, NOW stop motor
                currentState = DO_TAKE_RETRACT_CONFIRMED;
            }
            else
            {
                Serial.println("Sensor Inactive (Not fully retracted).");
                // Check if we should try recovery
                if (retractRetryCount < 1)
                { // Try recovery sequence once
                    Serial.println("  -> Starting Recovery Attempt 1 (Rotate)");
                    currentState = DO_RECOVER_TAKE_RETRACT_ROTATE;
                    retractRetryCount++;
                }
                else
                {
                    Serial.println("!!! ERROR: Retract Sensor check failed after retries! -> DO_TAKE_RETRACT_FAILED");
                    commandStopActuator(); // Stop trying
                    currentState = DO_TAKE_RETRACT_FAILED;
                }
            }
            stateStartTime = millis(); // Reset timer for next state
        }
        break; 

    // --- Recovery Sequence ---
    case DO_RECOVER_TAKE_RETRACT_ROTATE:
    {
        Serial.println("[MC State] DO_RECOVER_TAKE_RETRACT_ROTATE: Rotating gripper slightly...");
        commandStopActuator(); // Stop retract command first
        // Rotate gripper slightly (5 degrees from current) 
        int currentRot = servo1.read();
        commandRotateGripper(constrain(currentRot - 5, 0, 180)); // Rotate -5 degrees
        currentState = DO_WAIT_RECOVER_ROTATE;
        stateStartTime = millis();
    }
    break;

    case DO_WAIT_RECOVER_ROTATE:
    {
        if (timeInState >= 400)
        { // Wait for rotation servo
            Serial.println("  Rotation complete -> DO_RECOVER_TAKE_RETRACT_RETRY");
            currentState = DO_RECOVER_TAKE_RETRACT_RETRY;
            stateStartTime = millis();
        }
    }
    break;

    case DO_RECOVER_TAKE_RETRACT_RETRY:
    {
        Serial.println("[MC State] DO_RECOVER_TAKE_RETRACT_RETRY: Retrying retract...");
        commandRetractActuator(); // Try retracting again
        currentState = DO_WAIT_RECOVER_RETRY;
        stateStartTime = millis();
    }
    break;

    case DO_WAIT_RECOVER_RETRY:
    {
        // Wait normal retract time again
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150)
        {
            Serial.println("  Retry retract time elapsed -> Checking sensor again...");
            // Go back to sensor check state
            currentState = DO_CHECK_TAKE_RETRACT_SENSOR; // Loop back to check sensor
            stateStartTime = millis();
        }
    }
    break;
    // --- Retract Outcome States ---
    case DO_TAKE_RETRACT_CONFIRMED:
    {
        Serial.println("[MC State] DO_TAKE_RETRACT_CONFIRMED: 'Take' sequence fully complete -> DO_SAFETY_CHECKS_DEST");
        // Retraction successful, go to the next major phase (Move to Destination)
        currentState = DO_SAFETY_CHECKS_DEST;
        stateStartTime = millis();
    }
    break; 

    case DO_TAKE_RETRACT_FAILED:
        Serial.println("[MC State] DO_TAKE_RETRACT_FAILED: Retraction failed! -> ERROR_STATE");
        // Could not confirm retraction, enter error state
        currentState = ERROR_STATE;
        stateStartTime = millis();
        break; 

    case DO_SAFETY_CHECKS_DEST:
    {
        Serial.println("[MC State] DO_SAFETY_CHECKS_DEST");
        enforceCaptureHomedForLowCart(targetCart2);
        enforceCartSafetyRotation(targetCart2);
        Serial.println("  Safety checks dest complete.");
        if (locType2 == LOC_CAPTURE)
        {
            currentState = DO_MOVE_CART_TO_DEST;
        }
        else
        {
            currentState = DO_ROTATE_GRIPPER_DEST;
        }
        stateStartTime = millis();
    }
    break; 

    case DO_MOVE_CART_TO_DEST:
    {
        Serial.println("[MC State] DO_MOVE_CART_TO_DEST: Commanding...");
        stepper2.moveTo(targetCart2);
        stepper2.enableOutputs();
        currentState = DO_WAIT_CART_DEST;
        stateStartTime = millis();
    }
    break; 

    case DO_WAIT_CART_DEST:
        if (stepper2.distanceToGo() == 0)
        {
            Serial.println("[MC State] DO_WAIT_CART_DEST: Done -> DO_ROTATE_GRIPPER_DEST");
            currentState = DO_ROTATE_GRIPPER_DEST;
            stateStartTime = millis();
        }
        break; 

    case DO_ROTATE_GRIPPER_DEST:
        Serial.println("[MC State] DO_ROTATE_GRIPPER_DEST: Commanding...");
        commandRotateGripper(targetRot2);
        currentState = DO_WAIT_GRIPPER_DEST;
        stateStartTime = millis();
        break;

    case DO_WAIT_GRIPPER_DEST:
        if (timeInState >= 450)
        {
            Serial.println("[MC State] DO_WAIT_GRIPPER_DEST: Done -> DO_MOVE_STEPPERS_TO_DEST");
            currentState = DO_MOVE_STEPPERS_TO_DEST;
            stateStartTime = millis();
        }
        break; 

    case DO_MOVE_STEPPERS_TO_DEST:

        Serial.println("[MC State] DO_MOVE_STEPPERS_TO_DEST: Commanding...");
        stepper1.setMaxSpeed(STEPPER_SPEED);
        stepper1.setAcceleration(STEPPER_ACCEL);
        stepper2.setMaxSpeed(STEPPER_SPEED);
        stepper2.setAcceleration(STEPPER_ACCEL);
        stepper3.setMaxSpeed(STEPPER_SPEED);
        stepper3.setAcceleration(STEPPER_ACCEL); // <<< ENSURE FOR ORB
        if (locType2 == LOC_CAPTURE)
        {
            stepper1.moveTo(targetCapt2); // Fixed target
            stepper1.enableOutputs();
            stepper3.moveTo(targetOrb2);
        }
        else
        {
            stepper1.stop();

            stepper2.moveTo(targetCart2);
            stepper3.moveTo(targetOrb2);
            stepper2.enableOutputs();
            stepper3.enableOutputs();
        }
        currentState = DO_WAIT_STEPPERS_DEST;
        stateStartTime = millis();
        break;

    case DO_WAIT_STEPPERS_DEST:
        if (currentState == DO_WAIT_STEPPERS_DEST)
        {
            bool cartDoneDest = (locType2 == LOC_CAPTURE) || (stepper2.distanceToGo() == 0);
            bool orbDoneDest = (locType2 == LOC_CAPTURE) || (stepper3.distanceToGo() == 0);
            bool captDoneDest = (locType2 == LOC_BOARD) || (stepper1.distanceToGo() == 0);

            if (cartDoneDest && orbDoneDest && captDoneDest)
            {
                Serial.println("[MC State] DO_WAIT_STEPPERS_DEST: Done -> DO_PERFORM_RELEASE_EXTEND");
                currentState = DO_PERFORM_RELEASE_EXTEND;
                stateStartTime = millis();
            }
        }
        break; 

    
    //------- REGULAR RELEASE SEQUENCE -------
    case DO_PERFORM_RELEASE_EXTEND:
        Serial.println("[MC State] DO_PERFORM_RELEASE_EXTEND: Commanding...");
        commandExtendActuator();
        currentState = DO_WAIT_RELEASE_EXTEND;
        stateStartTime = millis();
        break; 

    case DO_WAIT_RELEASE_EXTEND:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            Serial.println("[MC State] DO_WAIT_RELEASE_EXTEND: Done -> DO_PERFORM_RELEASE_OPEN");
            commandStopActuator();
            currentState = DO_PERFORM_RELEASE_OPEN;
            stateStartTime = millis();
        }
        break;

    case DO_PERFORM_RELEASE_OPEN:
        Serial.println("[MC State] DO_PERFORM_RELEASE_OPEN: Commanding...");
        commandGripperOpen();
        currentState = DO_WAIT_RELEASE_OPEN;
        stateStartTime = millis();
        break; 

    case DO_WAIT_RELEASE_OPEN:
        if (timeInState >= 300)
        {
            Serial.println("[MC State] DO_WAIT_RELEASE_OPEN: Done -> DO_PERFORM_RELEASE_RETRACT");
            currentState = DO_PERFORM_RELEASE_RETRACT;
            stateStartTime = millis();
        }
        break; 

    case DO_PERFORM_RELEASE_RETRACT: 
        Serial.println("[MC State] DO_PERFORM_RELEASE_RETRACT (Simple Timed): Commanding actuator retract...");
        commandRetractActuator();
        currentState = DO_WAIT_RELEASE_RETRACT; 
        stateStartTime = millis();
        break;

    case DO_WAIT_RELEASE_RETRACT: 
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50)
        {
            Serial.println("[MC State] DO_WAIT_RELEASE_RETRACT (Simple Timed): Done -> DO_COMPLETE");
            commandStopActuator();
            currentState = DO_COMPLETE;
            stateStartTime = millis();
        }
        break;


    case DO_COMPLETE:
        Serial.print("[MC State] DO_COMPLETE. Sub-sequence was: ");
        Serial.println(subSequenceIsActive ? "ACTIVE" : "INACTIVE");
        Serial.print("  Returning to state: ");
        Serial.println(stateToReturnToAfterSubSequence);

        if (subSequenceIsActive)
        {
            currentState = stateToReturnToAfterSubSequence;
            subSequenceIsActive = false;
            stateToReturnToAfterSubSequence = MOTION_IDLE;

        }
        else
        {
            currentState = MOTION_IDLE;
            lastMoveWasResetSubMoveFlag = false; 
            resetSubMoveFromCoords = {-1, -1};   
            resetSubMoveToCoords = {-1, -1};
        }
        stateStartTime = millis();
        break;

    case ERROR_STATE:
        Serial.println("!!! Motion Controller in ERROR STATE !!!");
    break;

    default:
        Serial.print("MotionController Error: Unknown state: ");
        Serial.println(currentState);
        currentState = ERROR_STATE;
        break;
    }

    previousState = currentState;
}
MotionState MotionController::getCurrentState() const
{
    return currentState;
}