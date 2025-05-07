// --- START OF FILE MotionController.cpp ---

#include "MotionController.h"
#include "hardware_pins.h" // Ensure pins are included

#include <ArduinoWebsockets.h> // <<< Include WebSocket library header

// Make the global client object from main.cpp accessible
extern websockets::WebsocketsClient client; 
// Make the global board object accessible
extern Board board; // <<<=== DECLARE The Global Board Object

MotionController::MotionController() :
    stepper1(AccelStepper::DRIVER, CAPTURE_STEP_PIN, CAPTURE_DIR_PIN),
    stepper2(AccelStepper::DRIVER, CART_STEP_PIN, CART_DIR_PIN),
    stepper3(AccelStepper::DRIVER, ORB_STEP_PIN, ORB_DIR_PIN),
    currentState(MOTION_IDLE),
    stateToReturnToAfterSubSequence(MOTION_IDLE), // Initialize
    subSequenceIsActive(false)                   // Initialize
{
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

    servo1.attach(ROTATION_SERVO_PIN);
    servo2.attach(GRIPPER_SERVO_PIN);
    servo1.write(GRIPPER_ROT_BOARD);
    servo2.write(GripperOpen);

    pinMode(ACTUATOR_IN1_PIN, OUTPUT);
    pinMode(ACTUATOR_IN2_PIN, OUTPUT);
    digitalWrite(ACTUATOR_IN1_PIN, LOW);
    digitalWrite(ACTUATOR_IN2_PIN, LOW);
    pinMode(ACTUATOR_RETRACTED_SENSE_PIN, INPUT); 

    pinMode(BUTTON_PIN_1, INPUT);
    pinMode(BUTTON_PIN_2, INPUT);

    currentState = MOTION_IDLE; // <<<=== Use Renamed State
    Serial.println("MotionController::setup() complete. Waiting for Homing/Commands.");
    initializeCaptureZone(); // Initialize capture zone (can be done before homing)
    startHomingSequence();
}

// ==========================================================================
// === PRIVATE HELPER FUNCTIONS (Define Before Use) =========================
// ==========================================================================
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
// --- Helper: Get Home Square --- <<<=== CORRECT DEFINITION SCOPE
std::pair<int, int> MotionController::getUnambiguousHomeSquare(PieceType type, PieceColor color) {
    if (type == PieceType::KING) {
        return (color == PieceColor::WHITE) ? std::make_pair(4, 7) : std::make_pair(4, 0);
    } else if (type == PieceType::QUEEN) {
        return (color == PieceColor::WHITE) ? std::make_pair(3, 7) : std::make_pair(3, 0);
    }
    // Return invalid coords for ambiguous pieces or unhandled types
    return {-1, -1};
}
// --- Start Board Reset Sequence ---
bool MotionController::startBoardResetSequence() {
    if (currentState != MOTION_IDLE) {
       Serial.println("MotionController Error: Cannot start reset, busy!");
       return false;
   }
   stateToReturnToAfterSubSequence = MOTION_IDLE; // Ensure clean start
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
    // Pre-populate slots 0 and 1 with Queens (assuming Slot 16 is home/0 steps)
    /*
    // Adjust slot indices and piece types as needed for your setup.
    // Slot indices here are arbitrary examples.
    int whiteQueenSlot = 0; // Example
    int blackQueenSlot = 1; // Example

    // Check if indices are valid before assigning
    if (whiteQueenSlot >= 0 && whiteQueenSlot < 32) {
         capture_zone[whiteQueenSlot].occupied = true;
         capture_zone[whiteQueenSlot].type = PieceType::QUEEN;
         capture_zone[whiteQueenSlot].color = PieceColor::WHITE;
         Serial.print("  Slot "); Serial.print(whiteQueenSlot); Serial.println(": White Queen (Promotion Spare)");
    }
     if (blackQueenSlot >= 0 && blackQueenSlot < 32 && blackQueenSlot != whiteQueenSlot) {
         capture_zone[blackQueenSlot].occupied = true;
         capture_zone[blackQueenSlot].type = PieceType::QUEEN;
         capture_zone[blackQueenSlot].color = PieceColor::BLACK;
         Serial.print("  Slot "); Serial.print(blackQueenSlot); Serial.println(": Black Queen (Promotion Spare)");
     }*/
    // All other slots start empty.
}

// --- Find Free Capture Slot ---
// Simple linear search for now. Returns index (0-31) or -1 if full.
int MotionController::findFreeCaptureSlot(PieceColor color) {
    // Start search from index 2 to avoid overwriting initial Queens unless necessary
    for (int i = 2; i < 32; ++i) {
        if (!capture_zone[i].occupied) {
            return i; // Found a free slot
        }
    }
    // If no slots from 2 onwards, check the initial Queen slots
    // Be careful here if you don't want to overwrite promotion Queens
    // For now, let's allow overwriting if absolutely necessary
    if (!capture_zone[0].occupied) return 0;
    if (!capture_zone[1].occupied) return 1;

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
// --- Coordinate/Location Helper Implementations ---
bool MotionController::getTargetsForSquare(String square, long &orbTarget, long &cartTarget) {
  square.trim();
  square.toLowerCase();
  if (square.length() != 2) { Serial.println("Error: Square format incorrect (e.g., 'e4')"); return false; }
  char fileChar = square.charAt(0); char rankChar = square.charAt(1);
  orbTarget = -1; cartTarget = -1;
  switch (fileChar) {
    case 'a': orbTarget = 4100; break; case 'b': orbTarget = 3280; break; case 'c': orbTarget = 2500; break;
    case 'd': orbTarget = 1700; break; case 'e': orbTarget = 900;  break; case 'f': orbTarget = 120;    break;
    case 'g': orbTarget = 5700; break; case 'h': orbTarget = 4950; break;
    default: Serial.print("Error: Invalid file: "); Serial.println(fileChar); return false;
  }
  switch (rankChar) {
    case '1': cartTarget = 4500; break; case '2': cartTarget = 3900; break; case '3': cartTarget = 3400; break;
    case '4': cartTarget = 2625; break; case '5': cartTarget = 2050; break; case '6': cartTarget = 1400; break;
    case '7': cartTarget = 725;  break; case '8': cartTarget = 0;  break;
    default: Serial.print("Error: Invalid rank: "); Serial.println(rankChar); return false;
  }
  return true;
}

bool MotionController::getTargetForCapture(int captureSlot, long &captureTarget) {
   switch (captureSlot) {
    case 1: captureTarget = 2780; break; case 2: captureTarget = 2600; break; case 3: captureTarget = 2420; break;
    case 4: captureTarget = 2180; break; case 5: captureTarget = 2060; break; case 6: captureTarget = 1880; break;
    case 7: captureTarget = 1700; break; case 8: captureTarget = 1520; break; case 9: captureTarget = 1300; break;
    case 10: captureTarget = 1130; break; case 11: captureTarget = 920; break; case 12: captureTarget = 720; break;
    case 13: captureTarget = 550; break; case 14: captureTarget = 380; break; case 15: captureTarget = 200; break;
    case 16: captureTarget = 0; break; case 17: captureTarget = 6250; break; case 18: captureTarget = 6050; break;
    case 19: captureTarget = 5850; break; case 20: captureTarget = 5700; break; case 21: captureTarget = 5500; break;
    case 22: captureTarget = 5300; break; case 23: captureTarget = 5150; break; case 24: captureTarget = 4950; break;
    case 25: captureTarget = 4750; break; case 26: captureTarget = 4580; break; case 27: captureTarget = 4380; break;
    case 28: captureTarget = 4210; break; case 29: captureTarget = 4020; break; case 30: captureTarget = 3840; break;
    case 31: captureTarget = 3650; break; case 32: captureTarget = 3480; break;
    default: Serial.print("Error: Invalid capture slot: "); Serial.println(captureSlot); return false;
  }
  return true;
}

// <<<=== Definition of parseLocation moved before its first call
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
            gripperRotTarget = GRIPPER_ROT_CAPTURE; return LOC_CAPTURE;
        } else { return LOC_INVALID; }
    } else if (locStr.length() == 2) {
        long specificOrbTarget, specificCartTarget;
        if (getTargetsForSquare(locStr, specificOrbTarget, specificCartTarget)) {
            orbTarget = specificOrbTarget; cartTarget = specificCartTarget;
            gripperRotTarget = GRIPPER_ROT_BOARD; return LOC_BOARD;
        } else { return LOC_INVALID; }
    } else {
        Serial.print("Error: Invalid loc format: '"); Serial.print(locStr); Serial.println("'");
        return LOC_INVALID;
    }
}

// --- Safety Check Implementations ---
// <<<=== Definitions moved before executeStateMachine
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
               if (client.available()) { // Check if client is connected
                  client.poll(); // Process incoming WebSocket messages (like PING)
                  // Serial.println("[Safety Home Poll]"); // Optional debug
               }
               lastPollTime = millis(); // Reset poll timer
            }
            // -----------------------------------------
  
            delay(1); // Yield CPU briefly (important!)
          }
        } // --- End While Loop ---
  
        if (!homed) {
          Serial.println("!!! SAFETY HOME FAILED (Timeout) !!!");
          stepper1.stop();
        }
        stepper1.setMaxSpeed(o_sp); stepper1.setAcceleration(o_ac);
        stepper1.runToPosition(); // Ensure it settles at the final position (0 or stopped position)
        Serial.println(homed ? "!!! CRITICAL OK !!! Capture homed." : "!!! CRITICAL FAILED !!!");
      }
      // else { Serial.println("[Safety Info] Capture already home. OK."); } // Optional info
    }
    // else { Serial.println("[Safety Info] Cart target above threshold. OK."); } // Optional info
  }
// --- Low-level Action Implementations ---
// <<<=== Definitions moved before executeStateMachine
void MotionController::commandExtendActuator() { digitalWrite(ACTUATOR_IN1_PIN, LOW); digitalWrite(ACTUATOR_IN2_PIN, HIGH); }
void MotionController::commandRetractActuator() { digitalWrite(ACTUATOR_IN1_PIN, HIGH); digitalWrite(ACTUATOR_IN2_PIN, LOW); }
void MotionController::commandStopActuator() { digitalWrite(ACTUATOR_IN1_PIN, LOW); digitalWrite(ACTUATOR_IN2_PIN, LOW); }
void MotionController::commandGripperOpen() { servo2.write(GripperOpen); }
void MotionController::commandGripperClose() { servo2.write(GripperClose); }
void MotionController::commandRotateGripper(int angle) { servo1.write(constrain(angle, 0, 180)); }
String MotionController::coordsToAlgebraic(int x, int y) {
    if (x < 0 || x > 7 || y < 0 || y > 7) return "??";
    char file = 'a' + x; char rank = '8' - y; String r = ""; r += file; r += rank; return r;
}

// ==========================================================================
// === PUBLIC FUNCTIONS =====================================================
// ==========================================================================

void MotionController::update() {
    stepper1.run(); stepper2.run(); stepper3.run();
    executeStateMachine();
    // Handle manual input later
}

bool MotionController::isBusy() {
    return currentState != MOTION_IDLE; // <<<=== Use Renamed State
}

bool MotionController::startMoveSequence(String fromLoc, String toLoc, bool isSubSequenceCall) {
    // If it's NOT a sub-sequence call, then we must be IDLE to start.
    // If it IS a sub-sequence call, the calling state is responsible for ensuring it's safe to start.
    if (!isSubSequenceCall && currentState != MOTION_IDLE) {
        Serial.println("MotionController Error: Busy (called externally when not IDLE)!");
        return false;
    }
    // If it IS a sub-sequence call, we might be in a RESET_... state.
    // The important thing is that we are not ALREADY in a DO_... sequence or another conflicting sub-sequence.
    // A simple check for now: if subSequenceIsActive is already true, we can't start another one.
    if (subSequenceIsActive && isSubSequenceCall) {
        Serial.println("MotionController Error: Cannot start a new sub-sequence while another is active!");
        return false; // Should not happen if parent sequence logic is correct
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

void MotionController::executeStateMachine() {
    unsigned long timeInState = millis() - stateStartTime;

    switch (currentState) {
        case MOTION_IDLE: // <<<=== Use Renamed State
            // Do nothing
            break; // <<<=== Added break
        // =================== HOMING SEQUENCE ===================
        case HOMING_START:
            Serial.println("[MC State] HOMING_START -> HOMING_CAPTURE_START_MOVE");
            // Prepare for Phase 1: Home Capture Stepper
            currentState = HOMING_CAPTURE_START_MOVE;
            stateStartTime = millis();
            // break; // Fall through okay

        case HOMING_CAPTURE_START_MOVE:
            Serial.println("[MC State] HOMING_CAPTURE_START_MOVE: Commanding Capture home move...");
            stepper1.setMaxSpeed(abs(HOMING_SPEED_CAPTURE)); // Use abs() for maxSpeed
            stepper1.setAcceleration(HOMING_ACCEL);
            stepper1.enableOutputs();
            // Move towards the endstop (negative direction assumed)
            stepper1.move(-30000); // Large distance
            currentState = HOMING_CAPTURE_WAIT_HIT;
            stateStartTime = millis();
            break; // Wait for it to hit

        case HOMING_CAPTURE_WAIT_HIT:
            if (digitalRead(ENDSTOP_CAPTURE_PIN) == LOW) { // Assuming Active LOW endstop
                Serial.println("[MC State] HOMING_CAPTURE_WAIT_HIT: Endstop HIT!");
                stepper1.stop(); // Stop motor immediately
                stepper1.setCurrentPosition(0); // Define this as home
                homeCaptureHomed = true; // Mark as homed

                // Restore normal speed/accel for Capture stepper
                stepper1.setMaxSpeed(STEPPER_SPEED);
                stepper1.setAcceleration(STEPPER_ACCEL);

                // Optional: Add backoff sequence here if needed
                // currentState = HOMING_CAPTURE_BACKOFF_MOVE;

                // If no backoff, proceed to next phase
                Serial.println("  Capture Homing Complete -> HOMING_CART_ORB_START_MOVE");
                currentState = HOMING_CART_ORB_START_MOVE;
                stateStartTime = millis();

            } else if (timeInState > 20000) { // Timeout safety
                 Serial.println("!!! ERROR: Homing Capture Timeout !!!");
                 stepper1.stop();
                 // Restore speed/accel anyway
                 stepper1.setMaxSpeed(STEPPER_SPEED);
                 stepper1.setAcceleration(STEPPER_ACCEL);
                 currentState = ERROR_STATE; // Or try to continue? Risky.
                 stateStartTime = millis();
            }
            // else: Still moving, keep waiting in this state
            break;

        // TODO: Implement Optional Backoff States for Capture if desired

        case HOMING_CART_ORB_START_MOVE:
            Serial.println("[MC State] HOMING_CART_ORB_START_MOVE: Commanding Cart & Orb home moves...");
             // Prepare for Phase 2: Home Cart & Orb Steppers Simultaneously
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
            if (!homeCartHomed && digitalRead(ENDSTOP_CART_PIN) == LOW) {
                 Serial.println("  Cart Endstop HIT!");
                 stepper2.stop();
                 stepper2.setCurrentPosition(0);
                 homeCartHomed = true;
                 // Restore speed/accel
                 stepper2.setMaxSpeed(STEPPER_SPEED);
                 stepper2.setAcceleration(STEPPER_ACCEL);
                 // Optional: Trigger Cart backoff state
            }

            // Check Orb Endstop
            if (!homeOrbHomed && digitalRead(ENDSTOP_ORB_PIN) == LOW) {
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
            if (homeCartHomed && homeOrbHomed) {
                 Serial.println("[MC State] HOMING_CART_ORB_WAIT_HIT: Both homed -> HOMING_COMPLETE");
                 currentState = HOMING_COMPLETE;
                 stateStartTime = millis();
            } else if (timeInState > 20000) { // Timeout safety for this phase
                 Serial.println("!!! ERROR: Homing Cart/Orb Timeout !!!");
                 if (!homeCartHomed) stepper2.stop();
                 if (!homeOrbHomed) stepper3.stop();
                 // Restore speeds anyway
                 stepper2.setMaxSpeed(STEPPER_SPEED); stepper2.setAcceleration(STEPPER_ACCEL);
                 stepper3.setMaxSpeed(STEPPER_SPEED); stepper3.setAcceleration(STEPPER_ACCEL);
                 currentState = ERROR_STATE; // Go to error state
                 stateStartTime = millis();
            }
             // else: Still waiting for one or both
            break;

         // TODO: Implement Optional Backoff States for Cart/Orb if desired

         case HOMING_COMPLETE:
         Serial.println("[MC State] HOMING_COMPLETE: Homing Finished -> MOTION_IDLE");
         currentState = MOTION_IDLE;
         stateStartTime = millis();
            break;
           // =================== FULL BOARD RESET SEQUENCE ==================
        case RESET_START:
        Serial.println("[MC Reset] RESET_START -> Phase 1: Clear Board");
        resetBoardIterator = 0; // Start iterating board from square 0
        currentState = RESET_P1_ITERATE_BOARD;
        stateStartTime = millis();
        break;

    // === Phase 1: Clear Board (Non K/Q) to Capture Zone ===
    case RESET_P1_ITERATE_BOARD: {
        reset_currentBoardCoords.first = resetBoardIterator % 8;  // x
        reset_currentBoardCoords.second = resetBoardIterator / 8; // y
        reset_currentBoardAlg = coordsToAlgebraic(reset_currentBoardCoords.first, reset_currentBoardCoords.second);

        Serial.print("[MC Reset P1] ITERATE_BOARD: Square "); Serial.print(resetBoardIterator);
        Serial.print(" ("); Serial.print(reset_currentBoardAlg); Serial.println(")");

        if (resetBoardIterator >= 64) {
            Serial.println("  Phase 1 (Clear Board) Complete -> RESET_P2_START");
            currentState = RESET_P2_START;
            resetCZIterator = 0; // Initialize for Phase 2
            stateStartTime = millis();
        } else {
            currentState = RESET_P1_CHECK_SQUARE; // Check the current square
            stateStartTime = millis();
        }
    }
        break;

        case RESET_P1_CHECK_SQUARE:{
        Piece* pieceOnSquare = board.grid[reset_currentBoardCoords.first][reset_currentBoardCoords.second];
        bool pieceToClear = false; // Reset flag for current square

        if (pieceOnSquare != nullptr) {
            PieceType type = pieceOnSquare->getType();
            PieceColor color = pieceOnSquare->getColor();

            if (type != PieceType::KING && type != PieceType::QUEEN) {
                // It's a Pawn, Rook, Knight, or Bishop
                std::vector<std::pair<int, int>> potentialHomes = getPotentialHomeSquares(type, color);
                bool isAlreadyHome = false;

                // Check if the piece is ALREADY on one of its valid home squares
                for (const auto& home : potentialHomes) {
                    if (reset_currentBoardCoords.first == home.first && reset_currentBoardCoords.second == home.second) {
                        isAlreadyHome = true;
                        Serial.print("  Piece "); Serial.print(pieceOnSquare->getSymbol());
                        Serial.print(" at "); Serial.print(reset_currentBoardAlg);
                        Serial.println(" is already on a valid home square. Skipping clear.");
                        break;
                    }
                }

                if (!isAlreadyHome) {
                    Serial.print("  Found piece to clear: "); Serial.print(pieceOnSquare->getSymbol());
                    Serial.print(" at "); Serial.println(reset_currentBoardAlg);
                    reset_pieceBeingMoved.type = type;
                    reset_pieceBeingMoved.color = color;
                    reset_pieceBeingMoved.occupied = true;
                    pieceToClear = true;
                }
                // If isAlreadyHome is true, pieceToClear remains false
            } else {
                Serial.print("  Found King/Queen at "); Serial.print(reset_currentBoardAlg); Serial.println(", skipping in Phase 1 (Clear Board).");
            }
        } else {
            Serial.print("  Square "); Serial.print(reset_currentBoardAlg); Serial.println(" is empty.");
        }

        if (pieceToClear) { // Only proceed if piece needs clearing
            currentState = RESET_P1_MOVE_TO_GRAB_PIECE;
        } else {
            resetBoardIterator++; // Move to next square
            currentState = RESET_P1_ITERATE_BOARD;
        }
        stateStartTime = millis();
    }
        break;

    case RESET_P1_MOVE_TO_GRAB_PIECE:
        Serial.print("[MC Reset P1] MOVE_TO_GRAB_PIECE: Moving to "); Serial.println(reset_currentBoardAlg);
        getTargetsForSquare(reset_currentBoardAlg, targetOrb1, targetCart1); // Use target1 vars
        enforceCaptureHomedForLowCart(targetCart1); // Safety
        enforceCartSafetyRotation(targetCart1);   // Safety
        stepper1.moveTo(0); // Capture motor home
        stepper2.moveTo(targetCart1);
        stepper3.moveTo(targetOrb1);
        stepper1.enableOutputs(); stepper2.enableOutputs(); stepper3.enableOutputs();
        currentState = RESET_P1_WAIT_GRAB_PIECE_POS; stateStartTime = millis();
        break;

    case RESET_P1_WAIT_GRAB_PIECE_POS:
        if (stepper1.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepper3.distanceToGo()==0) {
            Serial.println("  Arrived at piece -> RESET_P1_ROTATE_FOR_GRAB");
            currentState = RESET_P1_ROTATE_FOR_GRAB; stateStartTime = millis();
        }
        break;

    case RESET_P1_ROTATE_FOR_GRAB:
        Serial.println("[MC Reset P1] ROTATE_FOR_GRAB");
        commandRotateGripper(GRIPPER_ROT_BOARD); // Should be board angle
        currentState = RESET_P1_WAIT_ROTATE_GRAB; stateStartTime = millis();
        break;

    case RESET_P1_WAIT_ROTATE_GRAB:
        if (timeInState >= 450) {
            Serial.println("  Rotated -> RESET_P1_PERFORM_GRAB_EXTEND");
            currentState = RESET_P1_PERFORM_GRAB_EXTEND; stateStartTime = millis();
        }
        break;

    // --- "Take" sub-sequence for P1 (uses sensor check for retract) ---
    case RESET_P1_PERFORM_GRAB_EXTEND:
         Serial.println("[MC Reset P1] PERFORM_GRAB_EXTEND");
         commandGripperOpen(); commandExtendActuator();
         currentState = RESET_P1_WAIT_GRAB_EXTEND; stateStartTime = millis(); break;
    case RESET_P1_WAIT_GRAB_EXTEND:
         if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
             commandStopActuator(); currentState = RESET_P1_PERFORM_GRAB_CLOSE; stateStartTime = millis();
         } break;
    case RESET_P1_PERFORM_GRAB_CLOSE:
         Serial.println("[MC Reset P1] PERFORM_GRAB_CLOSE");
         commandGripperClose();
         currentState = RESET_P1_WAIT_GRAB_CLOSE; stateStartTime = millis(); break;
    case RESET_P1_WAIT_GRAB_CLOSE:
         if (timeInState >= 300) {
             // Transition to the "initial retract" state from the robust retract logic
             currentState = DO_PERFORM_TAKE_RETRACT_INITIAL; // <<< REUSE EXISTING ROBUST RETRACT
             // Modify what happens on success/failure from that reused sequence:
             // Instead of DO_TAKE_RETRACT_CONFIRMED -> DO_SAFETY_CHECKS_DEST
             // we need DO_TAKE_RETRACT_CONFIRMED -> RESET_P1_GRAB_CONFIRMED
             // and DO_TAKE_RETRACT_FAILED -> RESET_P1_GRAB_FAILED
             // This requires modifying the reused states or duplicating them.
             // For now, let's duplicate the core retract logic for clarity in reset.
             Serial.println("  Grab done -> RESET_P1_PERFORM_GRAB_RETRACT (Start retract check)");
             currentState = RESET_P1_PERFORM_GRAB_RETRACT; // Go to dedicated reset retract
             stateStartTime = millis();
         } break;

    // --- Dedicated Retract with Sensor Check for Reset_P1 ---
    case RESET_P1_PERFORM_GRAB_RETRACT: // Initial timed retract
        Serial.println("[MC Reset P1] PERFORM_GRAB_RETRACT (Initial)");
        commandRetractActuator();
        retractRetryCount = 0;
        currentState = RESET_P1_WAIT_GRAB_RETRACT; // Wait for timed part
        stateStartTime = millis();
        break;

    case RESET_P1_WAIT_GRAB_RETRACT: // Timed wait and then sensor check
        // This state will now contain the logic from the old DO_WAIT_TAKE_RETRACT_INITIAL and DO_CHECK_TAKE_RETRACT_SENSOR
        // We'll simplify and assume one retry path through rotation.
        if (retractRetryCount == 0 && timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150) { // Initial timed retract done
             Serial.print("  Initial P1 retract time elapsed. Checking sensor... ");
             if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH) { // Active  HIGH
                 Serial.println("Sensor Active! -> RESET_P1_GRAB_CONFIRMED");
                 commandStopActuator();
                 currentState = RESET_P1_GRAB_CONFIRMED;
             } else {
                 Serial.println("Sensor Inactive. -> Attempting P1 recovery rotate.");
                 retractRetryCount = 1; // Mark first retry attempt
                 commandStopActuator(); // Stop retract before rotating
                 commandRotateGripper(constrain(servo1.read() + 5, 0, 180)); // Rotate +5
                 // Need a wait state for this rotation, then retry retract.
                 // Let's add a temporary delay here and then go back to RETRACT state
                 // This is not ideal non-blocking but simplifies for now.
                 delay(450);
                 Serial.println("  P1 Recovery rotate done. Retrying retract...");
                 commandRetractActuator(); // Start retract again
                 // Stay in RESET_P1_WAIT_GRAB_RETRACT but with retryCount = 1
             }
             stateStartTime = millis(); // Reset timer for next phase of this state
        } else if (retractRetryCount == 1 && timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150) { // After retry retract
             Serial.print("  P1 Retry retract time elapsed. Checking sensor... ");
             if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH) { // Active  HIGH
                 Serial.println("Sensor Active after retry! -> RESET_P1_GRAB_CONFIRMED");
                 commandStopActuator();
                 currentState = RESET_P1_GRAB_CONFIRMED;
             } else {
                 Serial.println("Sensor STILL Inactive after P1 retry! -> RESET_P1_GRAB_FAILED");
                 commandStopActuator();
                 currentState = RESET_P1_GRAB_FAILED;
             }
             stateStartTime = millis();
        }
        break;


    case RESET_P1_GRAB_CONFIRMED:
        Serial.println("[MC Reset P1] GRAB_CONFIRMED: Piece taken from board -> RESET_P1_ROTATE_AWAY_BOARD");
        currentState = RESET_P1_ROTATE_AWAY_BOARD; stateStartTime = millis();
        break;

    case RESET_P1_GRAB_FAILED:
        Serial.println("!!! [MC Reset P1] GRAB_FAILED: Could not confirm retract. Skipping piece.");
        // Problem: We might have the piece but sensor failed. Risk leaving it.
        // Or, we couldn't pick it up.
        // For now, skip this piece and move to the next.
        resetBoardIterator++; // Move to next square
        currentState = RESET_P1_ITERATE_BOARD; stateStartTime = millis();
        break;

    case RESET_P1_ROTATE_AWAY_BOARD:
        Serial.println("[MC Reset P1] ROTATE_AWAY_BOARD");
        commandRotateGripper(GRIPPER_ROT_BOARD); // Ensure safe rotation
        currentState = RESET_P1_WAIT_ROTATE_AWAY; stateStartTime = millis();
        break;

    case RESET_P1_WAIT_ROTATE_AWAY:
        if (timeInState >= 450) {
            Serial.println("  Rotated -> RESET_P1_MOVE_TO_CZ_DROPOFF");
            currentState = RESET_P1_MOVE_TO_CZ_DROPOFF; stateStartTime = millis();
        }
        break;

    case RESET_P1_MOVE_TO_CZ_DROPOFF:
        Serial.println("[MC Reset P1] MOVE_TO_CZ_DROPOFF");
        enforceCaptureHomedForLowCart(CART_CAPTURE_POS); // Safety for Cart
        // enforceCartSafetyRotation(CART_CAPTURE_POS); // Gripper should be at 0
        stepper1.moveTo(0); // Capture motor stays home
        stepper2.moveTo(CART_CAPTURE_POS);
        stepper3.moveTo(stepper3.currentPosition()); // Orb stays
        stepper2.enableOutputs();
        currentState = RESET_P1_WAIT_CZ_DROPOFF; stateStartTime = millis();
        break;

    case RESET_P1_WAIT_CZ_DROPOFF:
        if (stepper2.distanceToGo() == 0) {
            Serial.println("  Arrived at CZ dropoff -> RESET_P1_FIND_CZ_SLOT");
            currentState = RESET_P1_FIND_CZ_SLOT; stateStartTime = millis();
        }
        break;

    case RESET_P1_FIND_CZ_SLOT:
        Serial.println("[MC Reset P1] FIND_CZ_SLOT");
        reset_targetCZSlotIndex = findFreeCaptureSlot(reset_pieceBeingMoved.color);
        if (reset_targetCZSlotIndex == -1) {
            Serial.println("!!! ERROR: CZ Full during P1 reset! Skipping piece drop.");
            // What to do with held piece? Maybe drop it at current Cart pos?
            // For now, just skip placing it and move to next board square.
            resetBoardIterator++;
            currentState = RESET_P1_ITERATE_BOARD;
        } else {
            if (getTargetForCapture(reset_targetCZSlotIndex + 1, reset_targetCZSlotPos)) {
                Serial.print("  Found free CZ slot: "); Serial.print(reset_targetCZSlotIndex); Serial.print(" (Pos: "); Serial.print(reset_targetCZSlotPos); Serial.println(")");
                currentState = RESET_P1_MOVE_CZ_MOTOR_TO_SLOT;
            } else {
                 Serial.println("!!! ERROR: Failed to get pos for CZ slot! Skipping.");
                 resetBoardIterator++; currentState = RESET_P1_ITERATE_BOARD;
            }
        }
        stateStartTime = millis();
        break;

    case RESET_P1_MOVE_CZ_MOTOR_TO_SLOT:
        Serial.println("[MC Reset P1] MOVE_CZ_MOTOR_TO_SLOT");
        stepper1.moveTo(reset_targetCZSlotPos);
        stepper1.enableOutputs();
        currentState = RESET_P1_WAIT_CZ_MOTOR; stateStartTime = millis();
        break;

    case RESET_P1_WAIT_CZ_MOTOR:
        if (stepper1.distanceToGo() == 0) {
            Serial.println("  CZ Motor at slot -> RESET_P1_ROTATE_FOR_CZ_RELEASE");
            currentState = RESET_P1_ROTATE_FOR_CZ_RELEASE; stateStartTime = millis();
        }
        break;

    case RESET_P1_ROTATE_FOR_CZ_RELEASE:
        Serial.println("[MC Reset P1] ROTATE_FOR_CZ_RELEASE");
        commandRotateGripper(GRIPPER_ROT_CAPTURE);
        currentState = RESET_P1_WAIT_ROTATE_CZ_RELEASE; stateStartTime = millis();
        break;

    case RESET_P1_WAIT_ROTATE_CZ_RELEASE:
        if (timeInState >= 450) {
            Serial.println("  Rotated for CZ -> RESET_P1_PERFORM_RELEASE_EXTEND");
            currentState = RESET_P1_PERFORM_RELEASE_EXTEND; stateStartTime = millis();
        }
        break;

    // --- "Release" sub-sequence for P1 (simple timed retract) ---
    case RESET_P1_PERFORM_RELEASE_EXTEND:
         Serial.println("[MC Reset P1] PERFORM_RELEASE_EXTEND");
         commandExtendActuator(); currentState = RESET_P1_WAIT_RELEASE_EXTEND; stateStartTime = millis(); break;
    case RESET_P1_WAIT_RELEASE_EXTEND:
         if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
             commandStopActuator(); currentState = RESET_P1_PERFORM_RELEASE_OPEN; stateStartTime = millis();
         } break;
    case RESET_P1_PERFORM_RELEASE_OPEN:
          Serial.println("[MC Reset P1] PERFORM_RELEASE_OPEN");
          commandGripperOpen(); currentState = RESET_P1_WAIT_RELEASE_OPEN; stateStartTime = millis(); break;
    case RESET_P1_WAIT_RELEASE_OPEN:
         if (timeInState >= 300) { // Gripper open time
             currentState = RESET_P1_PERFORM_RELEASE_RETRACT; stateStartTime = millis();
         } break;
    case RESET_P1_PERFORM_RELEASE_RETRACT: // Simple timed retract for release
         Serial.println("[MC Reset P1] PERFORM_RELEASE_RETRACT (Timed)");
         commandRetractActuator(); currentState = RESET_P1_WAIT_RELEASE_RETRACT; stateStartTime = millis(); break;
     case RESET_P1_WAIT_RELEASE_RETRACT:
         if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
             commandStopActuator();
             Serial.println("  Release in CZ done -> RESET_P1_UPDATE_CZ_ARRAY");
             currentState = RESET_P1_UPDATE_CZ_ARRAY; stateStartTime = millis();
         } break;

    case RESET_P1_UPDATE_CZ_ARRAY:
        Serial.print("[MC Reset P1] UPDATE_CZ_ARRAY: Marking CZ slot "); Serial.print(reset_targetCZSlotIndex); Serial.println(" occupied.");
        if (reset_targetCZSlotIndex != -1) { // Should be valid if we got here
             capture_zone[reset_targetCZSlotIndex] = reset_pieceBeingMoved; // Store the piece info
             // capture_zone[reset_targetCZSlotIndex].occupied = true; // Already set in reset_pieceBeingMoved
        }
        // Rotate gripper back before moving away
        commandRotateGripper(GRIPPER_ROT_BOARD);
        delay(450); // TODO: Non-blocking wait
        currentState = RESET_P1_PIECE_CLEARED; stateStartTime = millis();
        break;

    case RESET_P1_PIECE_CLEARED:
        Serial.println("[MC Reset P1] PIECE_CLEARED: One piece moved from board to CZ.");
        resetBoardIterator++; // Move to next board square
        currentState = RESET_P1_ITERATE_BOARD; stateStartTime = millis();
        break;


       // === Phase 2: Place K/Q from Capture Zone to Home ===
       case RESET_P2_START:
       Serial.println("[MC Reset P2] START: Placing K/Q from CZ to Home -> RESET_P2_ITERATE_CZ");
       resetCZIterator = 0; // Start iterating CZ from slot 0
       currentState = RESET_P2_ITERATE_CZ;
       stateStartTime = millis();
       break;

   case RESET_P2_ITERATE_CZ:
       Serial.print("[MC Reset P2] ITERATE_CZ: Slot "); Serial.println(resetCZIterator);
       if (resetCZIterator >= 32) { // Done checking all CZ slots
           Serial.println("  Phase 2 (Place K/Q from CZ) Complete -> RESET_P3_START");
           currentState = RESET_P3_START; // Move to next phase
           resetBoardIterator = 0; // Reset board iterator for P3
           stateStartTime = millis();
       } else {
           currentState = RESET_P2_CHECK_CZ_PIECE; // Check current slot
           stateStartTime = millis();
       }
       break;

   case RESET_P2_CHECK_CZ_PIECE: {
       bool pieceToPlace = false;
       if (capture_zone[resetCZIterator].occupied) {
           PieceType type = capture_zone[resetCZIterator].type;
           PieceColor color = capture_zone[resetCZIterator].color;
           if (type == PieceType::KING || type == PieceType::QUEEN) {
               reset_targetHomeSquareCoords = getUnambiguousHomeSquare(type, color);
               if (reset_targetHomeSquareCoords.first != -1) {
                   // Check if the logical home square is empty
                   if (board.grid[reset_targetHomeSquareCoords.first][reset_targetHomeSquareCoords.second] == nullptr) {
                       reset_targetHomeSquareAlg = coordsToAlgebraic(reset_targetHomeSquareCoords.first, reset_targetHomeSquareCoords.second);
                       Serial.print("  Found K/Q in CZ slot "); Serial.print(resetCZIterator);
                       Serial.print(" for empty home "); Serial.print(reset_targetHomeSquareAlg); Serial.println(". Preparing move.");
                       // Store piece info for move
                       reset_pieceBeingMoved = capture_zone[resetCZIterator];
                       pieceToPlace = true;
                   } else {
                       Serial.print("  K/Q in CZ slot "); Serial.print(resetCZIterator);
                       Serial.print(", but home "); Serial.print(coordsToAlgebraic(reset_targetHomeSquareCoords.first, reset_targetHomeSquareCoords.second));
                       Serial.println(" is logically occupied. Skipping.");
                   }
               }
           } else { Serial.print("  Piece in CZ slot "); Serial.print(resetCZIterator); Serial.println(" is not K/Q. Skipping."); }
       } else { Serial.print("  CZ slot "); Serial.print(resetCZIterator); Serial.println(" is empty. Skipping."); }

       if (pieceToPlace) {
           // Get stepper pos for current capture slot
           if (getTargetForCapture(resetCZIterator + 1, reset_targetCZSlotPos)) { // Use reset_targetCZSlotPos
                Serial.print("  Moving to grab from CZ slot pos: "); Serial.println(reset_targetCZSlotPos);
                currentState = RESET_P2_MOVE_TO_CZ_SLOT;
           } else {
                Serial.println("!!! ERROR: Could not get stepper pos for CZ slot! Skipping piece.");
                resetCZIterator++; currentState = RESET_P2_ITERATE_CZ;
           }
       } else {
           resetCZIterator++; currentState = RESET_P2_ITERATE_CZ;
       }
       stateStartTime = millis();
    }
       break;

   case RESET_P2_MOVE_TO_CZ_SLOT:
       Serial.println("[MC Reset P2] MOVE_TO_CZ_SLOT");
       enforceCaptureHomedForLowCart(CART_CAPTURE_POS); // Cart goes to CZ interaction pos
       // Gripper should be at board angle 0
       stepper2.moveTo(CART_CAPTURE_POS);
       stepper1.moveTo(reset_targetCZSlotPos); // Capture motor to slot
       stepper3.moveTo(stepper3.currentPosition()); // Orb stays
       stepper1.enableOutputs(); stepper2.enableOutputs();
       currentState = RESET_P2_WAIT_CZ_SLOT_POS; stateStartTime = millis();
       break;

   case RESET_P2_WAIT_CZ_SLOT_POS:
       if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
           Serial.println("  Arrived at CZ slot -> RESET_P2_ROTATE_FOR_CZ_GRAB");
           currentState = RESET_P2_ROTATE_FOR_CZ_GRAB; stateStartTime = millis();
       }
       break;

   case RESET_P2_ROTATE_FOR_CZ_GRAB:
       Serial.println("[MC Reset P2] ROTATE_FOR_CZ_GRAB");
       commandRotateGripper(GRIPPER_ROT_CAPTURE);
       currentState = RESET_P2_WAIT_ROTATE_CZ_GRAB; stateStartTime = millis();
       break;

   case RESET_P2_WAIT_ROTATE_CZ_GRAB:
       if (timeInState >= 450) {
           Serial.println("  Rotated -> RESET_P2_PERFORM_CZ_GRAB_EXTEND");
           currentState = RESET_P2_PERFORM_CZ_GRAB_EXTEND; stateStartTime = millis();
       }
       break;

   // --- "Take" sub-sequence from CZ (uses sensor check for retract) ---
   case RESET_P2_PERFORM_CZ_GRAB_EXTEND:
        Serial.println("[MC Reset P2] PERFORM_CZ_GRAB_EXTEND");
        commandGripperOpen(); commandExtendActuator();
        currentState = RESET_P2_WAIT_CZ_GRAB_EXTEND; stateStartTime = millis(); break;
   case RESET_P2_WAIT_CZ_GRAB_EXTEND:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
            commandStopActuator(); currentState = RESET_P2_PERFORM_CZ_GRAB_CLOSE; stateStartTime = millis();
        } break;
   case RESET_P2_PERFORM_CZ_GRAB_CLOSE:
        Serial.println("[MC Reset P2] PERFORM_CZ_GRAB_CLOSE");
        commandGripperClose();
        currentState = RESET_P2_WAIT_CZ_GRAB_CLOSE; stateStartTime = millis(); break;
   case RESET_P2_WAIT_CZ_GRAB_CLOSE:
        if (timeInState >= 300) {
            // Use the same robust retract logic as in Phase 1.
            // We need states to return to after the shared retract logic.
            // For simplicity, duplicate the retract logic again or make it a true sub-state machine.
            // Let's duplicate the core retract sequence, redirecting outcomes.
            Serial.println("  Grab done -> RESET_P2_PERFORM_CZ_GRAB_RETRACT (Start retract check)");
            currentState = RESET_P2_PERFORM_CZ_GRAB_RETRACT; // Go to dedicated P2 retract
            stateStartTime = millis();
        } break;

   // --- Dedicated Retract with Sensor Check for Reset_P2 ---
   case RESET_P2_PERFORM_CZ_GRAB_RETRACT: // Initial timed retract
       Serial.println("[MC Reset P2] PERFORM_CZ_GRAB_RETRACT (Initial)");
       commandRetractActuator();
       retractRetryCount = 0;
       currentState = RESET_P2_WAIT_CZ_GRAB_RETRACT; // Wait for timed part
       stateStartTime = millis();
       break;

   case RESET_P2_WAIT_CZ_GRAB_RETRACT:
       if (retractRetryCount == 0 && timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150) {
            Serial.print("  Initial P2 retract time elapsed. Checking sensor... ");
            if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH) { // Active  HIGH
                Serial.println("Sensor Active! -> RESET_P2_CZ_GRAB_CONFIRMED");
                commandStopActuator();
                currentState = RESET_P2_CZ_GRAB_CONFIRMED;
            } else {
                Serial.println("Sensor Inactive. -> Attempting P2 recovery rotate.");
                retractRetryCount = 1;
                commandStopActuator();
                commandRotateGripper(constrain(servo1.read() - 5, 0, 180)); // Rotate -5 (other direction)
                delay(450); // TODO: Non-blocking wait state
                Serial.println("  P2 Recovery rotate done. Retrying retract...");
                commandRetractActuator();
            }
            stateStartTime = millis();
       } else if (retractRetryCount == 1 && timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150) {
            Serial.print("  P2 Retry retract time elapsed. Checking sensor... ");
            if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH) {
                Serial.println("Sensor Active after retry! -> RESET_P2_CZ_GRAB_CONFIRMED");
                commandStopActuator();
                currentState = RESET_P2_CZ_GRAB_CONFIRMED;
            } else {
                Serial.println("Sensor STILL Inactive after P2 retry! -> RESET_P2_CZ_GRAB_FAILED");
                commandStopActuator();
                currentState = RESET_P2_CZ_GRAB_FAILED;
            }
            stateStartTime = millis();
       }
       break;

   case RESET_P2_CZ_GRAB_CONFIRMED:
       Serial.println("[MC Reset P2] CZ_GRAB_CONFIRMED: Piece taken from CZ -> RESET_P2_UPDATE_CZ_ARRAY_FREE");
       currentState = RESET_P2_UPDATE_CZ_ARRAY_FREE; stateStartTime = millis();
       break;

   case RESET_P2_CZ_GRAB_FAILED:
       Serial.println("!!! [MC Reset P2] CZ_GRAB_FAILED: Could not confirm retract. Skipping CZ piece.");
       resetCZIterator++; // Move to next CZ slot
       currentState = RESET_P2_ITERATE_CZ; stateStartTime = millis();
       break;

   case RESET_P2_UPDATE_CZ_ARRAY_FREE:
       Serial.print("[MC Reset P2] UPDATE_CZ_ARRAY_FREE: Marking CZ slot "); Serial.print(resetCZIterator); Serial.println(" free.");
       if (resetCZIterator >= 0 && resetCZIterator < 32) {
           capture_zone[resetCZIterator].occupied = false;
       }
       // Rotate gripper back to board angle before moving Cart/Orb
       commandRotateGripper(GRIPPER_ROT_BOARD);
       currentState = RESET_P2_ROTATE_AWAY_CZ; stateStartTime = millis();
       break;

   case RESET_P2_ROTATE_AWAY_CZ:
       if (timeInState >= 450) {
           Serial.println("  Rotated for board -> RESET_P2_MOVE_TO_HOME_SQUARE");
           currentState = RESET_P2_MOVE_TO_HOME_SQUARE; stateStartTime = millis();
       }
       break;

   case RESET_P2_MOVE_TO_HOME_SQUARE:
       Serial.print("[MC Reset P2] MOVE_TO_HOME_SQUARE: Moving K/Q to "); Serial.println(reset_targetHomeSquareAlg);
       getTargetsForSquare(reset_targetHomeSquareAlg, targetOrb1, targetCart1); // Use target1 vars
       enforceCaptureHomedForLowCart(targetCart1); // Safety for destination
       enforceCartSafetyRotation(targetCart1);   // Safety for destination
       stepper1.moveTo(0); // Capture motor home
       stepper2.moveTo(targetCart1);
       stepper3.moveTo(targetOrb1);
       stepper1.enableOutputs(); stepper2.enableOutputs(); stepper3.enableOutputs();
       currentState = RESET_P2_WAIT_HOME_SQUARE_POS; stateStartTime = millis();
       break;

   case RESET_P2_WAIT_HOME_SQUARE_POS:
       if (stepper1.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepper3.distanceToGo()==0) {
           Serial.println("  Arrived at home square -> RESET_P2_ROTATE_FOR_HOME_RELEASE");
           currentState = RESET_P2_ROTATE_FOR_HOME_RELEASE; stateStartTime = millis();
       }
       break;

   case RESET_P2_ROTATE_FOR_HOME_RELEASE:
       Serial.println("[MC Reset P2] ROTATE_FOR_HOME_RELEASE");
       commandRotateGripper(GRIPPER_ROT_BOARD); // Should already be 0
       currentState = RESET_P2_WAIT_ROTATE_HOME_RELEASE; stateStartTime = millis();
       break;

   case RESET_P2_WAIT_ROTATE_HOME_RELEASE:
       if (timeInState >= 450) { // Even if no rotation, ensures sequencing
           Serial.println("  Rotated -> RESET_P2_PERFORM_HOME_RELEASE_EXTEND");
           currentState = RESET_P2_PERFORM_HOME_RELEASE_EXTEND; stateStartTime = millis();
       }
       break;

   // --- "Release" sub-sequence onto board for P2 (simple timed retract) ---
   case RESET_P2_PERFORM_HOME_RELEASE_EXTEND:
        Serial.println("[MC Reset P2] PERFORM_HOME_RELEASE_EXTEND");
        commandExtendActuator(); currentState = RESET_P2_WAIT_HOME_RELEASE_EXTEND; stateStartTime = millis(); break;
   case RESET_P2_WAIT_HOME_RELEASE_EXTEND:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
            commandStopActuator(); currentState = RESET_P2_PERFORM_HOME_RELEASE_OPEN; stateStartTime = millis();
        } break;
   case RESET_P2_PERFORM_HOME_RELEASE_OPEN:
         Serial.println("[MC Reset P2] PERFORM_HOME_RELEASE_OPEN");
         commandGripperOpen(); currentState = RESET_P2_WAIT_HOME_RELEASE_OPEN; stateStartTime = millis(); break;
   case RESET_P2_WAIT_HOME_RELEASE_OPEN:
        if (timeInState >= 300) {
            currentState = RESET_P2_PERFORM_HOME_RELEASE_RETRACT; stateStartTime = millis();
        } break;
   case RESET_P2_PERFORM_HOME_RELEASE_RETRACT:
        Serial.println("[MC Reset P2] PERFORM_HOME_RELEASE_RETRACT (Timed)");
        commandRetractActuator(); currentState = RESET_P2_WAIT_HOME_RELEASE_RETRACT; stateStartTime = millis(); break;
   case RESET_P2_WAIT_HOME_RELEASE_RETRACT:
        if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
            commandStopActuator();
            Serial.println("  Release on home square done -> RESET_P2_PIECE_PLACED");
            currentState = RESET_P2_PIECE_PLACED; stateStartTime = millis();
        } break;

   case RESET_P2_PIECE_PLACED:
       Serial.println("[MC Reset P2] PIECE_PLACED: K/Q moved from CZ to board.");
       // Logical board update will happen at the VERY END in main.cpp after board.resetBoard()
       resetCZIterator++; // Move to next CZ slot
       currentState = RESET_P2_ITERATE_CZ; stateStartTime = millis();
       break;


     // === Phase 3: Place Misplaced Pieces from Board to Home ===
     case RESET_P3_START:
     Serial.println("[MC Reset P3] START: Placing Misplaced Board Pieces to Home -> RESET_P3_ITERATE_BOARD");
     resetBoardIterator = 0; // Start iterating board from square 0
     currentState = RESET_P3_ITERATE_BOARD;
     stateStartTime = millis();
     break;

 case RESET_P3_ITERATE_BOARD:
     reset_currentBoardCoords.first = resetBoardIterator % 8;
     reset_currentBoardCoords.second = resetBoardIterator / 8;
     reset_currentBoardAlg = coordsToAlgebraic(reset_currentBoardCoords.first, reset_currentBoardCoords.second);

     Serial.print("[MC Reset P3] ITERATE_BOARD: Square "); Serial.print(resetBoardIterator);
     Serial.print(" ("); Serial.print(reset_currentBoardAlg); Serial.println(")");

     if (resetBoardIterator >= 64) { // Done with all board squares for phase 3
         Serial.println("  Phase 3 (Place Misplaced Board Pieces) Complete -> RESET_P4_HOME_CAPTURE_MOTOR");
         currentState = RESET_P4_HOME_CAPTURE_MOTOR; // Move to final phase
         stateStartTime = millis();
     } else {
         currentState = RESET_P3_CHECK_BOARD_PIECE;
         stateStartTime = millis();
     }
     break;

 case RESET_P3_CHECK_BOARD_PIECE: {
     Piece* pieceOnSquareP3 = board.grid[reset_currentBoardCoords.first][reset_currentBoardCoords.second];
     bool pieceToMoveHome = false;
     reset_targetHomeSquareCoords = {-1,-1}; // Reset target

     if (pieceOnSquareP3 != nullptr) {
         PieceType type = pieceOnSquareP3->getType();
         PieceColor color = pieceOnSquareP3->getColor();
         std::vector<std::pair<int, int>> potentialHomes = getPotentialHomeSquares(type, color);
         bool isAlreadyHome = false;

         // Check if piece is already on one of its valid home squares
         for (const auto& home : potentialHomes) {
             if (reset_currentBoardCoords.first == home.first && reset_currentBoardCoords.second == home.second) {
                 isAlreadyHome = true;
                 break;
             }
         }

         if (!isAlreadyHome) {
             // Find the first available empty home square for this piece type/color
             for (const auto& home : potentialHomes) {
                 if (board.grid[home.first][home.second] == nullptr) {
                     reset_targetHomeSquareCoords = home;
                     reset_targetHomeSquareAlg = coordsToAlgebraic(home.first, home.second);
                     Serial.print("  Found misplaced piece: "); Serial.print(pieceOnSquareP3->getSymbol());
                     Serial.print(" at "); Serial.print(reset_currentBoardAlg);
                     Serial.print(". Target available home: "); Serial.println(reset_targetHomeSquareAlg);
                     pieceToMoveHome = true;
                     break; // Found a suitable empty home
                 }
             }
             if (!pieceToMoveHome) {
                  Serial.print("  Piece "); Serial.print(pieceOnSquareP3->getSymbol());
                  Serial.print(" at "); Serial.print(reset_currentBoardAlg);
                  Serial.println(" is misplaced, but no available home squares found. Skipping.");
             }
         } else {
             Serial.print("  Piece "); Serial.print(pieceOnSquareP3->getSymbol());
             Serial.print(" at "); Serial.print(reset_currentBoardAlg); Serial.println(" is already home. Skipping.");
         }
     } else { Serial.print("  Square "); Serial.print(reset_currentBoardAlg); Serial.println(" is empty."); }

     if (pieceToMoveHome) {
          // Only store targets, don't call startMoveSequence yet
          Serial.print("[MC Reset P3] CHECK_BOARD_PIECE: Identified piece ");
          Serial.print(reset_currentBoardAlg); Serial.print(" needs to move to ");
          Serial.println(reset_targetHomeSquareAlg);
          currentState = RESET_P3_INITIATE_MOVE_TO_HOME; // Transition to initiate
     } else {
         resetBoardIterator++; // Move to next square
         currentState = RESET_P3_ITERATE_BOARD;
     }
     stateStartTime = millis();
    }
     break;

     case RESET_P3_INITIATE_MOVE_TO_HOME: {
     Serial.print("[MC Reset P3] INITIATE_MOVE_TO_HOME: For ");
     Serial.print(reset_currentBoardAlg); Serial.print(" -> "); Serial.println(reset_targetHomeSquareAlg);

     // Set the state we want to return to AFTER the DO sequence finishes
     stateToReturnToAfterSubSequence = RESET_P3_PIECE_HOMED;
     // Call startMoveSequence, indicating it's a sub-sequence call
     if (startMoveSequence(reset_currentBoardAlg, reset_targetHomeSquareAlg, true)) {
         // If successful, startMoveSequence has set currentState to DO_START (or DO_CAPTURE_START)
         // and subSequenceIsActive to true.
         // This state (RESET_P3_INITIATE_MOVE_TO_HOME) has done its job.
         // The DO sequence will now run. DO_COMPLETE will handle returning to RESET_P3_PIECE_HOMED.
         Serial.println("  P3: DO sub-sequence initiated.");
     } else {
         Serial.println("!!! ERROR: P3 Failed to start sub-move sequence! Skipping piece.");
         // If it failed (e.g., already another sub-sequence active, though shouldn't happen here),
         // then go to next iteration.
         stateToReturnToAfterSubSequence = MOTION_IDLE; // Clear return state
         subSequenceIsActive = false; // Clear flag
         resetBoardIterator++;
         currentState = RESET_P3_ITERATE_BOARD;
         stateStartTime = millis();
     }
     // No matter if it succeeded or failed to start, this state's action is done for this cycle.
     // If it succeeded, the next state will be DO_START.
     // If it failed, the next state is RESET_P3_ITERATE_BOARD.
    }
     break; // Break to allow the new currentState (DO_START or ITERATE_BOARD) to execute next cycle.

       


    case RESET_P3_PIECE_HOMED:
    Serial.println("[MC Reset P3] PIECE_HOMED. Iterating to next board square.");
    resetBoardIterator++;
    currentState = RESET_P3_ITERATE_BOARD;
    stateStartTime = millis();
    break;

    // === Phase 4: Finish ===
    case RESET_P4_HOME_CAPTURE_MOTOR:
        Serial.println("[MC Reset P4] HOME_CAPTURE_MOTOR");
        stepper1.moveTo(0); stepper1.enableOutputs();
        currentState = RESET_P4_WAIT_CAPTURE_MOTOR; stateStartTime = millis();
        break;
    case RESET_P4_WAIT_CAPTURE_MOTOR:
        if (stepper1.distanceToGo() == 0) {
            Serial.println("  Capture motor homed -> RESET_P4_LOGICAL_BOARD_RESET");
            currentState = RESET_P4_LOGICAL_BOARD_RESET; stateStartTime = millis();
        }
        break;
    case RESET_P4_LOGICAL_BOARD_RESET:
        Serial.println("[MC Reset P4] LOGICAL_BOARD_RESET");
        board.resetBoard(); // Reset logical board state
        board.printBoard();
        currentState = RESET_P4_UPDATE_NEXTION; stateStartTime = millis();
        break;
    case RESET_P4_UPDATE_NEXTION:
        Serial.println("[MC Reset P4] UPDATE_NEXTION (Full Redraw)");
        // Trigger a full redraw of the Nextion board
        // This needs a function in main.cpp or a direct call
        // Example: if (nextion.currentPageId == BOARD_SCREEN) { load_nextion_page(); } // Force refresh
        // For now, just mark as done
        Serial.println("  (Nextion redraw would happen here)");
        currentState = RESET_COMPLETE; stateStartTime = millis();
        break;
    case RESET_COMPLETE:
        Serial.println("[MC Reset] RESET_COMPLETE -> MOTION_IDLE");
        currentState = MOTION_IDLE; stateStartTime = millis();
        break;

                // =================== CAPTURE SEQUENCE ==================
        // (Executed *before* moving the source piece if isCaptureMove is true)
        case DO_CAPTURE_START:
        Serial.println("[MC State] DO_CAPTURE_START -> DO_CAPTURE_SAFETY_CHECKS_TARGET");
        currentState = DO_CAPTURE_SAFETY_CHECKS_TARGET; stateStartTime = millis();
        // break; // Fall through ok

    case DO_CAPTURE_SAFETY_CHECKS_TARGET:
        Serial.println("[MC State] DO_CAPTURE_SAFETY_CHECKS_TARGET");
        // Safety checks before moving to TARGET square (where piece to be captured is)
        enforceCaptureHomedForLowCart(targetCart2); // Use targetCart2 (destination cart pos)
        enforceCartSafetyRotation(targetCart2);
        Serial.println("  Safety checks for capture target complete.");
        currentState = DO_CAPTURE_MOVE_TO_TARGET; stateStartTime = millis();
        // break; // Fall through ok

    case DO_CAPTURE_MOVE_TO_TARGET:
        Serial.println("[MC State] DO_CAPTURE_MOVE_TO_TARGET: Commanding steppers...");
        // Move Cart/Orb to the TARGET square ('to' location)
        stepper1.stop(); // Ensure capture stepper stays put (should be home)
        stepper2.moveTo(targetCart2);
        stepper3.moveTo(targetOrb2);
        stepper2.enableOutputs(); stepper3.enableOutputs();
        currentState = DO_CAPTURE_WAIT_TARGET; stateStartTime = millis();
        break; // Wait

    case DO_CAPTURE_WAIT_TARGET:
        if (stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
             Serial.println("[MC State] DO_CAPTURE_WAIT_TARGET: Arrived -> DO_CAPTURE_ROTATE_TARGET");
             currentState = DO_CAPTURE_ROTATE_TARGET; stateStartTime = millis();
        }
        break; // Keep waiting

    case DO_CAPTURE_ROTATE_TARGET:
         Serial.println("[MC State] DO_CAPTURE_ROTATE_TARGET: Commanding rotation...");
         commandRotateGripper(targetRot2); // Rotate gripper for board interaction
         currentState = DO_CAPTURE_WAIT_ROTATE_TARGET; stateStartTime = millis();
         break; // Wait

    case DO_CAPTURE_WAIT_ROTATE_TARGET:
        if (timeInState >= 450) {
            Serial.println("[MC State] DO_CAPTURE_WAIT_ROTATE_TARGET: Rotated -> DO_CAPTURE_PERFORM_TAKE_EXTEND");
            currentState = DO_CAPTURE_PERFORM_TAKE_EXTEND; stateStartTime = millis();
        }
        break;

    // --- Grab the piece being captured (uses TAKE states but prefixed with CAPTURE_) ---
    case DO_CAPTURE_PERFORM_TAKE_EXTEND:
         Serial.println("[MC State] DO_CAPTURE_PERFORM_TAKE_EXTEND: Opening & Extending...");
         commandGripperOpen(); commandExtendActuator();
         currentState = DO_CAPTURE_WAIT_TAKE_EXTEND; stateStartTime = millis();
         break;
    case DO_CAPTURE_WAIT_TAKE_EXTEND:
         if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
             Serial.println("  Extend complete."); commandStopActuator();
             currentState = DO_CAPTURE_PERFORM_TAKE_GRAB; stateStartTime = millis();
         } break;
    case DO_CAPTURE_PERFORM_TAKE_GRAB:
         Serial.println("[MC State] DO_CAPTURE_PERFORM_TAKE_GRAB: Closing gripper...");
         commandGripperClose();
         currentState = DO_CAPTURE_WAIT_TAKE_GRAB; stateStartTime = millis();
         break;
    case DO_CAPTURE_WAIT_TAKE_GRAB:
         if (timeInState >= 300) {
             Serial.println("  Grab complete.");
             currentState = DO_CAPTURE_PERFORM_TAKE_RETRACT; stateStartTime = millis();
         } break;
     case DO_CAPTURE_PERFORM_TAKE_RETRACT:
         Serial.println("[MC State] DO_CAPTURE_PERFORM_TAKE_RETRACT: Retracting...");
         commandRetractActuator();
         currentState = DO_CAPTURE_WAIT_TAKE_RETRACT; stateStartTime = millis();
         break;
         case DO_CAPTURE_WAIT_TAKE_RETRACT: // State after grabbing the piece to be captured
         if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
            Serial.println("  Retract complete (Captured piece held)."); commandStopActuator();
            // NOW, move towards capture zone dropoff point (Cart only first)
            currentState = DO_CAPTURE_MOVE_TO_DROPOFF; // <<< Transition remains the same
            stateStartTime = millis();
        } break;

   // --- Move to Capture Zone Dropoff ---
   case DO_CAPTURE_MOVE_TO_DROPOFF:
       Serial.println("[MC State] DO_CAPTURE_MOVE_TO_DROPOFF: Commanding Cart to dropoff...");
       // Move ONLY the Cart to the predefined capture interaction position first
       enforceCaptureHomedForLowCart(CART_CAPTURE_POS);
       // Gripper should still be at board angle (0) here, so safety rotation isn't strictly needed
       // enforceCartSafetyRotation(CART_CAPTURE_POS);
       stepper1.stop(); // Capture stays home for now
       stepper3.moveTo(stepper3.currentPosition()); // Orb stays put
       stepper2.moveTo(CART_CAPTURE_POS);
       stepper2.enableOutputs();
       currentState = DO_CAPTURE_WAIT_DROPOFF; // <<< Wait for Cart to arrive
       stateStartTime = millis();
       break;

   case DO_CAPTURE_WAIT_DROPOFF:
        if (stepper2.distanceToGo() == 0) {
            Serial.println("[MC State] DO_CAPTURE_WAIT_DROPOFF: Cart Arrived -> DO_CAPTURE_FIND_SLOT");
            currentState = DO_CAPTURE_FIND_SLOT; // <<< Find the slot NEXT
            stateStartTime = millis();
        }
        break;

   case DO_CAPTURE_FIND_SLOT:
       Serial.println("[MC State] DO_CAPTURE_FIND_SLOT: Finding free slot...");
       targetCaptureSlotIndex = findFreeCaptureSlot(pieceBeingCaptured.color);
       if (targetCaptureSlotIndex == -1) {
           Serial.println("!!! ERROR: Capture zone full! Cannot place piece. !!!");
           currentState = ERROR_STATE;
       } else {
           if (getTargetForCapture(targetCaptureSlotIndex + 1, targetCaptureSlotPos)) { // +1 for 1-32 index
               Serial.print("  Found free slot: "); Serial.print(targetCaptureSlotIndex);
               Serial.print(" -> Stepper Pos: "); Serial.println(targetCaptureSlotPos);
               // NOW that we have the target position, move the Capture Stepper
               currentState = DO_CAPTURE_MOVE_CAPTURE_STEPPER; // <<< Move Capture Stepper NEXT
           } else {
               Serial.println("!!! ERROR: Could not get stepper pos for found capture slot!");
               currentState = ERROR_STATE;
           }
       }
       stateStartTime = millis();
       break; // Break after finding slot or error

   // --- NEW ORDER: Move Capture Stepper BEFORE Rotating Gripper ---
   case DO_CAPTURE_MOVE_CAPTURE_STEPPER: // <<< This state comes earlier now
        Serial.println("[MC State] DO_CAPTURE_MOVE_CAPTURE_STEPPER: Moving Capture stepper...");
        stepper1.moveTo(targetCaptureSlotPos); // Move capture stepper to calculated slot pos
        stepper1.enableOutputs();
        currentState = DO_CAPTURE_WAIT_CAPTURE_STEPPER; // <<< Wait for Capture Stepper
        stateStartTime = millis();
        break;

   case DO_CAPTURE_WAIT_CAPTURE_STEPPER: // <<< This state comes earlier now
       if (stepper1.distanceToGo() == 0) {
            Serial.println("[MC State] DO_CAPTURE_WAIT_CAPTURE_STEPPER: Arrived -> DO_CAPTURE_ROTATE_GRIPPER_CAPZONE");
            // NOW Rotate the Gripper
            currentState = DO_CAPTURE_ROTATE_GRIPPER_CAPZONE; // <<< Rotate Gripper LAST before release
            stateStartTime = millis();
       }
       break;

   // --- Rotate Gripper (Now happens AFTER Capture Stepper move) ---
    case DO_CAPTURE_ROTATE_GRIPPER_CAPZONE: // <<< This state comes later now
        Serial.println("[MC State] DO_CAPTURE_ROTATE_GRIPPER_CAPZONE: Rotating for capture zone...");
        commandRotateGripper(GRIPPER_ROT_CAPTURE); // Rotate to capture zone angle
        currentState = DO_CAPTURE_WAIT_ROTATE_CAPZONE; // <<< Wait for Rotation
        stateStartTime = millis();
        break;

    case DO_CAPTURE_WAIT_ROTATE_CAPZONE: // <<< This state comes later now
        if (timeInState >= 450) {
            Serial.println("  Rotated for capture zone -> DO_CAPTURE_PERFORM_RELEASE_EXTEND");
            // NOW proceed with releasing the piece
            currentState = DO_CAPTURE_PERFORM_RELEASE_EXTEND; // <<< Start release sequence
            stateStartTime = millis();
        }
        break;

    // --- Release the captured piece into the slot (uses RELEASE states prefixed with CAPTURE_) ---
    case DO_CAPTURE_PERFORM_RELEASE_EXTEND:
         Serial.println("[MC State] DO_CAPTURE_PERFORM_RELEASE_EXTEND: Extending...");
         commandExtendActuator();
         currentState = DO_CAPTURE_WAIT_RELEASE_EXTEND; stateStartTime = millis();
         break;
     case DO_CAPTURE_WAIT_RELEASE_EXTEND:
         if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
             Serial.println("  Extend complete."); commandStopActuator();
             currentState = DO_CAPTURE_PERFORM_RELEASE_OPEN; stateStartTime = millis();
         } break;
     case DO_CAPTURE_PERFORM_RELEASE_OPEN:
          Serial.println("[MC State] DO_CAPTURE_PERFORM_RELEASE_OPEN: Opening gripper...");
          commandGripperOpen();
          currentState = DO_CAPTURE_WAIT_RELEASE_OPEN; stateStartTime = millis();
          break;
     case DO_CAPTURE_WAIT_RELEASE_OPEN:
         if (timeInState >= 300) {
             Serial.println("  Open complete.");
             // Update capture zone array NOW that piece is placed
             if (targetCaptureSlotIndex != -1) {
                capture_zone[targetCaptureSlotIndex] = pieceBeingCaptured; // Store piece info
                capture_zone[targetCaptureSlotIndex].occupied = true;
                Serial.print("  Updated capture_zone["); Serial.print(targetCaptureSlotIndex); Serial.println("]");
             }
             currentState = DO_CAPTURE_PERFORM_RELEASE_RETRACT; stateStartTime = millis();
         } break;
    case DO_CAPTURE_PERFORM_RELEASE_RETRACT:
         Serial.println("[MC State] DO_CAPTURE_PERFORM_RELEASE_RETRACT: Retracting...");
         commandRetractActuator();
         currentState = DO_CAPTURE_WAIT_RELEASE_RETRACT; stateStartTime = millis();
         break;
     case DO_CAPTURE_WAIT_RELEASE_RETRACT:
         if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
             Serial.println("  Retract complete (Captured piece released)."); commandStopActuator();
             // Finished handling the captured piece. Now proceed to move the original piece.
             currentState = DO_CAPTURE_COMPLETE; stateStartTime = millis();
         } break;

    case DO_CAPTURE_COMPLETE:
         Serial.println("[MC State] DO_CAPTURE_COMPLETE -> DO_START (to move original piece)");
         // Need to rotate gripper back to board angle BEFORE moving steppers away
         commandRotateGripper(GRIPPER_ROT_BOARD);
         // Small delay might be needed here, or add a dedicated wait state
         delay(450);
         // Now we proceed as if starting the move from the beginning, but without the capture check
         // Go back to the safety checks for the *source* location of the piece we actually want to move
         currentState = DO_SAFETY_CHECKS_SOURCE;
         stateStartTime = millis();
         break; // Don't fall through

        case DO_START:
            Serial.println("[MC State] DO_START (Non-Capture) -> DO_SAFETY_CHECKS_SOURCE");
            currentState = DO_SAFETY_CHECKS_SOURCE; stateStartTime = millis();

        case DO_SAFETY_CHECKS_SOURCE:
             Serial.println("[MC State] DO_SAFETY_CHECKS_SOURCE");
             enforceCaptureHomedForLowCart(targetCart1);
             enforceCartSafetyRotation(targetCart1);
             Serial.println("  Safety checks source complete.");
             currentState = DO_MOVE_STEPPERS_TO_SOURCE; stateStartTime = millis();
             // break; // Fall through okay here too
             // Fall through is okay here

        case DO_MOVE_STEPPERS_TO_SOURCE:
            Serial.println("[MC State] DO_MOVE_STEPPERS_TO_SOURCE: Commanding...");
             stepper1.moveTo(targetCapt1); stepper2.moveTo(targetCart1); stepper3.moveTo(targetOrb1);
             stepper1.enableOutputs(); stepper2.enableOutputs(); stepper3.enableOutputs();
             currentState = DO_WAIT_STEPPERS_SOURCE; stateStartTime = millis();
             break; // <<<=== MUST Break here to wait

        case DO_WAIT_STEPPERS_SOURCE: {
            bool cartDone = (stepper2.distanceToGo() == 0);
            bool orbDone = (locType1 != LOC_BOARD) || (stepper3.distanceToGo() == 0);
            bool captDone = (locType1 != LOC_CAPTURE) || (stepper1.distanceToGo() == 0);
            if (cartDone && orbDone && captDone) {
                Serial.println("[MC State] DO_WAIT_STEPPERS_SOURCE: Done -> DO_ROTATE_GRIPPER_SOURCE");
                currentState = DO_ROTATE_GRIPPER_SOURCE; stateStartTime = millis();
            }
        }
            break; // <<<=== Keep waiting or transition

        case DO_ROTATE_GRIPPER_SOURCE: {
            Serial.println("[MC State] DO_ROTATE_GRIPPER_SOURCE: Commanding...");
            commandRotateGripper(targetRot1);
            currentState = DO_WAIT_GRIPPER_SOURCE; stateStartTime = millis();
        }
            break; // <<<=== MUST Break here to wait
        

        case DO_WAIT_GRIPPER_SOURCE: {
             if (timeInState >= 450) {
                 Serial.println("[MC State] DO_WAIT_GRIPPER_SOURCE: Done -> DO_PERFORM_TAKE_EXTEND");
                 currentState = DO_PERFORM_TAKE_EXTEND; stateStartTime = millis();
             }
            }
             break; // <<<=== Keep waiting or transition
            
        // --- REGULAR TAKE SEQUENCE (Modified Retract Logic) ---
        case DO_PERFORM_TAKE_EXTEND: {
             Serial.println("[MC State] DO_PERFORM_TAKE_EXTEND: Commanding...");
             commandGripperOpen(); commandExtendActuator();
             currentState = DO_WAIT_TAKE_EXTEND; stateStartTime = millis();
        }
             break; // <<<=== MUST Break here to wait

        case DO_WAIT_TAKE_EXTEND: {
             if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
                 Serial.println("[MC State] DO_WAIT_TAKE_EXTEND: Done -> DO_PERFORM_TAKE_GRAB");
                 commandStopActuator();
                 currentState = DO_PERFORM_TAKE_GRAB; stateStartTime = millis();
             }
            }
             break; // <<<=== Keep waiting or transition

        case DO_PERFORM_TAKE_GRAB:
             Serial.println("[MC State] DO_PERFORM_TAKE_GRAB: Commanding...");
             commandGripperClose();
             currentState = DO_WAIT_TAKE_GRAB; stateStartTime = millis();
             break; // <<<=== MUST Break here to wait

             case DO_WAIT_TAKE_GRAB:      // ... (same as before -> transition to new retract start) ...
             if (currentState == DO_WAIT_TAKE_GRAB && timeInState >= 300) {
                 Serial.println("[MC State] DO_WAIT_TAKE_GRAB: Done -> DO_PERFORM_TAKE_RETRACT_INITIAL");
                 currentState = DO_PERFORM_TAKE_RETRACT_INITIAL; // <<< Start new retract sequence
                 stateStartTime = millis();
             }
             break;

        // --- New Retract Sequence with Sensor Check ---
        case DO_PERFORM_TAKE_RETRACT_INITIAL:
            Serial.println("[MC State] DO_PERFORM_TAKE_RETRACT_INITIAL: Starting timed retract...");
            commandRetractActuator(); // Start retracting motor
            retractRetryCount = 0; // Reset retry counter
            currentState = DO_WAIT_TAKE_RETRACT_INITIAL;
            stateStartTime = millis();
            break;

        case DO_WAIT_TAKE_RETRACT_INITIAL:
            // Wait slightly longer than the expected travel time to ensure it *should* hit the limit
            if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150) {
                Serial.println("  Initial retract time elapsed -> DO_CHECK_TAKE_RETRACT_SENSOR");
                // *** DO NOT STOP THE ACTUATOR YET *** - Keep power applied for sensor check
                currentState = DO_CHECK_TAKE_RETRACT_SENSOR;
                stateStartTime = millis(); // Reset timer for the check phase
            }
            break;

        case DO_CHECK_TAKE_RETRACT_SENSOR:
            // Give a brief moment for the sensor signal to stabilize after potentially hitting limit
            if (timeInState >= 100) { // Wait 100ms before checking sensor
                 Serial.print("  Checking retract sensor (Pin "); Serial.print(ACTUATOR_RETRACTED_SENSE_PIN); Serial.print(")... ");
                 // *** CHECK THE SENSOR PIN *** (Adjust LOW/HIGH based on your circuit)
                 if (digitalRead(ACTUATOR_RETRACTED_SENSE_PIN) == HIGH) { 
                     Serial.println("Sensor Active (Retracted!) -> DO_TAKE_RETRACT_CONFIRMED");
                     commandStopActuator(); // Successfully retracted, NOW stop motor
                     currentState = DO_TAKE_RETRACT_CONFIRMED;
                 } else {
                     Serial.println("Sensor Inactive (Not fully retracted).");
                     // Check if we should try recovery
                     if (retractRetryCount < 1) { // Try recovery sequence once
                         Serial.println("  -> Starting Recovery Attempt 1 (Rotate)");
                         currentState = DO_RECOVER_TAKE_RETRACT_ROTATE;
                         retractRetryCount++;
                     } else {
                         Serial.println("!!! ERROR: Retract Sensor check failed after retries! -> DO_TAKE_RETRACT_FAILED");
                         commandStopActuator(); // Stop trying
                         currentState = DO_TAKE_RETRACT_FAILED;
                     }
                 }
                 stateStartTime = millis(); // Reset timer for next state
            }
            break; // Keep checking for 100ms or until transition

        // --- Recovery Sequence ---
        case DO_RECOVER_TAKE_RETRACT_ROTATE:{
            Serial.println("[MC State] DO_RECOVER_TAKE_RETRACT_ROTATE: Rotating gripper slightly...");
            commandStopActuator(); // Stop retract command first
            // Rotate gripper slightly (e.g., 5 degrees from current) - Choose direction carefully
            int currentRot = servo1.read();
            commandRotateGripper(constrain(currentRot - 5, 0, 180)); // Rotate +5 degrees (adjust angle/direction)
            currentState = DO_WAIT_RECOVER_ROTATE;
            stateStartTime = millis();
        }
            break;

        case DO_WAIT_RECOVER_ROTATE:
        {
            if (timeInState >= 400) { // Wait for rotation servo
                Serial.println("  Rotation complete -> DO_RECOVER_TAKE_RETRACT_RETRY");
                currentState = DO_RECOVER_TAKE_RETRACT_RETRY;
                stateStartTime = millis();
            }
        }
            break;

        case DO_RECOVER_TAKE_RETRACT_RETRY:{
            Serial.println("[MC State] DO_RECOVER_TAKE_RETRACT_RETRY: Retrying retract...");
            commandRetractActuator(); // Try retracting again
            currentState = DO_WAIT_RECOVER_RETRY;
            stateStartTime = millis();
        }
            break;

        case DO_WAIT_RECOVER_RETRY:{
            // Wait normal retract time again
             if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 150) {
                Serial.println("  Retry retract time elapsed -> Checking sensor again...");
                // Go back to sensor check state, DO NOT stop motor
                 currentState = DO_CHECK_TAKE_RETRACT_SENSOR; // Loop back to check sensor
                 stateStartTime = millis();
             }
            }
             break;
        // --- Retract Outcome States ---
        case DO_TAKE_RETRACT_CONFIRMED:{
            Serial.println("[MC State] DO_TAKE_RETRACT_CONFIRMED: 'Take' sequence fully complete -> DO_SAFETY_CHECKS_DEST");
            // Retraction successful, proceed to the next major phase (Move to Destination)
            currentState = DO_SAFETY_CHECKS_DEST;
            stateStartTime = millis();
        }
            break; // Proceed

        case DO_TAKE_RETRACT_FAILED:
             Serial.println("[MC State] DO_TAKE_RETRACT_FAILED: Retraction failed! -> ERROR_STATE");
             // Could not confirm retraction, enter error state
             currentState = ERROR_STATE;
             stateStartTime = millis();
             break; // Go to error

         case DO_SAFETY_CHECKS_DEST: {
            Serial.println("[MC State] DO_SAFETY_CHECKS_DEST");
            enforceCaptureHomedForLowCart(targetCart2); enforceCartSafetyRotation(targetCart2);
            Serial.println("  Safety checks dest complete.");
            if (locType2 == LOC_CAPTURE) { currentState = DO_MOVE_CART_TO_DEST; }
            else { currentState = DO_ROTATE_GRIPPER_DEST; }
            stateStartTime = millis();
         }
            break; // <<<=== MUST Break here

         case DO_MOVE_CART_TO_DEST: {
            Serial.println("[MC State] DO_MOVE_CART_TO_DEST: Commanding...");
            stepper2.moveTo(targetCart2); stepper2.enableOutputs();
            currentState = DO_WAIT_CART_DEST; stateStartTime = millis();
         }
            break; // <<<=== MUST Break here to wait

         case DO_WAIT_CART_DEST:
            if (stepper2.distanceToGo() == 0) {
                Serial.println("[MC State] DO_WAIT_CART_DEST: Done -> DO_ROTATE_GRIPPER_DEST");
                currentState = DO_ROTATE_GRIPPER_DEST; stateStartTime = millis();
            }
            break; // <<<=== Keep waiting or transition

        case DO_ROTATE_GRIPPER_DEST:
             Serial.println("[MC State] DO_ROTATE_GRIPPER_DEST: Commanding...");
             commandRotateGripper(targetRot2);
             currentState = DO_WAIT_GRIPPER_DEST; stateStartTime = millis();
             break; // <<<=== MUST Break here to wait

        case DO_WAIT_GRIPPER_DEST:
             if (timeInState >= 450) {
                 Serial.println("[MC State] DO_WAIT_GRIPPER_DEST: Done -> DO_MOVE_STEPPERS_TO_DEST");
                 currentState = DO_MOVE_STEPPERS_TO_DEST; stateStartTime = millis();
             }
             break; // <<<=== Keep waiting or transition

        case DO_MOVE_STEPPERS_TO_DEST:
            Serial.println("[MC State] DO_MOVE_STEPPERS_TO_DEST: Commanding...");
            if (locType2 == LOC_CAPTURE) {
                stepper1.moveTo(targetCapt2); // Fixed target
                stepper1.enableOutputs();
                stepper3.moveTo(targetOrb2);
            } else {
                stepper1.stop(); // Let it stay home if forced
                // stepper1.moveTo(0); // Optional explicit home
                stepper2.moveTo(targetCart2);
                stepper3.moveTo(targetOrb2);
                stepper2.enableOutputs();
                stepper3.enableOutputs();
            }
             currentState = DO_WAIT_STEPPERS_DEST; stateStartTime = millis();
             break; // <<<=== MUST Break here to wait

        case DO_WAIT_STEPPERS_DEST:
        if (currentState == DO_WAIT_STEPPERS_DEST) {
            // Check completion logic from previous implementation...
            bool cartDoneDest = (locType2 == LOC_CAPTURE) || (stepper2.distanceToGo() == 0);
            bool orbDoneDest = (locType2 == LOC_CAPTURE) || (stepper3.distanceToGo() == 0);
            bool captDoneDest = (locType2 == LOC_BOARD) || (stepper1.distanceToGo() == 0);

            if (cartDoneDest && orbDoneDest && captDoneDest) {
                Serial.println("[MC State] DO_WAIT_STEPPERS_DEST: Done -> DO_PERFORM_RELEASE_EXTEND");
                currentState = DO_PERFORM_RELEASE_EXTEND; stateStartTime = millis();
            }
        }
             break; // <<<=== Keep waiting or transition
// --- REGULAR RELEASE SEQUENCE (Does NOT use sensor check) ---
         case DO_PERFORM_RELEASE_EXTEND:
             Serial.println("[MC State] DO_PERFORM_RELEASE_EXTEND: Commanding...");
             commandExtendActuator();
             currentState = DO_WAIT_RELEASE_EXTEND; stateStartTime = millis();
             break; // <<<=== MUST Break here to wait

         case DO_WAIT_RELEASE_EXTEND:
             if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
                 Serial.println("[MC State] DO_WAIT_RELEASE_EXTEND: Done -> DO_PERFORM_RELEASE_OPEN");
                 commandStopActuator();
                 currentState = DO_PERFORM_RELEASE_OPEN; stateStartTime = millis();
             }
             break; // <<<=== Keep waiting or transition

         case DO_PERFORM_RELEASE_OPEN:
              Serial.println("[MC State] DO_PERFORM_RELEASE_OPEN: Commanding...");
              commandGripperOpen();
              currentState = DO_WAIT_RELEASE_OPEN; stateStartTime = millis();
              break; // <<<=== MUST Break here to wait

         case DO_WAIT_RELEASE_OPEN:
             if (timeInState >= 300) {
                 Serial.println("[MC State] DO_WAIT_RELEASE_OPEN: Done -> DO_PERFORM_RELEASE_RETRACT");
                 currentState = DO_PERFORM_RELEASE_RETRACT; stateStartTime = millis();
             }
             break; // <<<=== Keep waiting or transition

        case DO_PERFORM_RELEASE_RETRACT:// <<< Release uses simple timed retract
             Serial.println("[MC State] DO_PERFORM_RELEASE_RETRACT (Simple Timed): Commanding actuator retract...");
             commandRetractActuator();
             currentState = DO_WAIT_RELEASE_RETRACT; // <<< Go to simple wait state
             stateStartTime = millis();
             break;
         case DO_WAIT_RELEASE_RETRACT:   // <<< Simple timed wait state for release
              if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
                 Serial.println("[MC State] DO_WAIT_RELEASE_RETRACT (Simple Timed): Done -> DO_COMPLETE");
                 commandStopActuator();
                 currentState = DO_COMPLETE;
                 stateStartTime = millis();
             }
             break;

             case DO_COMPLETE: {
             Serial.print("[MC State] DO_COMPLETE. Sub-sequence was: "); Serial.println(subSequenceIsActive ? "ACTIVE" : "INACTIVE");
             Serial.print("  Returning to state: "); Serial.println(stateToReturnToAfterSubSequence);
 
              if (subSequenceIsActive) {
                  currentState = stateToReturnToAfterSubSequence; // Go back to the calling sequence's next state
                  subSequenceIsActive = false; // Clear the flag
                  stateToReturnToAfterSubSequence = MOTION_IDLE; // Reset for next time
              } else {
                  currentState = MOTION_IDLE; // Standard completion to IDLE
              }
              stateStartTime = millis();
              // Note: If returning to another state, that state's logic will execute in the *next* cycle.
            }
              break; // IMPORTANT: Break here

         case ERROR_STATE:
             Serial.println("!!! Motion Controller in ERROR STATE !!!");
        
             break; 

        default:
            Serial.print("MotionController Error: Unknown state: "); Serial.println(currentState);
            currentState = ERROR_STATE;
            break; 
    }
}
MotionState MotionController::getCurrentState() const { // <<<=== ADDED Getter Definition
    return currentState;
}