// --- START OF FILE MotionController.cpp ---

#include "MotionController.h"
#include "hardware_pins.h" // Ensure pins are included

extern Board board; // <<<=== DECLARE The Global Board Object

MotionController::MotionController() :
    stepper1(AccelStepper::DRIVER, CAPTURE_STEP_PIN, CAPTURE_DIR_PIN),
    stepper2(AccelStepper::DRIVER, CART_STEP_PIN, CART_DIR_PIN),
    stepper3(AccelStepper::DRIVER, ORB_STEP_PIN, ORB_DIR_PIN),
    currentState(MOTION_IDLE) // <<<=== Use Renamed State
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
// --- Capture Zone Initialization ---
void MotionController::initializeCaptureZone() {
    Serial.println("Initializing Capture Zone array...");
    for (int i = 0; i < 32; ++i) {
        capture_zone[i].occupied = false;
    }
    // Pre-populate slots 0 and 1 with Queens (assuming Slot 16 is home/0 steps)
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
     }
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
    case 'a': orbTarget = 4100; break; case 'b': orbTarget = 3200; break; case 'c': orbTarget = 2500; break;
    case 'd': orbTarget = 1700; break; case 'e': orbTarget = 900;  break; case 'f': orbTarget = 80;    break;
    case 'g': orbTarget = 5700; break; case 'h': orbTarget = 4950; break;
    default: Serial.print("Error: Invalid file: "); Serial.println(fileChar); return false;
  }
  switch (rankChar) {
    case '1': cartTarget = 4500; break; case '2': cartTarget = 3900; break; case '3': cartTarget = 3400; break;
    case '4': cartTarget = 2750; break; case '5': cartTarget = 2050; break; case '6': cartTarget = 1400; break;
    case '7': cartTarget = 725;  break; case '8': cartTarget = 0;  break;
    default: Serial.print("Error: Invalid rank: "); Serial.println(rankChar); return false;
  }
  return true;
}

bool MotionController::getTargetForCapture(int captureSlot, long &captureTarget) {
   switch (captureSlot) {
    case 1: captureTarget = 2780; break; case 2: captureTarget = 2600; break; case 3: captureTarget = 2420; break;
    case 4: captureTarget = 2240; break; case 5: captureTarget = 2060; break; case 6: captureTarget = 1880; break;
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
  if (targetCartPos < CART_CAPTURE_HOME_THRESHOLD) {
    if (stepper1.currentPosition() != 0) {
      Serial.print("!!! CRITICAL WARN !!! Cart "); Serial.print(targetCartPos); Serial.print("<CritThresh. Capture not home (");
      Serial.print(stepper1.currentPosition()); Serial.println("). Forcing home!");
      float o_sp = stepper1.maxSpeed(); float o_ac = stepper1.acceleration();
      stepper1.setMaxSpeed(abs(HOMING_SPEED_CAPTURE)); stepper1.setAcceleration(HOMING_ACCEL);
      stepper1.enableOutputs(); stepper1.move(-30000);
      unsigned long startT = millis(); bool homed = false; Serial.println("[SAFETY HOME] Moving...");
      while (!homed && (millis() - startT < 15000)) {
        if (digitalRead(ENDSTOP_CAPTURE_PIN) == LOW) {
          stepper1.stop(); stepper1.setCurrentPosition(0); homed = true; Serial.println("[SAFETY HOME] Homed!");
        } else { stepper1.run(); } delay(1);
      }
      if (!homed) { Serial.println("!!! SAFETY HOME FAILED (Timeout) !!!"); stepper1.stop(); }
      stepper1.setMaxSpeed(o_sp); stepper1.setAcceleration(o_ac); stepper1.runToPosition();
      Serial.println(homed ? "!!! CRITICAL OK !!! Capture homed." : "!!! CRITICAL FAILED !!!");
    }
  }
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

bool MotionController::startMoveSequence(String fromLoc, String toLoc) {
    if (currentState != MOTION_IDLE) { // <<<=== Use Renamed State
        Serial.println("MotionController Error: Busy!"); return false;
    }
    Serial.print("MotionController: Starting move '"); Serial.print(fromLoc);
    Serial.print("' -> '"); Serial.print(toLoc); Serial.println("'");
    targetFromLoc = fromLoc; targetToLoc = toLoc;

    // Parse locations - Calls helper function defined above
    locType1 = parseLocation(targetFromLoc, targetOrb1, targetCart1, targetCapt1, targetRot1);
    locType2 = parseLocation(targetToLoc, targetOrb2, targetCart2, targetCapt2, targetRot2);

    if (locType1 == LOC_INVALID || locType2 == LOC_INVALID) {
        Serial.println("MotionController Error: Invalid location(s)."); return false;
    }

  // --- Check for Capture ---
    isCaptureMove = false; // Reset flag
    if (locType2 == LOC_BOARD) { // Can only capture on the board
        // Convert algebraic 'to' notation to board indices
        // Need a robust conversion function... let's assume simple for now
        int toX = targetToLoc.charAt(0) - 'a'; // a=0, b=1, ...
        int toY = 7 - (targetToLoc.charAt(1) - '1'); // 1=7, 2=6, ..., 8=0
        // Check bounds
        if (toX >= 0 && toX < 8 && toY >= 0 && toY < 8) {
             if (board.grid[toX][toY] != nullptr) { // Check the GLOBAL board object
                 isCaptureMove = true;
                 pieceBeingCaptured.occupied = true; // Mark that we need to store info
                 pieceBeingCaptured.type = board.grid[toX][toY]->getType(); // Store type
                 pieceBeingCaptured.color = board.grid[toX][toY]->getColor(); // Store color
                 Serial.print("  *CAPTURE DETECTED* at "); Serial.print(targetToLoc);
                 Serial.print(" Piece: "); Serial.println(board.grid[toX][toY]->getSymbol());
            }
        } else {
             Serial.print("Error converting 'to' location for capture check: "); Serial.println(targetToLoc);
             // Proceed without capture? Or error out? Let's proceed cautiously.
             isCaptureMove = false;
        }
    }

    // --- Start State Machine ---
    if (isCaptureMove) {
        currentState = DO_CAPTURE_START; // Start with the capture sequence
    } else {
        currentState = DO_START; // Start with the regular move sequence (Safety Checks -> Move Source)
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
     case DO_CAPTURE_WAIT_TAKE_RETRACT:
          if (timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
             Serial.println("  Retract complete (Captured piece held)."); commandStopActuator();
             currentState = DO_CAPTURE_MOVE_TO_DROPOFF; stateStartTime = millis(); // Move towards capture zone
         } break;

    // --- Move to Capture Zone Dropoff ---
    case DO_CAPTURE_MOVE_TO_DROPOFF:
        Serial.println("[MC State] DO_CAPTURE_MOVE_TO_DROPOFF: Commanding Cart to dropoff...");
        // Move ONLY the Cart to the predefined capture interaction position
        enforceCaptureHomedForLowCart(CART_CAPTURE_POS); // Safety check for dropoff pos
        enforceCartSafetyRotation(CART_CAPTURE_POS);   // Should be needed (angle 0)
        stepper1.stop(); // Capture stays home
        stepper3.moveTo(stepper3.currentPosition()); // Orb stays put
        stepper2.moveTo(CART_CAPTURE_POS);
        stepper2.enableOutputs();
        currentState = DO_CAPTURE_WAIT_DROPOFF; stateStartTime = millis();
        break;

    case DO_CAPTURE_WAIT_DROPOFF:
         if (stepper2.distanceToGo() == 0) {
             Serial.println("[MC State] DO_CAPTURE_WAIT_DROPOFF: Arrived -> DO_CAPTURE_FIND_SLOT");
             currentState = DO_CAPTURE_FIND_SLOT; stateStartTime = millis();
         }
         break;

    case DO_CAPTURE_FIND_SLOT:
        Serial.println("[MC State] DO_CAPTURE_FIND_SLOT: Finding free slot...");
        targetCaptureSlotIndex = findFreeCaptureSlot(pieceBeingCaptured.color);
        if (targetCaptureSlotIndex == -1) {
            Serial.println("!!! ERROR: Capture zone full! Cannot place piece. !!!");
            // What to do here? Drop the piece? Go to error state?
            currentState = ERROR_STATE; // Go to error state for now
        } else {
            if (getTargetForCapture(targetCaptureSlotIndex + 1, targetCaptureSlotPos)) { // +1 because slots are 1-32 in func
                Serial.print("  Found free slot: "); Serial.print(targetCaptureSlotIndex);
                Serial.print(" -> Stepper Pos: "); Serial.println(targetCaptureSlotPos);
                currentState = DO_CAPTURE_ROTATE_GRIPPER_CAPZONE; // Proceed to rotate gripper
            } else {
                Serial.println("!!! ERROR: Could not get stepper pos for found capture slot!");
                currentState = ERROR_STATE;
            }
        }
        stateStartTime = millis();
        break; // Break after finding slot or error

     case DO_CAPTURE_ROTATE_GRIPPER_CAPZONE:
         Serial.println("[MC State] DO_CAPTURE_ROTATE_GRIPPER_CAPZONE: Rotating for capture zone...");
         commandRotateGripper(GRIPPER_ROT_CAPTURE); // Rotate to capture zone angle
         currentState = DO_CAPTURE_WAIT_ROTATE_CAPZONE; stateStartTime = millis();
         break;

     case DO_CAPTURE_WAIT_ROTATE_CAPZONE:
         if (timeInState >= 450) {
             Serial.println("  Rotated for capture zone.");
             currentState = DO_CAPTURE_MOVE_CAPTURE_STEPPER; stateStartTime = millis();
         }
         break;

    case DO_CAPTURE_MOVE_CAPTURE_STEPPER:
         Serial.println("[MC State] DO_CAPTURE_MOVE_CAPTURE_STEPPER: Moving Capture stepper...");
         stepper1.moveTo(targetCaptureSlotPos); // Move capture stepper to calculated slot pos
         stepper1.enableOutputs();
         currentState = DO_CAPTURE_WAIT_CAPTURE_STEPPER; stateStartTime = millis();
         break;

    case DO_CAPTURE_WAIT_CAPTURE_STEPPER:
        if (stepper1.distanceToGo() == 0) {
             Serial.println("[MC State] DO_CAPTURE_WAIT_CAPTURE_STEPPER: Arrived -> DO_CAPTURE_PERFORM_RELEASE_EXTEND");
             currentState = DO_CAPTURE_PERFORM_RELEASE_EXTEND; stateStartTime = millis();
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

        case DO_WAIT_TAKE_GRAB:
             if (timeInState >= 300) {
                 Serial.println("[MC State] DO_WAIT_TAKE_GRAB: Done -> DO_PERFORM_TAKE_RETRACT");
                 currentState = DO_PERFORM_TAKE_RETRACT; stateStartTime = millis();
             }
             break; // <<<=== Keep waiting or transition

         case DO_PERFORM_TAKE_RETRACT:
             Serial.println("[MC State] DO_PERFORM_TAKE_RETRACT: Commanding...");
             commandRetractActuator();
             currentState = DO_WAIT_TAKE_RETRACT; stateStartTime = millis();
             break; // <<<=== MUST Break here to wait

         case DO_WAIT_TAKE_RETRACT:
         if (currentState == DO_WAIT_TAKE_RETRACT && timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
            Serial.println("[MC State] DO_WAIT_TAKE_RETRACT: Done -> DO_SAFETY_CHECKS_DEST");
            commandStopActuator();
            currentState = DO_SAFETY_CHECKS_DEST; stateStartTime = millis();
        }
             break; // <<<=== Keep waiting or transition

         case DO_SAFETY_CHECKS_DEST:
            Serial.println("[MC State] DO_SAFETY_CHECKS_DEST");
            enforceCaptureHomedForLowCart(targetCart2); enforceCartSafetyRotation(targetCart2);
            Serial.println("  Safety checks dest complete.");
            if (locType2 == LOC_CAPTURE) { currentState = DO_MOVE_CART_TO_DEST; }
            else { currentState = DO_ROTATE_GRIPPER_DEST; }
            stateStartTime = millis();
            break; // <<<=== MUST Break here

         case DO_MOVE_CART_TO_DEST:
            Serial.println("[MC State] DO_MOVE_CART_TO_DEST: Commanding...");
            stepper2.moveTo(targetCart2); stepper2.enableOutputs();
            currentState = DO_WAIT_CART_DEST; stateStartTime = millis();
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

        case DO_PERFORM_RELEASE_RETRACT:
             Serial.println("[MC State] DO_PERFORM_RELEASE_RETRACT: Commanding...");
             commandRetractActuator();
             currentState = DO_WAIT_RELEASE_RETRACT; stateStartTime = millis();
             break; // <<<=== MUST Break here to wait

         case DO_WAIT_RELEASE_RETRACT:
         if (currentState == DO_WAIT_RELEASE_RETRACT && timeInState >= ACTUATOR_TRAVEL_TIME_MS + 50) {
            Serial.println("[MC State] DO_WAIT_RELEASE_RETRACT: Done -> DO_COMPLETE");
            commandStopActuator();
            currentState = DO_COMPLETE; stateStartTime = millis();
        }
             break; // <<<=== Keep waiting or transition

        case DO_COMPLETE:
            Serial.println("[MC State] DO_COMPLETE: Seq Finished -> MOTION_IDLE");
             currentState = MOTION_IDLE; // <<<=== Use Renamed State
             stateStartTime = millis();
             // Immediately process IDLE state in next loop() call
             break; // <<<=== Added break

         case ERROR_STATE:
             Serial.println("!!! Motion Controller in ERROR STATE !!!");
             // Maybe add logic to stop motors here if not already stopped
             break; // <<<=== Added break

        default:
            Serial.print("MotionController Error: Unknown state: "); Serial.println(currentState);
            currentState = ERROR_STATE;
            break; // <<<=== Added break
    }
}
MotionState MotionController::getCurrentState() const { // <<<=== ADDED Getter Definition
    return currentState;
}