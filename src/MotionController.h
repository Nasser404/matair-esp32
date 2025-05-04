// --- START OF FILE MotionController.h ---

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include "hardware_pins.h" // Include pin definitions
#include "board.h"         // Needed for coordinate conversion potentially
#include "enums.h"         // <<<=== INCLUDE ENUMS.H (contains ORB_STATUS::IDLE)
#include "piece.h" // <<< Need Piece definition
// --- Constants (Motion Specific) ---
const float STEPPER_SPEED = 4000;
const float STEPPER_ACCEL = 5000;
const float HOMING_SPEED_CAPTURE = 800;
const float HOMING_SPEED_CART_ORB = 1000;
const float HOMING_ACCEL = 1500;

const int GRIPPER_ROT_BOARD = 180;
const int GRIPPER_ROT_CAPTURE = 63;
const long CART_SAFETY_THRESHOLD = 2250;
const long CART_CAPTURE_HOME_THRESHOLD = 800;
const long CART_CAPTURE_POS = 2250;

const int GripperOpen = 180;
const int GripperClose = 50;

const unsigned long ACTUATOR_TRAVEL_TIME_MS = 600;
// --- Capture Zone Representation ---
struct CapturedPieceInfo {
    bool occupied = false;
    PieceType type = PieceType::PAWN; // Default placeholder
    PieceColor color = PieceColor::WHITE; // Default placeholder
};

// --- Location Types (Defined Globally Here) --- <<<=== DEFINE ENUM HERE
enum LocationType {
    LOC_INVALID,
    LOC_BOARD,
    LOC_CAPTURE
};

// --- State Machine ---
enum MotionState {
    MOTION_IDLE,
    HOMING_START,
    // ... (Homing states) ...
    HOMING_CAPTURE_START_MOVE,
    HOMING_CAPTURE_WAIT_HIT,
    HOMING_CART_ORB_START_MOVE,
    HOMING_CART_ORB_WAIT_HIT,
    HOMING_COMPLETE,
    DO_START,
    DO_SAFETY_CHECKS_SOURCE,
    DO_MOVE_STEPPERS_TO_SOURCE,
    DO_WAIT_STEPPERS_SOURCE,
    DO_ROTATE_GRIPPER_SOURCE,
    DO_WAIT_GRIPPER_SOURCE,
    // Capture Handling (If destination is occupied)
    DO_CAPTURE_START,               // State to handle capture before taking source piece
    DO_CAPTURE_SAFETY_CHECKS_TARGET, // Safety before moving to target
    DO_CAPTURE_MOVE_TO_TARGET,       // Move to target square to grab captured piece
    DO_CAPTURE_WAIT_TARGET,
    DO_CAPTURE_ROTATE_TARGET,       // Rotate gripper at target
    DO_CAPTURE_WAIT_ROTATE_TARGET,
    DO_CAPTURE_PERFORM_TAKE_EXTEND, // Grab the piece being captured
    DO_CAPTURE_WAIT_TAKE_EXTEND,
    DO_CAPTURE_PERFORM_TAKE_GRAB,
    DO_CAPTURE_WAIT_TAKE_GRAB,
    DO_CAPTURE_PERFORM_TAKE_RETRACT,
    DO_CAPTURE_WAIT_TAKE_RETRACT,
    DO_CAPTURE_MOVE_TO_DROPOFF,     // Move Cart/Orb to above capture zone
    DO_CAPTURE_WAIT_DROPOFF,
    DO_CAPTURE_FIND_SLOT,           // Determine which capture slot to use
    DO_CAPTURE_ROTATE_GRIPPER_CAPZONE, // Rotate gripper for capture zone dropoff
    DO_CAPTURE_WAIT_ROTATE_CAPZONE,
    DO_CAPTURE_MOVE_CAPTURE_STEPPER, // Move capture stepper to the chosen slot
    DO_CAPTURE_WAIT_CAPTURE_STEPPER,
    DO_CAPTURE_PERFORM_RELEASE_EXTEND,// Release the captured piece
    DO_CAPTURE_WAIT_RELEASE_EXTEND,
    DO_CAPTURE_PERFORM_RELEASE_OPEN,
    DO_CAPTURE_WAIT_RELEASE_OPEN,
    DO_CAPTURE_PERFORM_RELEASE_RETRACT,
    DO_CAPTURE_WAIT_RELEASE_RETRACT,
    DO_CAPTURE_COMPLETE,            // Finished handling the captured piece
    // Regular 'Take' Sequence (Now happens *after* potential capture handling)
    DO_PERFORM_TAKE_EXTEND,
    DO_WAIT_TAKE_EXTEND,
    DO_PERFORM_TAKE_GRAB,
    DO_WAIT_TAKE_GRAB,
    DO_PERFORM_TAKE_RETRACT,
    DO_WAIT_TAKE_RETRACT,
    // Move to Destination
    DO_SAFETY_CHECKS_DEST,
    DO_MOVE_CART_TO_DEST,
    DO_WAIT_CART_DEST,
    DO_ROTATE_GRIPPER_DEST,
    DO_WAIT_GRIPPER_DEST,
    DO_MOVE_STEPPERS_TO_DEST,
    DO_WAIT_STEPPERS_DEST,
    // Regular 'Release' Sequence
    DO_PERFORM_RELEASE_EXTEND,
    DO_WAIT_RELEASE_EXTEND,
    DO_PERFORM_RELEASE_OPEN,
    DO_WAIT_RELEASE_OPEN,
    DO_PERFORM_RELEASE_RETRACT,
    DO_WAIT_RELEASE_RETRACT,
    DO_COMPLETE,
    // ... (Manual Move States, ERROR_STATE) ...
    MANUAL_MOVE_START,
    MANUAL_MOVING,
    MANUAL_MOVE_STOP,
    ERROR_STATE
};


class MotionController {
public:
    MotionController();
    void setup();
    void update();

    bool isBusy();

    bool startMoveSequence(String fromLoc, String toLoc);
    bool startHomingSequence();
    void initializeCaptureZone(); // Call after homing
    // bool startManualMove(int actuatorId, bool direction);
    // bool stopManualMove(int actuatorId);
    MotionState getCurrentState() const; // <<<=== ADDED Getter Declaration

private:
    AccelStepper stepper1;
    AccelStepper stepper2;
    AccelStepper stepper3;
    Servo servo1;
    Servo servo2;

    MotionState currentState;
    unsigned long stateStartTime;

    String targetFromLoc;
    String targetToLoc;
    long targetOrb1, targetCart1, targetCapt1;
    long targetOrb2, targetCart2, targetCapt2;
    int targetRot1, targetRot2;
    // These use the globally defined LocationType now
    LocationType locType1;
    LocationType locType2;
    // Capture-specific data
    bool isCaptureMove;           // Flag set during parsing if 'to' is occupied
    int targetCaptureSlotIndex;   // Index (0-31) of the chosen slot
    long targetCaptureSlotPos;    // Stepper position for the chosen slot
    CapturedPieceInfo pieceBeingCaptured; // Store info about the piece removed

    // --- Capture Zone State ---
    CapturedPieceInfo capture_zone[32]; // Array representing the 32 slots
    
    bool homeCaptureHomed;
    bool homeCartHomed;
    bool homeOrbHomed;

    // Helper function declarations (ensure they match definitions)
    bool getTargetsForSquare(String square, long &orbTarget, long &cartTarget);
    bool getTargetForCapture(int captureSlot, long &captureTarget);
    // No need to declare private member functions if definitions in .cpp are ordered correctly,
    // but declaring them here doesn't hurt and can help readability.
    LocationType parseLocation(String locStr, long &orbTarget, long &cartTarget, long &captureTarget, int &gripperRotTarget);
    void enforceCartSafetyRotation(long targetCartPos);
    void enforceCaptureHomedForLowCart(long targetCartPos);
    void commandExtendActuator();
    void commandRetractActuator();
    void commandStopActuator();
    void commandGripperOpen();
    void commandGripperClose();
    void commandRotateGripper(int angle);
    String coordsToAlgebraic(int x, int y);
    int findFreeCaptureSlot(PieceColor color); // Helper to find an empty slot
    void executeStateMachine();


};

#endif // MOTION_CONTROLLER_H