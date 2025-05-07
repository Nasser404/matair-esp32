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
const float HOMING_SPEED_CAPTURE = 1000;
const float HOMING_SPEED_CART_ORB = 1000;
const float HOMING_ACCEL = 1500;

const int GRIPPER_ROT_BOARD = 180;
const int GRIPPER_ROT_CAPTURE = 62;
const long CART_SAFETY_THRESHOLD = 2250;
const long CART_CAPTURE_HOME_THRESHOLD = 800;
const long CART_CAPTURE_POS = 2250;

const int GripperOpen = 160;
const int GripperClose = 50;

const unsigned long ACTUATOR_TRAVEL_TIME_MS = 650;
const float MANUAL_STEPPER_SPEED = 3000; // Speed for manual button control (adjust as needed)

const long CAPTURE_HOME_BACKUP_STEPS = 200; // Adjust as needed

const long ORB_MANUAL_MIN_POS = 0;
const long ORB_MANUAL_MAX_POS = 6000;
const unsigned int MANUAL_ORB_SPEED = 300;
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
    HOMING_CAPTURE_START_BACKUP_MOVE, // <<< NEW: Start backing up
    HOMING_CAPTURE_WAIT_BACKUP,       // <<< NEW: Wait for backup to complete
    HOMING_CAPTURE_START_HOME_MOVE,   // <<< RENAMED from HOMING_CAPTURE_START_MOVE
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
    // Regular 'Take' Sequence (Modified for Retract Check)
    DO_PERFORM_TAKE_EXTEND,
    DO_WAIT_TAKE_EXTEND,
    DO_PERFORM_TAKE_GRAB,
    DO_WAIT_TAKE_GRAB,
    DO_PERFORM_TAKE_RETRACT_INITIAL,  // <<< Start initial timed retract
    DO_WAIT_TAKE_RETRACT_INITIAL,   // <<< Wait for initial timed retract
    DO_CHECK_TAKE_RETRACT_SENSOR,   // <<< Check the sensor pin
    DO_RECOVER_TAKE_RETRACT_ROTATE, // <<< Recovery: Rotate slightly
    DO_WAIT_RECOVER_ROTATE,         // <<< Wait for rotation
    DO_RECOVER_TAKE_RETRACT_RETRY,  // <<< Retry retract after rotation
    DO_WAIT_RECOVER_RETRY,          // <<< Wait for retry retract
    DO_RECOVER_TAKE_RETRACT_EXTEND, // <<< Recovery: Extend slightly
    DO_WAIT_RECOVER_EXTEND,         // <<< Wait for extend
    DO_RECOVER_TAKE_RETRACT_FINAL,  // <<< Final retract attempt after extend
    DO_WAIT_RECOVER_FINAL,          // <<< Wait for final retract
    DO_TAKE_RETRACT_CONFIRMED,      // <<< State entered when retraction is confirmed
    DO_TAKE_RETRACT_FAILED,         // <<< State entered if recovery fails
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
    // --- FULL BOARD RESET STATES ---
    RESET_START,
    // === Phase 1: Clear ALL Pieces from Board to Capture Zone ===
    RESET_P1_ITERATE_BOARD,         // Iterate through board squares (0-63)
    RESET_P1_CHECK_SQUARE_FOR_CLEAR,// Check if piece exists to be cleared
    RESET_P1_MOVE_TO_GRAB_FROM_BOARD,
    RESET_P1_WAIT_GRAB_POS_BOARD,
    RESET_P1_ROTATE_FOR_GRAB_BOARD,
    RESET_P1_WAIT_ROTATE_GRAB_BOARD,
    RESET_P1_GRAB_EXTEND_BOARD,
    RESET_P1_WAIT_GRAB_EXTEND_BOARD,
    RESET_P1_GRAB_CLOSE_BOARD,
    RESET_P1_WAIT_GRAB_CLOSE_BOARD,
    RESET_P1_GRAB_RETRACT_BOARD,      // Needs sensor check!
    RESET_P1_WAIT_GRAB_RETRACT_BOARD, // Needs sensor check!
    RESET_P1_GRAB_CONFIRMED_BOARD,
    RESET_P1_GRAB_FAILED_BOARD,
    RESET_P1_ROTATE_AWAY_BOARD_TO_CZ,
    RESET_P1_WAIT_ROTATE_AWAY_BOARD_TO_CZ,
    RESET_P1_MOVE_TO_CZ_DROPOFF,
    RESET_P1_WAIT_CZ_DROPOFF,
    RESET_P1_FIND_AVAILABLE_CZ_SLOT,
    RESET_P1_MOVE_CZ_MOTOR_TO_SLOT,
    RESET_P1_WAIT_CZ_MOTOR,
    RESET_P1_ROTATE_FOR_CZ_RELEASE,
    RESET_P1_WAIT_ROTATE_CZ_RELEASE,
    RESET_P1_RELEASE_EXTEND_IN_CZ,
    RESET_P1_WAIT_RELEASE_EXTEND_IN_CZ,
    RESET_P1_RELEASE_OPEN_IN_CZ,
    RESET_P1_WAIT_RELEASE_OPEN_IN_CZ,
    RESET_P1_RELEASE_RETRACT_IN_CZ,
    RESET_P1_WAIT_RELEASE_RETRACT_IN_CZ,
    RESET_P1_UPDATE_LOGIC_AND_CZ_ARRAY, // Update board.grid and capture_zone
    RESET_P1_PIECE_CLEARED_TO_CZ,       // Finished clearing one piece

    // === Phase 2: Place ALL Pieces from Capture Zone to Board Home ===
    RESET_P2_START,                 // Entry point for Phase 2
    RESET_P2_ITERATE_CZ,            // Iterate through capture_zone (0-31)
    RESET_P2_CHECK_CZ_PIECE,        // Check piece in CZ slot & find available home on board
    RESET_P2_MOVE_TO_CZ_SLOT,       // Move Cart/Capture to grab piece from slot
    RESET_P2_WAIT_CZ_SLOT_POS,
    RESET_P2_ROTATE_FOR_CZ_GRAB,
    RESET_P2_WAIT_ROTATE_CZ_GRAB,
    RESET_P2_GRAB_EXTEND_FROM_CZ,   // Renamed for clarity
    RESET_P2_WAIT_GRAB_EXTEND_FROM_CZ,
    RESET_P2_GRAB_CLOSE_FROM_CZ,
    RESET_P2_WAIT_GRAB_CLOSE_FROM_CZ,
    RESET_P2_GRAB_RETRACT_FROM_CZ,    // Needs sensor check!
    RESET_P2_WAIT_GRAB_RETRACT_FROM_CZ,// Needs sensor check!
    RESET_P2_CZ_GRAB_CONFIRMED,
    RESET_P2_CZ_GRAB_FAILED,
    RESET_P2_UPDATE_CZ_ARRAY_AS_FREE, // Mark the CZ slot as free NOW
    RESET_P2_ROTATE_AWAY_CZ_TO_BOARD, // Rotate gripper back to board angle
    RESET_P2_WAIT_ROTATE_AWAY_CZ_TO_BOARD,
    RESET_P2_MOVE_TO_HOME_SQUARE,   // Move Cart/Orb to the piece's chosen home square
    RESET_P2_WAIT_HOME_SQUARE_POS,
    RESET_P2_ROTATE_FOR_HOME_RELEASE,
    RESET_P2_WAIT_ROTATE_HOME_RELEASE,
    RESET_P2_RELEASE_EXTEND_ON_BOARD, // Renamed for clarity
    RESET_P2_WAIT_RELEASE_EXTEND_ON_BOARD,
    RESET_P2_RELEASE_OPEN_ON_BOARD,
    RESET_P2_WAIT_RELEASE_OPEN_ON_BOARD,
    RESET_P2_RELEASE_RETRACT_ON_BOARD,
    RESET_P2_WAIT_RELEASE_RETRACT_ON_BOARD,
    RESET_P2_UPDATE_LOGICAL_BOARD,  // Add piece to board.grid
    RESET_P2_PIECE_PLACED_ON_BOARD, // Finished placing one piece from CZ

    // === Phase 3: Place Misplaced Pieces from Board to Home ===
    RESET_P3_START,
    RESET_P3_ITERATE_BOARD,         // Iterate through board squares (0-63)
    RESET_P3_CHECK_BOARD_PIECE,     // Check if piece on square needs to be moved home
    RESET_P3_INITIATE_MOVE_TO_HOME, // Starts a standard move sequence (DO_...)
    RESET_P3_WAIT_MOVE_TO_HOME,     // Waits for the DO_... sequence to complete
    RESET_P3_PIECE_HOMED,           // Finished moving one piece home

    // === Phase 4: Finish ===
    RESET_P4_HOME_CAPTURE_MOTOR,
    RESET_P4_WAIT_CAPTURE_MOTOR,
    RESET_P4_LOGICAL_BOARD_RESET,
    RESET_P4_UPDATE_NEXTION,
    RESET_COMPLETE,
    // ... (Manual Move States, ERROR_STATE) ...
    MANUAL_MOVE_START,
    MANUAL_MOVING,
    MANUAL_MOVE_STOP,
    ERROR_STATE
};
const float MANUAL_JOG_CART_SPEED = 1500;
const float MANUAL_JOG_ORB_SPEED = 1000;
const float MANUAL_JOG_CAPTURE_SPEED = 800;
const int   MANUAL_JOG_SERVO_INCREMENT = 3; // Degrees per jog step for servos
enum class ManualActuator {
    CART,
    ORB,
    CAPTURE,
    GRIPPER_ROTATION, // Servo1
    GRIPPER_OPEN_CLOSE, // Servo2
    LINEAR_ACTUATOR
};
class MotionController {
public:
    MotionController();
    void setup();
    void update();

    bool isBusy();
 // --- Manual Jogging Methods ---
    bool startManualJog(ManualActuator actuator, bool positiveDirection);
    bool stopManualJog(ManualActuator actuator); // Could be generic stopAllJogs
    bool stopAllManualJogs();
    bool startMoveSequence(String fromLoc, String toLoc, bool isSubSequenceCall = false); // New
    bool startHomingSequence();
    void initializeCaptureZone();
    void resetInternalCaptureZoneState(); // <<<=== ADD THIS METHOD
    bool startBoardResetSequence();
    MotionState getCurrentState() const; // <<<=== ADDED Getter Declaration
    bool getResetSubMoveDetails(std::pair<int, int>& from, std::pair<int, int>& to);

    AccelStepper stepper3;


private:
ManualActuator currentJoggingActuator;
bool jogDirectionPositive;
    AccelStepper stepper1;
    AccelStepper stepper2;
    
    Servo servo1;
    Servo servo2;

    MotionState currentState;
    MotionState stateToReturnToAfterSubSequence; // <<< ADD THIS
    bool subSequenceIsActive;                    // <<< ADD THIS
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
    int retractRetryCount;
    
    bool homeCaptureHomed;
    bool homeCartHomed;
    bool homeOrbHomed;

    // --- Reset State Tracking ---
    int resetBoardIterator;
    int resetCZIterator;
    std::pair<int, int> reset_currentBoardCoords;
    String reset_currentBoardAlg;
    CapturedPieceInfo reset_pieceBeingMoved; // For P1 & P2 when dealing with piece info
    int reset_targetCZSlotIndex;
    long reset_targetCZSlotPos;
    std::pair<int, int> reset_targetHomeSquareCoords;
    String reset_targetHomeSquareAlg;
    bool resetP3_subMoveInProgress; 
    bool lastMoveWasResetSubMoveFlag;
    std::pair<int, int> resetSubMoveFromCoords; // Store FROM for P3 sub-move
    std::pair<int, int> resetSubMoveToCoords;   // Store TO for P3 sub-move
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
    std::pair<int, int> getUnambiguousHomeSquare(PieceType type, PieceColor color);
    std::vector<std::pair<int, int>> getPotentialHomeSquares(PieceType type, PieceColor color);
    void executeStateMachine();


};

#endif // MOTION_CONTROLLER_H