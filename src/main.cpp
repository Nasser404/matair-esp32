#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <EasyNextionLibrary.h>
#include <board.h>
#include <enums.h>
#include <config.h>
#include <Preferences.h>

#include "hardware_pins.h"     
#include "MotionController.h" 
#include "nextion_handler.h"



Preferences prefs;
MotionController motionController; 
Board board;
NextionHandler nextionHandler(motionController, board);

char     ssid[SSID_MAX_LEN];
char     password[PWD_MAX_LEN];
char     websockets_server_host[HOST_MAX_LEN];
uint16_t websockets_server_port;
bool     proper_bootup = false;

////////////////////////////// TIMERS ///////////////////////////////
unsigned long network_timer = millis();
unsigned long timeout_timer = millis();
unsigned long timer = millis();
unsigned long pageRefreshTimer = millis();
/////////////////////////////////////////////////////////////////////

/////////////////////////////// WIFI AND SERVER ///////////////
using namespace websockets;
WebsocketsClient client;
bool CONNECTED_TO_WIFI      = false;
bool CONNECTED_TO_SERVER    = false;
bool hasIdentified = false;
////////////////////////////////////////////////

//////////////////////////// GAME STATE //////////////////////////
String ORB_CODE   = "";
String GAME_ID    = "";
String PLAYER_NAMES[2] = {"", ""};
bool IN_GAME = false;
///////////////////////////////////////////////////////////////////

//////// REBOOT & RESET ////////////
bool REBOOT_ASKED = false;
bool RESET_ASKED  = false;
///////////

std::pair<int, int> currentMoveFromCoords = {-1, -1};
std::pair<int, int> currentMoveToCoords = {-1, -1};

// --- Struct to Store Last Processed Command ---
struct LastCommandInfo {
    bool isValid = false;
    std::pair<int, int> fromCoords = {-1, -1};
    std::pair<int, int> toCoords = {-1, -1};
    bool wasCapture = false;

    // --- Fields for Special Moves ---
    bool isSpecialMove = false;
    SPECIAL_MOVES specialMoveType = SPECIAL_MOVES::NONE; 
    PieceType promotionPieceType = PieceType::QUEEN; 

};

LastCommandInfo lastCommandProcessed;   // Global instance to store the info
bool currentMoveIsCapture = false;      // Flag if the current command involved a capture
bool resetBoardAfterHoming = false;     // Flag to trigger board reset after homing completes


void readCredentials() {
    prefs.begin("orb_cfg", true); // read-only

    String s = prefs.getString("ssid", DEFAULT_SSID);
    String p = prefs.getString("pwd",  DEFAULT_PWD);
    String h = prefs.getString("host", DEFAULT_HOST);
  
    uint32_t port = prefs.getUInt("port", DEFAULT_PORT);
    proper_bootup = prefs.getBool("boot", false);

    prefs.end();
  
    s.toCharArray(ssid, SSID_MAX_LEN);
    p.toCharArray(password, PWD_MAX_LEN);
    h.toCharArray(websockets_server_host, HOST_MAX_LEN);
    websockets_server_port = uint16_t(port);
}

void writeCredentials() {
    prefs.begin("orb_cfg", false);  // RW
    prefs.putString("ssid",  String(ssid));
    prefs.putString("pwd",   String(password));
    prefs.putString("host",  String(websockets_server_host));
    prefs.putUInt  ("port",  websockets_server_port);
    prefs.putBool("bool",    proper_bootup);
    prefs.end();
}
void reboot() {
    String ns = nextionHandler.getString("txt_wifi_ssid.txt");
    String np = nextionHandler.getString("txt_wifi_pwd.txt");
    String nh = nextionHandler.getString("txt_ip.txt");
    int    npn = nextionHandler.getNumber("num_port.val");

    ns.toCharArray(ssid, SSID_MAX_LEN);
    np.toCharArray(password, PWD_MAX_LEN);
    nh.toCharArray(websockets_server_host, HOST_MAX_LEN);
    websockets_server_port = uint16_t(npn);

    if (board.isAtStartingPosition()) proper_bootup = true;

    // Persist in flash
    writeCredentials();
    delay(1000);
    ESP.restart();
}

String generate_orb_code() {
    String code = "";

    static const char alphanum[] =
    "0123456789"
    "ABCDEFHIJKLMNPQRTUVWXY";

    for (int i = 0; i < 4; ++i) {
        code += alphanum[random(sizeof(alphanum) - 1)];
    }
    
    return code;
    
}


// --- Send Orb Status Update ---
void send_orb_status_update() {
    if (!hasIdentified) return;

    JsonDocument orb_data;
    // Determine status based on motion controller
    unsigned short current_status = motionController.isBusy() ? OCCUPIED : IDLE;

    orb_data["type"]    =  ORB_DATA;
    orb_data["orb_code"]= ORB_CODE;
    orb_data["status"]  = current_status; 

    String json_data;
    serializeJson(orb_data, json_data);

    if (client.available()) { 
       client.send(json_data);
       Serial.print("Sent Orb Status Update: "); Serial.println(current_status == IDLE ? "IDLE" : "OCCUPIED");
    } else {
       Serial.println("Cannot send status update, client not available.");
    }
}

// --- Reset Orb Function ---
void reset_orb() {
    GAME_ID = ""; PLAYER_NAMES[0] = ""; PLAYER_NAMES[1] = "";
    IN_GAME = false;
    ORB_CODE = generate_orb_code();
    Serial.println("Reset requested. Initiating Homing Sequence first...");

    if (motionController.startHomingSequence()) {

        resetBoardAfterHoming = true; 
        send_orb_status_update(); // Send OCCUPIED (due to homing)

    } else {
         resetBoardAfterHoming = false;

         Serial.println("Could not start homing (already busy?). Board reset skipped.");
         nextionHandler.forceFullRefresh();
    }
    
}


bool connect_to_wifi() {

    // Connect to wifi
    WiFi.begin(ssid, password);
    // Wait some time to connect to wifi
    for(int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
        Serial.print(".");
        delay(1000);
    }
 
    // Check if connected to wifi
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("No Wifi!");
        return false;
    } else {
        Serial.println("Connected to Wifi");
        return true;

    }
}



// --- WebSocket Message Handler ---
void handle_data(WebsocketsMessage packet) {
    String json_data = packet.data();
    // Serial.print("Got Message: "); Serial.println(json_data); // Optional debug

    JsonDocument data;
    DeserializationError error = deserializeJson(data, json_data);
    if (error) { Serial.print(F("deserializeJson() failed: ")); Serial.println(error.f_str()); return; }

    int type = data["type"];
    
    switch (type) {
        case PING :
        
        timeout_timer = timer;
        client.send("{\"type\":2}"); // SEND PONG
        break;
        

        case IDENTIFICATION : 
        client.send("{\"type\":0,\"identifier\":0}"); // IDENTIFICATION
        Serial.println("INDENTIFICATION REQUEST");
        break;
        
        case ORB_CONNECT : {
            
            JsonDocument connect_data;

            Serial.println("ORB CONNECT");
            connect_data["orb_id"]     = ORB_ID;
            connect_data["type"]       = ORB_CONNECT;
            connect_data["orb_code"]   = ORB_CODE ;
            connect_data["orb_board"]  = board.getBoardString();
            String json_data;
            serializeJson(connect_data, json_data);

            client.send(json_data);
            hasIdentified = true;
        }
        break;

        case GAME_INFO :
        {
            IN_GAME = true;
            Serial.println("GAME INFO");
            GAME_ID = (const char*)data["info"]["game_id"];
            bool local_game = data["info"]["local_game"];
            
  
            if (local_game) {
                PLAYER_NAMES[0] = (const char*)data["info"]["white_player"];
                PLAYER_NAMES[1] = (const char*)data["info"]["black_player"];
            } else {
                PLAYER_NAMES[0] = (const char*)data["info"]["white_orb"];
                PLAYER_NAMES[1] = (const char*)data["info"]["black_orb"];
            }

             nextionHandler.updateGameInfo(GAME_ID, PLAYER_NAMES[0], PLAYER_NAMES[1]);

        }
        break;


        case ORB_RESET :
            IN_GAME = false;
            Serial.println("RESET INSTRUCTION");
            //current_status = OCCUPIED;
            RESET_ASKED = true;
        break;


        case MOVE : {
            Serial.println("Received MOVE instruction:");
            Serial.println(json_data);
            if (motionController.isBusy()) {
                Serial.println("!!! WARNING: MOVE ignored, controller busy."); return;
            }

            JsonArray fromArr = data["from"]; JsonArray toArr = data["to"];
            if (fromArr.size() != 2 || toArr.size() != 2) {
                Serial.println("Error: Invalid coordinate format."); return;
            }
            int xfrom = fromArr[0].as<int>(); int yfrom = fromArr[1].as<int>();
            int xto = toArr[0].as<int>(); int yto = toArr[1].as<int>();

             // --- Store Command Details ---
             lastCommandProcessed.isValid = true; // Assume valid until sequence fails
             lastCommandProcessed.fromCoords = {xfrom, yfrom};
             lastCommandProcessed.toCoords = {xto, yto};
             lastCommandProcessed.wasCapture = (board.grid[xto][yto] != nullptr);
             lastCommandProcessed.isSpecialMove = false; // Default
             lastCommandProcessed.specialMoveType = SPECIAL_MOVES::NONE; // Default
              
             // --- Check for Special Move Details ---
             if (data["special_move"].is<JsonVariant>() && data["special_move"].is<JsonObject>()) {
                 JsonObject specialMoveData = data["special_move"];
                 if (specialMoveData["type"].is<JsonVariant>()) {
                     int specialTypeInt = specialMoveData["type"].as<int>();
                     // Map integer from JSON to SPECIAL_MOVES enum
                     if (specialTypeInt == (int)SPECIAL_MOVES::PROMOTION) { // Check for PROMOTION
                         lastCommandProcessed.isSpecialMove = true;
                         lastCommandProcessed.specialMoveType = SPECIAL_MOVES::PROMOTION;
                         Serial.println("  *PROMOTION DETECTED*");
 
                         // Get the piece type to promote to 
                         if (specialMoveData["promoted_to_type"].is<JsonVariant>()) {
                             String promotedTypeStr = specialMoveData["promoted_to_type"].as<String>();
                             promotedTypeStr.toUpperCase();
                             if (promotedTypeStr == "QUEEN") lastCommandProcessed.promotionPieceType = PieceType::QUEEN;
                             else if (promotedTypeStr == "ROOK") lastCommandProcessed.promotionPieceType = PieceType::ROOK;
                             else if (promotedTypeStr == "BISHOP") lastCommandProcessed.promotionPieceType = PieceType::BISHOP;
                             else if (promotedTypeStr == "KNIGHT") lastCommandProcessed.promotionPieceType = PieceType::KNIGHT;
                             else lastCommandProcessed.promotionPieceType = PieceType::QUEEN; // Default to Queen on error
                             Serial.print("  Promoting to: "); Serial.println(promotedTypeStr);
                         } else {
                              lastCommandProcessed.promotionPieceType = PieceType::QUEEN; // Default if not specified
                              Serial.println("  Promotion type not specified, defaulting to QUEEN.");
                         }
                     }
                 }
             }
 
             // --- Convert to Algebraic & Start Sequence ---
             String fromAlg = board.getSquareString(lastCommandProcessed.fromCoords);
             String toAlg = board.getSquareString(lastCommandProcessed.toCoords);
             Serial.print("  Parsed Move: "); Serial.print(fromAlg); Serial.print(" -> "); Serial.println(toAlg);
             if (lastCommandProcessed.wasCapture) { Serial.println("  (Is a capture)"); }
 
             // Start the standard move sequence - MotionController doesn't need special handling for this version
             bool sequenceStarted = motionController.startMoveSequence(fromAlg, toAlg);
             if (sequenceStarted) {
                 Serial.println("  Physical move sequence initiated.");
                 send_orb_status_update();
             } else {
                 Serial.println("  Failed to initiate physical move sequence.");
                 lastCommandProcessed.isValid = false; // Invalidate data
             }
         } break;
    }
}
bool connect_to_server() {
    Serial.println("Connecting to server.");
    // try to connect to Websockets server
    bool connected = client.connect(websockets_server_host , websockets_server_port, "/");
    if(connected) {
        Serial.println("Connected to server !");

        // run callback when messages are received
        client.onMessage(handle_data);
        return true;

    } else {
        Serial.println("Unable to Connect to server");
        return false;
    }
 }
void disconnect_from_server() {
    client.close();
    IN_GAME = false;
    CONNECTED_TO_SERVER = false;
}

void setup() {
    Serial.begin(115200);

    readCredentials();
    Serial.printf("Loaded SSID: %s, PWD: %s, Server: %s:%u\n", ssid, password, websockets_server_host, websockets_server_port);


    ORB_CODE = generate_orb_code();
    Serial.print("Generated ORB Code: "); Serial.println(ORB_CODE);
    board.printBoard();
    
    motionController.setup(); 
    nextionHandler.setup();


    // Initial connection attempts
    CONNECTED_TO_WIFI = connect_to_wifi();
    if (CONNECTED_TO_WIFI) {
        CONNECTED_TO_SERVER = connect_to_server();
        if(CONNECTED_TO_SERVER) {
            timeout_timer = millis();
        } else IN_GAME = false;
    } else IN_GAME = false;


    nextionHandler.forceFullRefresh(); 
}

void loop() {
    timer = millis();
    
    nextionHandler.update(); // Check for Nextion events
    motionController.update(); //  Update Motion Controller
    
    // --------------------------------- CONTROLLER  ------------------------------------- //
    if (motionController.getCurrentState() == MOTION_IDLE) {
        unsigned int step3_pos = motionController.stepper3.currentPosition();
        bool b1= (digitalRead(BUTTON_PIN_1));
        bool b2 = (digitalRead(BUTTON_PIN_2));
        if ((b1 && !b2) && (step3_pos < ORB_MAX_POS)){
            motionController.stepper3.enableOutputs();
            motionController.stepper3.setSpeed(MANUAL_JOG_ORB_SPEED);
            motionController.stepper3.runSpeed();
        }
        else if ((!b1 && b2) && (step3_pos> ORB_MIN_POS)){
            motionController.stepper3.enableOutputs();
            motionController.stepper3.setSpeed(-MANUAL_JOG_ORB_SPEED);
            motionController.stepper3.runSpeed();

        }
        else {
            motionController.stepper3.setSpeed(0);
            motionController.stepper3.disableOutputs();
        };
    };

    // -------------------------------------- State Transition Detection & Actions  ---------------------------------------------
    /// HERE WE DO THING LIKE UPDATING NEXTION BOARD, SENDING ORB STATUS, UPDATING LOGICAL BOARD AFTER FINISHING A MOVE ////
    static MotionState lastMotionState = MOTION_IDLE;
    MotionState currentMotionState = motionController.getCurrentState();

    if (lastMotionState != MOTION_IDLE && currentMotionState == MOTION_IDLE) {
        Serial.print("MotionController transitioned from state "); Serial.print(lastMotionState);
        Serial.println(" to MOTION_IDLE.");
        send_orb_status_update();

        // --- Post-Sequence Updates ---
        if (lastMotionState == DO_COMPLETE) {
            std::pair<int, int> from, to;
            if (motionController.getResetSubMoveDetails(from, to)) { // Check if it was a reset sub-move
                Serial.print(board.getSquareString(from)); Serial.print(" -> "); Serial.println(board.getSquareString(to));

                nextionHandler.movePieceOnDisplay(from, to); // Update display
                board.movePiece(from, to);    // Update internal logic
                board.printBoard();

            } else if (lastCommandProcessed.isValid) { // NORMAL MOVE
             Serial.println("Executing Post-Move Update...");

             from = lastCommandProcessed.fromCoords;
             to = lastCommandProcessed.toCoords;
             String fromStr = board.getSquareString(from);
             String toStr = board.getSquareString(to);

             // --- Logical Board Update ---
             Serial.print("  Updating logical board for: "); Serial.print(fromStr); Serial.print(" -> "); Serial.println(toStr);
    
             delete board.grid[to.first][to.second]; 
             board.grid[to.first][to.second] = nullptr;

             // Get the piece that moved
             Piece* movingPiece = board.grid[from.first][from.second];
             if (movingPiece == nullptr) {
                Serial.println("!!! ERROR: No piece found at 'from' square for logical update!");
             } else {
                 // Remove piece pointer from 'from' square
                 board.grid[from.first][from.second] = nullptr;

                 // Check if it was a promotion
                 if (lastCommandProcessed.isSpecialMove && lastCommandProcessed.specialMoveType == SPECIAL_MOVES::PROMOTION) {
                     Serial.print("  Promotion! Deleting Pawn, Adding new piece type: ");
                     Serial.println((int)lastCommandProcessed.promotionPieceType); 

                     PieceColor color = movingPiece->getColor(); // Get color from the original pawn
                     delete movingPiece; // Delete the original Pawn object

                     // Add the NEW promoted piece to the 'to' square
                     board.addPiece(lastCommandProcessed.promotionPieceType, color, to);

                 } else {
                     // Normal move: Place the original piece pointer at the 'to' square
                     board.grid[to.first][to.second] = movingPiece;
                     // Update the piece's internal position tracker
                     movingPiece->setPosition(to);
                 }
             }
             board.printBoard(); // Print updated logical board state

             // --- Nextion Display Update ---
             Serial.print("  Updating Nextion display for: "); Serial.print(fromStr); Serial.print(" -> "); Serial.println(toStr);
         
             String nextionFromSquare = fromStr + ".picc";
             nextionHandler.setNumber(nextionFromSquare, 1); 

             
             String nextionToSquare = toStr + ".picc";
             int finalPieceNextionId = board.getSquareNextionId(to); 
             nextionHandler.setNumber(nextionToSquare, finalPieceNextionId);
             /// WE DONT USE THE nextionHandler.movePieceOnDisplay beauce last move could have been a promotion, if so we have to change the type of the piece
             Serial.println("  Post-move updates complete.");
            }

            lastCommandProcessed.isValid = false;
        } else if (lastMotionState == HOMING_COMPLETE) {
            Serial.println("Homing sequence completed.");
            // --- Check if we need to trigger board reset ---
            if (resetBoardAfterHoming) {

                Serial.println("Triggering Board Reset Sequence now...");
                resetBoardAfterHoming = false; // Clear the flag

                if (motionController.startBoardResetSequence()) send_orb_status_update(); // Send OCCUPIED (due to reset sequence)
                else Serial.println("Failed to start Board Reset sequence!");
             }
            lastCommandProcessed.isValid = false; // Ensure invalid after homing
        } else if (lastMotionState == RESET_COMPLETE) { //  Check if RESET finished
            
            Serial.println("Board Reset Sequence Completed.");
            lastCommandProcessed.isValid = false; // Ensure invalid after reset

            nextionHandler.forceFullRefresh();

       } else if (lastCommandProcessed.isValid){
            Serial.println("Became IDLE unexpectedly. Invalidating last command data.");
            lastCommandProcessed.isValid = false;
       }
    }
    //////////////////// WE UPDATE NEXTION BASE ON MOTION CONTROLLER STATE ////////////////////
    if (currentMotionState != lastMotionState) {
        switch (currentMotionState) {
            case RESET_START: 
            nextionHandler.changePage(RESET_IN_PROGRESS_SCREEN, true); 
            break;

            case RESET_P1_ITERATE_BOARD         : nextionHandler.updateResetStatus(motionController.resetProgress , "Clearing Board");      break;
            case RESET_P2_ITERATE_CZ            : nextionHandler.updateResetStatus(motionController.resetProgress , "Placing back pieces"); break;
            case RESET_P3_ITERATE_BOARD         : nextionHandler.updateResetStatus(motionController.resetProgress , "Final check");         break;
            case RESET_P4_HOME_CAPTURE_MOTOR    : nextionHandler.updateResetStatus(motionController.resetProgress , "Almost Done");         break;

            case RESET_COMPLETE :
            nextionHandler.changePage(HOME_SCREEN);
            break;
            case ERROR_STATE:
            nextionHandler.changePage(ERROR_STATE_SCREEN, true);
            break;
            
        }
    }


    lastMotionState = currentMotionState; // Update tracker


    // Handle WebSocket polling l
    if (timer - network_timer > 200) {
        if(CONNECTED_TO_SERVER && client.available()) {
            client.poll();
        }
        network_timer = timer;
    }




    // Handle WebSocket connection/timeout logic
    CONNECTED_TO_WIFI = (WiFi.status() == WL_CONNECTED);
    if (CONNECTED_TO_SERVER) {
        if ((timer - timeout_timer > 20000) || (WiFi.status() == WL_CONNECTED)) { // 20 second timeout
            Serial.println("TIMEOUT - No PING received from server.");
            disconnect_from_server();
            nextionHandler.changePage(CONNECTION_LOST_SCREEN);
        }
    } 

    bool can_reboot = ( (currentMotionState == MOTION_IDLE) || (currentMotionState == ERROR_STATE));
    if ((REBOOT_ASKED) && (can_reboot)) {
        REBOOT_ASKED = false;
        reboot();
    }

    if ((RESET_ASKED) && (can_reboot)) {
        RESET_ASKED = false;
        if (IN_GAME) disconnect_from_server();
        
        reset_orb();
    }
}



// ===================================================================================
// ========================== NEXTION TRIGGER FUNCTIONS  =============================
// ===================================================================================
void trigger0()  { nextionHandler.handleTrigger(0); }
void trigger1()  { nextionHandler.handleTrigger(1); }
void trigger2()  { nextionHandler.handleTrigger(2); }
void trigger3()  { nextionHandler.handleTrigger(3); }
void trigger4()  { nextionHandler.handleTrigger(4); }
void trigger5()  { nextionHandler.handleTrigger(5); }
void trigger6()  { nextionHandler.handleTrigger(6); }
void trigger7()  { nextionHandler.handleTrigger(7); }
void trigger8()  { nextionHandler.handleTrigger(8); }
void trigger9()  { nextionHandler.handleTrigger(9); }
void trigger10() { nextionHandler.handleTrigger(10); }
void trigger11() { nextionHandler.handleTrigger(11); }
void trigger12() { nextionHandler.handleTrigger(12); }
void trigger13() { nextionHandler.handleTrigger(13); }
void trigger14() { nextionHandler.handleTrigger(14); }
void trigger15() { nextionHandler.handleTrigger(15); }
void trigger16() { nextionHandler.handleTrigger(16); }
void trigger17() { nextionHandler.handleTrigger(17); }
void trigger18() { nextionHandler.handleTrigger(18); }
void trigger19() { nextionHandler.handleTrigger(19); }
void trigger20() { nextionHandler.handleTrigger(20); }
void trigger21() { nextionHandler.handleTrigger(21); }
void trigger22() { nextionHandler.handleTrigger(22); }
void trigger23() { nextionHandler.handleTrigger(23); }
void trigger24() { nextionHandler.handleTrigger(24); }
void trigger25() { nextionHandler.handleTrigger(25); }
void trigger40() { nextionHandler.handleTrigger(40); }