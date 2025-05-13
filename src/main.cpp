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

Preferences prefs;

char     ssid[SSID_MAX_LEN];
char     password[PWD_MAX_LEN];
char     websockets_server_host[HOST_MAX_LEN];
uint16_t websockets_server_port;

bool hasIdentified = false;

void readCredentials() {
    prefs.begin("orb_cfg", true); // read-only

    String s = prefs.getString("ssid", "");
    String p = prefs.getString("pwd",  "");
    String h = prefs.getString("host", "");
  
    uint32_t port = prefs.getUInt("port", 0);
  
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
    prefs.end();
  }

void force_nextion_refresh();

/////////////////////////////////// NEXTION //////////////
EasyNex nextion(Serial2);
bool newPageLoaded = false; // true when the page is first loaded ( lastCurrentPageId != currentPageId )
bool forceNextionFullRefresh = false; 
/////////////////////////////////////

////////////////////////////// TIMERS ///////////////////////////////
unsigned long network_timer = millis();
unsigned long timeout_timer = millis();
unsigned long timer = millis();
/////////////////////////////////////////////////////////////////////

/////////////////////////////// WIFI AND SERVER ///////////////
using namespace websockets;
WebsocketsClient client;
bool CONNECTED_TO_WIFI      = false;
bool CONNECTED_TO_SERVER    = false;
////////////////////////////////////////////////

//////////////////////////// ROBOT STATE //////////////////////////
String ORB_CODE   = "";
String GAME_ID    = "";
String PLAYER_NAMES[2] = {"", ""};
Board board; // Logical board state
MotionController motionController; 
///////////////////////////////////////////////////////////////////

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

    ORB_CODE = generate_orb_code();
    force_nextion_refresh();
    Serial.println("Reset requested. Initiating Homing Sequence first...");
    if (motionController.startHomingSequence()) {
        resetBoardAfterHoming = true; 
        send_orb_status_update(); // Send OCCUPIED (due to homing)
    } else {
         Serial.println("Could not start homing (already busy?). Board reset skipped.");
         resetBoardAfterHoming = false;
    }
    
}


void move_nextion_piece(std::pair<int, int> from, std::pair<int, int> to) {
    if (nextion.currentPageId != BOARD_SCREEN) nextion.writeStr("page board_screen"); // IF NOT IN BOARD SCREEN, GO TO BOARD SCREEN

    int xfrom = from.first, yfrom = from.second;
    int xto = to.first, yto = to.second;
           
    String from_square  = board.getSquareString({xfrom, yfrom})     + ".picc";
    String to_square    = board.getSquareString({xto, yto})         + ".picc";
    int id              = board.getSquareNextionId({xfrom, yfrom});


    nextion.writeNum(to_square, id);  
    nextion.writeNum(from_square, 1);

}
void update_nextion_game_info() {
    if (nextion.currentPageId != BOARD_SCREEN) nextion.writeStr("page board_screen"); // IF NOT IN BOARD SCREEN, GO TO BOARD SCREEN
    nextion.writeStr("txt_game_id.txt", GAME_ID);
    nextion.writeStr("txt_p1_name.txt", PLAYER_NAMES[0]);
    nextion.writeStr("txt_p2_name.txt", PLAYER_NAMES[1]);

}


void load_nextion_page(){ // This function's purpose is to update the values of a new page when is first loaded,
        // and refreshing all the components with the current values as Nextion shows the Attribute val.
    if (forceNextionFullRefresh) {
        nextion.lastCurrentPageId = -1; // Force all components to update
        forceNextionFullRefresh = false; // Reset flag
        Serial.println("Forcing Nextion full page refresh.");
    }
    if(nextion.currentPageId != nextion.lastCurrentPageId){ // If the two variables are different, means a new page is loaded.

    newPageLoaded = true;    // A new page is loaded
                // This variable is used as an argument at the if() statement on the refreshPageXX() voids, 
                // in order when is true to update all the values on the page with their current values
                // with out run a comparison with the last value.

    switch(nextion.currentPageId) {
        case HOME_SCREEN :
            nextion.writeStr("txt_orb_code.txt", ORB_CODE);

            if (CONNECTED_TO_WIFI) {
                if (CONNECTED_TO_SERVER) {
                    nextion.writeStr("txt_connected.txt", "connected");
                    nextion.writeNum("txt_connected.pco", 2016);
                } else {
                    nextion.writeStr("txt_connected.txt", "no server");
                    nextion.writeNum("txt_connected.pco", 52000);  
                }
            } else {
                nextion.writeStr("txt_connected.txt", "no wifi");
                nextion.writeNum("txt_connected.pco", 63488);
            }
        break;

        case BOARD_SCREEN:
            for (int j = 0; j < 8; ++j) { // LOAD BOARD
                for (int i = 0; i < 8; ++i) {
                    String square  = board.getSquareString({i, j})     + ".picc";
                    int id         = board.getSquareNextionId({i, j});

                    nextion.writeNum(square, id);  
                }
            }
            nextion.writeStr("txt_orb_code.txt", ORB_CODE);
            nextion.writeStr("txt_game_id.txt", GAME_ID);
            nextion.writeStr("txt_p1_name.txt", PLAYER_NAMES[0]);
            nextion.writeStr("txt_p2_name.txt", PLAYER_NAMES[1]);
        break;

        case CONTROL_SCREEN:
        break;

        case SETTING_SCREEN:
        nextion.writeStr("txt_wifi_ssid.txt",   ssid);
        nextion.writeStr("txt_wifi_pwd.txt",    password);
        nextion.writeStr("txt_ip.txt",          websockets_server_host);
        nextion.writeNum("num_port.val",       websockets_server_port);
        break;
    }

    newPageLoaded = false;  // After we have updated the new page for the first time, we update the variable to false.
            // Now the values updated ONLY if the new value is different from the last Sent value.
            // See void refreshPage0()

    nextion.lastCurrentPageId = nextion.currentPageId; // Afer the refresh of the new page We make them equal,
                                    // in order to identify the next page change.
    }
}



void force_nextion_refresh() {
    forceNextionFullRefresh=true;
    load_nextion_page();
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

            update_nextion_game_info();

        }
        break;


        case ORB_RESET :
            Serial.println("RESET INSTRUCTION");
            //current_status = OCCUPIED;
            reset_orb();
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
             if (data.containsKey("special_move") && data["special_move"].is<JsonObject>()) {
                 JsonObject specialMoveData = data["special_move"];
                 if (specialMoveData.containsKey("type")) {
                     int specialTypeInt = specialMoveData["type"].as<int>();
                     // Map integer from JSON to SPECIAL_MOVES enum
                     if (specialTypeInt == (int)SPECIAL_MOVES::PROMOTION) { // Check for PROMOTION
                         lastCommandProcessed.isSpecialMove = true;
                         lastCommandProcessed.specialMoveType = SPECIAL_MOVES::PROMOTION;
                         Serial.println("  *PROMOTION DETECTED*");
 
                         // Get the piece type to promote to 
                         if (specialMoveData.containsKey("promoted_to_type")) {
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


void setup() {
    Serial.begin(115200);
    nextion.begin(9600);

    readCredentials();
    Serial.printf("Loaded SSID: %s, PWD: %s, Server: %s:%u\n", ssid, password, websockets_server_host, websockets_server_port);

    motionController.setup(); // <<<=== Setup Motion Controller Hardware

    ORB_CODE = generate_orb_code();
    Serial.print("Generated ORB Code: "); Serial.println(ORB_CODE);
    board.printBoard(); // Print initial logical board

    nextion.lastCurrentPageId = -1; 
    nextion.writeStr("page home_screen"); 
    load_nextion_page();

    // Initial connection attempts
    CONNECTED_TO_WIFI = connect_to_wifi();
    if (CONNECTED_TO_WIFI) {
        CONNECTED_TO_SERVER = connect_to_server();
        if(CONNECTED_TO_SERVER) {
            timeout_timer = millis();
        }
    }

    force_nextion_refresh();
}

void loop() {
    timer = millis();
    nextion.NextionListen(); // Check for Nextion events

    motionController.update(); //  Update Motion Controller
    
    // -- Manual move (with controller) --
    if (motionController.getCurrentState() == MOTION_IDLE) {
        unsigned int step3_pos = motionController.stepper3.currentPosition();
        bool b1= (digitalRead(BUTTON_PIN_1));
        bool b2 = (digitalRead(BUTTON_PIN_2));
        if ((b1 && !b2) && (step3_pos < ORB_MANUAL_MAX_POS)){
            motionController.stepper3.enableOutputs();
            motionController.stepper3.setSpeed(MANUAL_ORB_SPEED);
            motionController.stepper3.runSpeed();
        }
        else if ((!b1 && b2) && (step3_pos> ORB_MANUAL_MIN_POS)){
            motionController.stepper3.enableOutputs();
            motionController.stepper3.setSpeed(-MANUAL_ORB_SPEED);
            motionController.stepper3.runSpeed();

        }
        else {
            motionController.stepper3.setSpeed(0);
            motionController.stepper3.disableOutputs();
        };
    };

    // --- State Transition Detection & Actions ---
    static MotionState lastMotionState = MOTION_IDLE;
    MotionState currentMotionState = motionController.getCurrentState();

    if (lastMotionState != MOTION_IDLE && currentMotionState == MOTION_IDLE) {
        Serial.print("MotionController transitioned from state "); Serial.print(lastMotionState);
        Serial.println(" to MOTION_IDLE.");
        send_orb_status_update();

        // --- Post-Sequence Updates ---
        if (lastMotionState == DO_COMPLETE)
        {
            std::pair<int, int> from, to;
            if (motionController.getResetSubMoveDetails(from, to)) { // Check if it was a P3 reset sub-move
                Serial.println("  DO_COMPLETE was part of RESET Phase 3 sequence.");
                Serial.print("  Logically updating board for P3 move: ");
                Serial.print(board.getSquareString(from)); Serial.print(" -> "); Serial.println(board.getSquareString(to));

                move_nextion_piece(from, to); // Update display
                board.movePiece(from, to);    // Update internal logic
                board.printBoard();

            } else if (lastCommandProcessed.isValid) {
             Serial.println("Executing Post-Move/Promotion Updates...");

             from = lastCommandProcessed.fromCoords;
             to = lastCommandProcessed.toCoords;
             String fromStr = board.getSquareString(from);
             String toStr = board.getSquareString(to);

             // --- Logical Board Update ---
             Serial.print("  Updating logical board for: "); Serial.print(fromStr); Serial.print(" -> "); Serial.println(toStr);
        
             delete board.grid[to.first][to.second];
             board.grid[to.first][to.second] = nullptr;

             // Get the piece that moved (should be the pawn)
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
             nextion.writeNum(nextionFromSquare, 1); 

             
             String nextionToSquare = toStr + ".picc";
             int finalPieceNextionId = board.getSquareNextionId(to); 
             nextion.writeNum(nextionToSquare, finalPieceNextionId);
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

            if (nextion.currentPageId == BOARD_SCREEN) forceNextionFullRefresh = true;

       } else if (lastCommandProcessed.isValid){
            Serial.println("Became IDLE unexpectedly. Invalidating last command data.");
            lastCommandProcessed.isValid = false;
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
    if (CONNECTED_TO_SERVER) {
        if (timer - timeout_timer > 20000) { // 20 second timeout
            Serial.println("TIMEOUT - No PING received from server.");
            CONNECTED_TO_SERVER = false;
            client.close();
            // Update Nextion status immediately
            if(nextion.currentPageId == HOME_SCREEN) { 
                nextion.writeStr("txt_connected.txt", "no server");
                nextion.writeNum("txt_connected.pco", 64512); // Orange/Yellow
            }
        }
    } 


    // Refresh Nextion page contents
    load_nextion_page();
}


// --- Nextion Trigger Functions ---

void trigger0(){     String ns = nextion.readStr("txt_wifi_ssid.txt");
    String np = nextion.readStr("txt_wifi_pwd.txt");
    String nh = nextion.readStr("txt_ip.txt");
    int    npn = nextion.readNumber("num_port.val");

    ns.toCharArray(ssid, SSID_MAX_LEN);
    np.toCharArray(password, PWD_MAX_LEN);
    nh.toCharArray(websockets_server_host, HOST_MAX_LEN);
    websockets_server_port = uint16_t(npn);

    // Persist in flash
    writeCredentials();
    delay(1000);
    ESP.restart();
}

void trigger1() {  Serial.println("Nextion Home Button Pressed.");
    if (!motionController.isBusy()) {
        motionController.startHomingSequence();
    } else {
        Serial.println("Cannot Home: Motion Controller is busy.");
    }
}

// === CART Stepper ===
void trigger2() { // Cart + PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Cart + PRESS");
        motionController.startManualJog(ManualActuator::CART, true);
    }
}
void trigger3() { // Cart + RELEASE
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Cart + RELEASE");
        motionController.stopManualJog(ManualActuator::CART);
    }
}
void trigger4() { // Cart - PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Cart - PRESS");
        motionController.startManualJog(ManualActuator::CART, false);
    }
}
void trigger5() { // Cart - RELEASE
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Cart - RELEASE");
        motionController.stopManualJog(ManualActuator::CART);
    }
}

// === ORB Stepper ===
void trigger6() { // Orb + PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Orb + PRESS");
        motionController.startManualJog(ManualActuator::ORB, true);
    }
}
void trigger7() { // Orb + RELEASE
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Orb + RELEASE");
        motionController.stopManualJog(ManualActuator::ORB);
    }
}
void trigger8() { // Orb - PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Orb - PRESS");
        motionController.startManualJog(ManualActuator::ORB, false);
    }
}
void trigger9() { // Orb - RELEASE
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Orb - RELEASE");
        motionController.stopManualJog(ManualActuator::ORB);
    }
}

// === CAPTURE Stepper ===
void trigger10() { // Capture + PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Capture + PRESS");
        motionController.startManualJog(ManualActuator::CAPTURE, true);
    }
}
void trigger11() { // Capture + RELEASE
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Capture + RELEASE");
        motionController.stopManualJog(ManualActuator::CAPTURE);
    }
}
void trigger12() { // Capture - PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Capture - PRESS");
        motionController.startManualJog(ManualActuator::CAPTURE, false);
    }
}
void trigger13() { // Capture - RELEASE
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Capture - RELEASE");
        motionController.stopManualJog(ManualActuator::CAPTURE);
    }
}

// === GRIPPER ROTATION (Servo1) ===
void trigger14() { // Gripper Rot + PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Gripper Rot + PRESS (Step)");
        motionController.startManualJog(ManualActuator::GRIPPER_ROTATION, true); // True = more positive angle
    }
}
void trigger15() { // Gripper Rot + RELEASE - Does nothing for stepped servo control
    if (nextion.currentPageId == CONTROL_SCREEN) {
    }
}
void trigger16() { // Gripper Rot - PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Gripper Rot - PRESS (Step)");
        motionController.startManualJog(ManualActuator::GRIPPER_ROTATION, false); // False = more negative angle
    }
}
void trigger17() { // Gripper Rot - RELEASE - Does nothing
    if (nextion.currentPageId == CONTROL_SCREEN) {;
    }
}

// === LINEAR ACTUATOR ===
void trigger18() { // Linear Actuator + PRESS (Extend)
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Linear Actuator + PRESS (Extend)");
        motionController.startManualJog(ManualActuator::LINEAR_ACTUATOR, true); // True = Extend
    }
}
void trigger19() { // Linear Actuator + RELEASE (Stop Extend)
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Linear Actuator + RELEASE (Stop)");
        motionController.stopManualJog(ManualActuator::LINEAR_ACTUATOR);
    }
}
void trigger20() { // Linear Actuator - PRESS (Retract)
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Linear Actuator - PRESS (Retract)");
        motionController.startManualJog(ManualActuator::LINEAR_ACTUATOR, false); // False = Retract
    }
}
void trigger21() { // Linear Actuator - RELEASE (Stop Retract)
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Linear Actuator - RELEASE (Stop)");
        motionController.stopManualJog(ManualActuator::LINEAR_ACTUATOR);
    }
}

// === GRIPPER OPEN/CLOSE (Servo2) ===
void trigger22() { // Gripper Open + PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Gripper Open + PRESS (Step)");
        motionController.startManualJog(ManualActuator::GRIPPER_OPEN_CLOSE, true); 
    }
}
void trigger23() { // Gripper Open + RELEASE - Does nothing
    if (nextion.currentPageId == CONTROL_SCREEN) {
    }
}
void trigger24() { // Gripper Close - PRESS
    if (nextion.currentPageId == CONTROL_SCREEN) {
        Serial.println("Nextion: Gripper Close - PRESS (Step)");
        motionController.startManualJog(ManualActuator::GRIPPER_OPEN_CLOSE, false);
    }
}
void trigger25() { // Gripper Close - RELEASE
    if (nextion.currentPageId == CONTROL_SCREEN) {
    }
}
