// --- START OF FILE main.cpp ---

#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <EasyNextionLibrary.h>
#include <board.h>
#include <enums.h>
#include <config.h>
#include <Preferences.h>

#include "hardware_pins.h"     // Include hardware pins
#include "MotionController.h" // Include our new motion controller

Preferences prefs;

char     ssid[SSID_MAX_LEN];
char     password[PWD_MAX_LEN];
char     websockets_server_host[HOST_MAX_LEN];
uint16_t websockets_server_port;
bool hasIdentified = false;
void readCredentials() {
    prefs.begin("orb_cfg", true); // read-only
    // retrieve with default = empty
    String s = prefs.getString("ssid", "");
    String p = prefs.getString("pwd",  "");
    String h = prefs.getString("host", "");
  
    // integers come back as `uint32_t`
    uint32_t port = prefs.getUInt("port", 0);
  
    prefs.end();
  
    // copy into C-strings (or just hold as Strings if you like)
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


/////////////////////////////////// NEXTION //////////////
EasyNex nextion(Serial2);
bool newPageLoaded = false; // true when the page is first loaded ( lastCurrentPageId != currentPageId )
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
// unsigned short status = OCCUPIED; // Now managed implicitly by MotionController busy state
Board board; // Logical board state
MotionController motionController; // <<<=== Instantiate Motion Controller
///////////////////////////////////////////////////////////////////
std::pair<int, int> currentMoveFromCoords = {-1, -1};
std::pair<int, int> currentMoveToCoords = {-1, -1};
// --- Struct to Store Last Processed Command ---
struct LastCommandInfo {
    bool isValid = false; // Flag to indicate if the data is for a valid, processed command
    std::pair<int, int> fromCoords = {-1, -1};
    std::pair<int, int> toCoords = {-1, -1};
    bool wasCapture = false; // Did the original 'to' square have a piece?
};

LastCommandInfo lastCommandProcessed; // Global instance to store the info
bool currentMoveIsCapture = false; // Flag if the current command involved a capture
// ... (generate_orb_code remains the same) ...
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
// Call this when motion starts or stops
void send_orb_status_update() {
    if (!hasIdentified) return;

    JsonDocument orb_data;
    // Determine status based on motion controller
    unsigned short current_status = motionController.isBusy() ? OCCUPIED : IDLE;

    orb_data["type"]    =  ORB_DATA;
    orb_data["orb_code"]= ORB_CODE;
    orb_data["status"]  = current_status; // Use status derived from controller

    String json_data;
    serializeJson(orb_data, json_data);

    if (client.available()) { // Check if client is valid before sending
       client.send(json_data);
       Serial.print("Sent Orb Status Update: "); Serial.println(current_status == IDLE ? "IDLE" : "OCCUPIED");
    } else {
       Serial.println("Cannot send status update, client not available.");
    }
}

// --- Reset Orb Function ---
void reset_orb() {
    // status = IDLE; // Status is now derived
    GAME_ID = "";
    PLAYER_NAMES[0] = "";
    PLAYER_NAMES[1] = "";

    board.resetBoard(); // Reset logical board
    ORB_CODE = generate_orb_code(); // Generate new code on reset

    // TODO: Reset motion controller state if needed? It should return to IDLE naturally.
    // motionController.resetState(); // Might need a reset function

    send_orb_status_update(); 
    nextion.writeStr("page home_screen"); 
    nextion.lastCurrentPageId = 255; 
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
            lastCommandProcessed.fromCoords = {xfrom, yfrom};
            lastCommandProcessed.toCoords = {xto, yto};
            lastCommandProcessed.wasCapture = (board.grid[xto][yto] != nullptr); // Check board *before* move starts
            lastCommandProcessed.isValid = true; // Mark data as valid for post-move update
            // TODO: Parse and store special move data here later

            // --- Convert to Algebraic for Motion Controller ---
            String fromAlg = board.getSquareString(lastCommandProcessed.fromCoords);
            String toAlg = board.getSquareString(lastCommandProcessed.toCoords);
            Serial.print("  Parsed Move: "); Serial.print(fromAlg); Serial.print(" -> "); Serial.println(toAlg);
            if (lastCommandProcessed.wasCapture) { Serial.println("  (Is a capture)"); }

            // --- Initiate Physical Move ---
            // MotionController's start sequence will internally re-check for capture if needed
            bool sequenceStarted = motionController.startMoveSequence(fromAlg, toAlg);
            if (sequenceStarted) {
                Serial.println("  Physical move sequence initiated.");
                send_orb_status_update(); // Send OCCUPIED status
            } else {
                Serial.println("  Failed to initiate physical move sequence.");
                // Invalidate stored command data if sequence didn't start
                lastCommandProcessed.isValid = false;
            }
        } break;

        default:
            Serial.print("Unhandled message type: "); Serial.println(type);
            break;
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

    nextion.lastCurrentPageId = -1;
    nextion.writeStr("page home_screen"); // Go to home screen on boot
    load_nextion_page();
}

void loop() {
    timer = millis();
    nextion.NextionListen(); // Check for Nextion events

    // Non-blocking update for the motion controller state machine
    motionController.update(); // <<<=== Update Motion Controller

    
    // --- State Transition Detection & Actions ---
    static MotionState lastMotionState = MOTION_IDLE;
    MotionState currentMotionState = motionController.getCurrentState();

    // Check if the state just transitioned *to* IDLE from a non-IDLE state
    if (lastMotionState != MOTION_IDLE && currentMotionState == MOTION_IDLE) {
        Serial.print("MotionController transitioned from state ");
        Serial.print(lastMotionState); // State it came FROM
        Serial.println(" to MOTION_IDLE.");

        // Always send IDLE status update when becoming idle
        send_orb_status_update();

        // --- Post-Sequence Updates ---
        // Check if the sequence that just finished was a move AND we have valid stored data
        if (lastMotionState == DO_COMPLETE && lastCommandProcessed.isValid) {
            Serial.println("Executing Post-Move Updates (Board Logic, Nextion)...");

            // Use the globally stored coordinates from the completed command
            std::pair<int, int> from = lastCommandProcessed.fromCoords;
            std::pair<int, int> to = lastCommandProcessed.toCoords;
            String fromStr = board.getSquareString(from); // For logging
            String toStr = board.getSquareString(to);     // For logging

            Serial.print("  Updating logical board for: "); Serial.print(fromStr); Serial.print(" -> "); Serial.println(toStr);
            // Update Logical Board FIRST - board.movePiece handles internal capture logic
            bool moveSuccess = board.movePiece(from, to);
            if (!moveSuccess) {
                Serial.println("!!! WARNING: Logical board move failed! Board state might be inconsistent.");
            }
            board.printBoard(); // Print updated logical board state

            Serial.print("  Updating Nextion display for: "); Serial.print(fromStr); Serial.print(" -> "); Serial.println(toStr);
            // Update Nextion Display SECOND
            move_nextion_piece(from, to);

            // Invalidate the stored command data now that it has been processed
            lastCommandProcessed.isValid = false;
            Serial.println("  Post-move updates complete.");

        } else if (lastMotionState == HOMING_COMPLETE) {
            Serial.println("Homing sequence completed. No board updates needed.");
            // Ensure command data is invalid after homing too
            lastCommandProcessed.isValid = false;
        } else if (lastCommandProcessed.isValid){
            // Became IDLE from a state other than DO_COMPLETE or HOMING_COMPLETE (e.g., ERROR)
            // Invalidate data to prevent accidental updates later
            Serial.println("Became IDLE unexpectedly. Invalidating last command data.");
            lastCommandProcessed.isValid = false;
        }
    }
    // Update the tracked state for the next iteration
    lastMotionState = currentMotionState;


    // Handle WebSocket polling less frequently
    if (timer - network_timer > 200) { // Check more often than 500ms?
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
            if(nextion.currentPageId == HOME_SCREEN) { // Only update if on home screen?
                 nextion.writeStr("txt_connected.txt", "no server");
                 nextion.writeNum("txt_connected.pco", 64512); // Orange/Yellow
            }
        }
    } 


    // Refresh Nextion page contents if needed (call frequently but it has internal checks)
    load_nextion_page();
}


// --- Nextion Trigger Functions ---
// TODO: Implement these later using MotionController manual control methods

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