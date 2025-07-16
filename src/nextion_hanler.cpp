
#include "nextion_handler.h"
#include <Arduino.h> 
// --- Externally defined variables and functions from main.cpp ---
extern char ssid[];
extern char password[];
extern char websockets_server_host[];
extern uint16_t websockets_server_port;
extern String ORB_CODE;
extern bool IN_GAME;
extern bool CONNECTED_TO_WIFI;
extern bool CONNECTED_TO_SERVER;
extern void writeCredentials();
extern bool RESET_ASKED;
extern bool REBOOT_ASKED;
extern bool proper_bootup;
const int pagePriorities[NUMBER_OF_PAGE] = {
  1, // HOME_SCREEN
  1, // BOARD_SCREEN
  1, // CONTROL_SCREEN
  1, // SETTING_SCREEN
  1, // CONTROL_CART_SCREEN
  1, // CONTROL_ORB_SCREEN
  1, // CONTROL_CAPTURE_SCREEN
  3, // CONNECTION_LOST_SCREEN  
  4, // ERROR_STATE_SCREEN      
  3, // RESET_IN_PROGRESS_SCREEN
  3, // BEFORE_REBOOT_SCREEN    
  2, // ASK_PIECES_SCREEN       
};

const char* pageCommands[NUMBER_OF_PAGE] = {
  "page home_screen",
  "page board_screen",
  "page control_screen",
  "page setting_screen",
  "page control_cart",
  "page control_orb",
  "page control_capt",
  "page connect_lost",
  "page error_screen",
  "page before_reboot",
  "page ask_piece"
};


NextionHandler::NextionHandler(MotionController& mc, Board& b) :
    nextion(Serial2), 
    
    motionController(mc),
    board(b) {
}

void NextionHandler::setup() {
    nextion.begin(9600);
    nextion.lastCurrentPageId = -1; // Force initial page load

    
    if (proper_bootup) {
        changePage(HOME_SCREEN, true);
        proper_bootup = false;
    }
    else changePage(ASK_PIECES_SCREEN, true);
}

void NextionHandler::update() {
    nextion.NextionListen(); 
    loadPageContent();       // Check if page has changed and update content

    // Handle periodic updates for specific pages
    if ((millis() - pageRefreshTimer) > 1000) {
        if (nextion.currentPageId == CONTROL_SCREEN) {
            refreshControlPage(); 
        }
        if (nextion.currentPageId==HOME_SCREEN) {
            refreshHomePage();
        }
        pageRefreshTimer = millis();
    }
}

String NextionHandler::getString(String objectName) {
    return nextion.readStr(objectName);
    
}

uint32_t NextionHandler::getNumber(String objectName) {
    return nextion.readNumber(objectName);
}

void NextionHandler::setString(String objectName, String value) {
    nextion.writeStr(objectName, value);
}
void NextionHandler::setNumber(String objectName, uint32_t value) {
    nextion.writeNum(objectName, value);
}
void NextionHandler::updateResetStatus(int progress, String status) {
    if (nextion.currentPageId == RESET_IN_PROGRESS_SCREEN) {
        nextion.writeNum("b_reset_bar.val", progress);
        nextion.writeStr("b_reset_text.txt", status);
    }
}

void NextionHandler::changePage(int newPageId, bool forceChange) {
    if (newPageId < 0 || newPageId >= NUMBER_OF_PAGE) return;
    
    // Logic to prevent moving from a high-priority screen (like an error) to a low one
    if (forceChange || (pagePriorities[newPageId] >= pagePriorities[nextion.currentPageId])) {
        Serial.print("Nextion: Changing page to "); Serial.println(pageCommands[newPageId]);
        nextion.writeStr(pageCommands[newPageId]);
    } else {
        Serial.print("Nextion: Page change to "); Serial.print(pageCommands[newPageId]);
        Serial.println(" blocked due to priority.");
    }
}

void NextionHandler::forceFullRefresh() {
    forceFullRefreshFlag = true;
    // The loadPageContent method will catch this flag
}

void NextionHandler::loadPageContent() {
    if (forceFullRefreshFlag) {
        nextion.lastCurrentPageId = -1; // Force all components to update
        forceFullRefreshFlag = false;
        Serial.println("Nextion: Forcing full page refresh.");
    }

    if (nextion.currentPageId == nextion.lastCurrentPageId) {
        return; // Page hasn't changed, do nothing
    }

    // A new page has loaded
    switch (nextion.currentPageId) {
        case HOME_SCREEN:
            refreshHomePage();
            break;
        case BOARD_SCREEN:
            refreshBoardPage();
            break;
        case CONTROL_SCREEN:
            refreshControlPage();
            break;
        case SETTING_SCREEN:
            refreshSettingPage();
            break;
   
    }
    nextion.lastCurrentPageId = nextion.currentPageId; 
}

void NextionHandler::refreshHomePage() {
    Serial.println("Nextion: Refreshing Home Screen.");
    nextion.writeStr("txt_orb_code.txt", ORB_CODE);
    updateConnectionStatus();
}

void NextionHandler::refreshBoardPage() {
    Serial.println("Nextion: Refreshing Board Screen (Full Redraw).");
    for (int j = 0; j < 8; ++j) {
        for (int i = 0; i < 8; ++i) {
            String square = board.getSquareString({i, j}) + ".picc";
            int id = board.getSquareNextionId({i, j});
            nextion.writeNum(square, id);
        }
    }
    
}

void NextionHandler::refreshControlPage() {
    int orb_pos = motionController.stepper3.currentPosition();
    int cart_pos = motionController.stepper2.currentPosition();
    int capt_pos = motionController.stepper1.currentPosition();
    nextion.writeNum("num_orb_pos.val", orb_pos);
    nextion.writeNum("num_cart_pos.val", cart_pos);
    nextion.writeNum("num_capt_pos.val", capt_pos);

    bool is_idle = (motionController.getCurrentState() == MOTION_IDLE);
    nextion.writeStr("txt_status.txt", is_idle ? "IDLE" : "BUSY");
    nextion.writeNum("txt_status.pco", is_idle ? 2016 : 63488); // Green for IDLE, Red for BUSY
}

void NextionHandler::refreshSettingPage() {
    Serial.println("Nextion: Refreshing Settings Screen.");
    nextion.writeStr("txt_wifi_ssid.txt", ssid);
    nextion.writeStr("txt_wifi_pwd.txt", password);
    nextion.writeStr("txt_ip.txt", websockets_server_host);
    nextion.writeNum("num_port.val", websockets_server_port);
}

void NextionHandler::updateConnectionStatus() {
    if (nextion.currentPageId == HOME_SCREEN) {
        if (CONNECTED_TO_WIFI) {
            if (CONNECTED_TO_SERVER) {
                nextion.writeStr("txt_connected.txt", "connected");
                nextion.writeNum("txt_connected.pco", 2016); // Green
            } else {
                nextion.writeStr("txt_connected.txt", "no server");
                nextion.writeNum("txt_connected.pco", 64512); // Orange/Yellow
            }
        } else {
            nextion.writeStr("txt_connected.txt", "no wifi");
            nextion.writeNum("txt_connected.pco", 63488); // Red
        }
    }
}

void NextionHandler::updateGameInfo(const String& gameId, const String& p1Name, const String& p2Name) {
    if (nextion.currentPageId == BOARD_SCREEN) {
        nextion.writeStr("txt_game_id.txt", gameId);
        nextion.writeStr("txt_p1_name.txt", p1Name);
        nextion.writeStr("txt_p2_name.txt", p2Name);
    }
}

void NextionHandler::movePieceOnDisplay(std::pair<int, int> from, std::pair<int, int> to) {
    if (nextion.currentPageId != BOARD_SCREEN) {
        changePage(BOARD_SCREEN);
    }
    String from_square = board.getSquareString({from.first, from.second}) + ".picc";
    String to_square = board.getSquareString({to.first, to.second}) + ".picc";
    int id = board.getSquareNextionId({from.first, from.second}); // ID of the piece at the 'from' square BEFORE the logical move
    nextion.writeNum(to_square, id);
    nextion.writeNum(from_square, 1); // 1 is empty square image ID
}


// --- Main Trigger Handler ---
void NextionHandler::handleTrigger(int triggerId) {
    switch (triggerId) {
        // Reboot/Reset Triggers
        case 0: 

            if (nextion.currentPageId == SETTING_SCREEN) {
                if ((!board.isAtStartingPosition()) && IN_GAME) { // Check if board is modified and in game
                    changePage(BEFORE_REBOOT_SCREEN);
                } else {
                    // Directly reboot if not in-game or board is clean
                    nextion.writeStr("txt_reboot.txt", "Saving & Rebooting...");
                    REBOOT_ASKED = true; 
                }
            } else if (nextion.currentPageId == BEFORE_REBOOT_SCREEN || nextion.currentPageId == ERROR_STATE_SCREEN) {
                REBOOT_ASKED = true; 
            }
            break;
        case 40: // Clicked Reset on ASK_REBOOT screen
           RESET_ASKED = true; 
        break;
            
        // Homing
        case 1:
            Serial.println("Nextion: Home Button Pressed.");
            if (!motionController.isBusy()) {
                motionController.startHomingSequence();
                changePage(RESET_IN_PROGRESS_SCREEN);
            } else {
                Serial.println("Cannot Home: Motion Controller is busy.");
            }
            break;

        // CART Stepper Jog
        case 2: motionController.startManualJog(ManualActuator::STEPPER_CART, true); break;
        case 3: motionController.stopManualJog(ManualActuator::STEPPER_CART); break;
        case 4: motionController.startManualJog(ManualActuator::STEPPER_CART, false); break;
        case 5: motionController.stopManualJog(ManualActuator::STEPPER_CART); break;

        // ORB Stepper Jog
        case 6: motionController.startManualJog(ManualActuator::STEPPER_ORB, true); break;
        case 7: motionController.stopManualJog(ManualActuator::STEPPER_ORB); break;
        case 8: motionController.startManualJog(ManualActuator::STEPPER_ORB, false); break;
        case 9: motionController.stopManualJog(ManualActuator::STEPPER_ORB); break;
        
        // CAPTURE Stepper Jog
        case 10: motionController.startManualJog(ManualActuator::STEPPER_CAPTURE, true); break;
        case 11: motionController.stopManualJog(ManualActuator::STEPPER_CAPTURE); break;
        case 12: motionController.startManualJog(ManualActuator::STEPPER_CAPTURE, false); break;
        case 13: motionController.stopManualJog(ManualActuator::STEPPER_CAPTURE); break;
        
        // GRIPPER ROTATION Servo Jog
        case 14: motionController.startManualJog(ManualActuator::GRIPPER_ROTATION, true); break;
        case 15: /* Release does nothing for stepped jog */ break;
        case 16: motionController.startManualJog(ManualActuator::GRIPPER_ROTATION, false); break;
        case 17: /* Release does nothing for stepped jog */ break;
        
        // LINEAR ACTUATOR Jog
        case 18: motionController.startManualJog(ManualActuator::LINEAR_ACTUATOR, true); break;
        case 19: motionController.stopManualJog(ManualActuator::LINEAR_ACTUATOR); break;
        case 20: motionController.startManualJog(ManualActuator::LINEAR_ACTUATOR, false); break;
        case 21: motionController.stopManualJog(ManualActuator::LINEAR_ACTUATOR); break;
        
        // GRIPPER OPEN/CLOSE Servo Jog
        case 22: motionController.startManualJog(ManualActuator::GRIPPER_OPEN_CLOSE, true); break;
        case 23: /* Release does nothing for stepped jog */ break;
        case 24: motionController.startManualJog(ManualActuator::GRIPPER_OPEN_CLOSE, false); break;
        case 25: /* Release does nothing for stepped jog */ break;
        
        default:
            Serial.print("Unhandled Nextion Trigger ID: "); Serial.println(triggerId);
            break;
    }
}