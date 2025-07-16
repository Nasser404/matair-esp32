#pragma once

#include <EasyNextionLibrary.h>
#include "MotionController.h" 
#include "board.h"            
#include "enums.h"           

class NextionHandler {
public:
    // Constructor takes references to the main components it needs to interact with
    NextionHandler(MotionController& mc, Board& b);

    void setup();  // Initialize the Nextion serial connection
    void update(); // Main update loop, call from main.cpp loop()

    // High-level UI update methods
    void updateGameInfo(const String& gameId, const String& p1Name, const String& p2Name);
    void updateConnectionStatus();
    void movePieceOnDisplay(std::pair<int, int> from, std::pair<int, int> to);
    void changePage(int newPageId, bool forceChange = false);
    void forceFullRefresh();

    // Trigger handlers
    void handleTrigger(int triggerId);
    String getString(String objectName); // Wrapper for nextion readStr()
    uint32_t getNumber(String objectName);  // Wrapper for nextion readNumber()
    
    void setString(String objectName, String value);
    void setNumber(String objectName, uint32_t value);
    void updateResetStatus(int progress, String status);


private:
    EasyNex nextion; 
    
    // References to other system components
    MotionController& motionController;
    Board& board;

    // Internal state
    bool newPageLoaded = false;
    bool forceFullRefreshFlag = false;
    unsigned long pageRefreshTimer = 0;

    // Methods to update content on a specific page when it's first loaded
    void loadPageContent();
    void refreshHomePage();
    void refreshBoardPage();
    void refreshControlPage();
    void refreshSettingPage();
};