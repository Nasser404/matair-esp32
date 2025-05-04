#pragma once // <<<=== ADD THIS INCLUDE GUARD
enum MESSAGE_TYPE {
    IDENTIFICATION  = 0 ,           
    
    PING            = 1,
    PONG            = 2,
    
    GAME_DATA       = 3,
    GAME_INFO       = 4,
    GAME_DISCONNECT = 5,
    
    ORB_DATA        = 6,
    ORB_RESET       = 7,
    
    ORB_CONNECT     = 8,
    PLAYER_CONNECT  = 9,
    VIEWER_CONNECT  = 10,

    ORB_LIST        = 11,
    GAME_LIST       = 12,
    
    DISCONNECT_REASON=13,
    
    ASK_MOVE        = 14,
    MOVE            = 15,
    
    ORB_NEW_GAME    = 16,
    ORB_CONTINUE_GAME=17,
    ORB_END_GAME    = 18,
    
    DISCONNECT_FROM_SERVER = 19 , 
    INFORMATION            = 20 ,
};

enum CLIENT_TYPE {
    ORB,
    PLAYER,
    VIEWER,
};

enum ORB_STATUS {
    IDLE,
    OCCUPIED,
};

enum SPECIAL_MOVES {
    EN_PASSANT,
    PROMOTION,
    CASTLE,

};


enum NEXTION_PAGE { // GET THE CORRESPONDING ID IN THE NEXTION EDITOR
    HOME_SCREEN     = 0,
    BOARD_SCREEN    = 1,
    CONTROL_SCREEN   = 2,
    SETTING_SCREEN  = 3,

};