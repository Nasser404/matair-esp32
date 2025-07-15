#pragma once 


enum MESSAGE_TYPE {
    IDENTIFICATION          = 0,           
    
    PING                    = 1,
    PONG                    = 2,
    
    GAME_DATA               = 3,
    GAME_INFO               = 4,
    GAME_DISCONNECT         = 5,
    
    ORB_DATA                = 6,
    ORB_RESET               = 7,
    
    ORB_CONNECT             = 8,
    PLAYER_CONNECT          = 9,
    VIEWER_CONNECT          = 10,

    ORB_LIST                = 11,
    GAME_LIST               = 12,
    
    DISCONNECT_REASON       = 13,
    
    ASK_MOVE                = 14,
    MOVE                    = 15,
    
    ORB_NEW_GAME            = 16,
    ORB_CONTINUE_GAME       = 17,
    ORB_END_GAME            = 18,
    
    DISCONNECT_FROM_SERVER  = 19, 
    INFORMATION             = 20,
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
    PROMOTION   = 0,
    EN_PASSANT  = 1, // DISABELED
    CASTLE      = 2, // DISABELED
    NONE        = 3, // DISABELED

};

enum ManualActuator {
    STEPPER_CART,
    STEPPER_ORB,
    STEPPER_CAPTURE,
    GRIPPER_ROTATION,
    GRIPPER_OPEN_CLOSE,
    LINEAR_ACTUATOR
};
enum NEXTION_PAGE { // GET THE CORRESPONDING ID IN THE NEXTION EDITOR
    HOME_SCREEN             = 0,
    BOARD_SCREEN            = 1,
    CONTROL_SCREEN          = 2,
    SETTING_SCREEN          = 3,
    CONTROL_CART_SCREEN     = 4,
    CONTROL_ORB_SCREEN      = 5,
    CONTROL_CAPTURE_SCREEN  = 6,

    CONNECTION_LOST_SCREEN   = 7,
    ERROR_STATE_SCREEN       = 8,
    RESET_IN_PROGRESS_SCREEN = 9,
    BEFORE_REBOOT_SCREEN    = 10,
    ASK_PIECES_SCREEN        = 11,    
    
    NUMBER_OF_PAGE

};

extern const int pagePriorities[];
extern const char* pageCommands[];

