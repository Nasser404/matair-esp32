#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "EasyNextionLibrary.h" 
#include <AccelStepper.h>
const char* ssid                        = "AAA";            //WIFI SSID 
const char* password                    = "grelo1234";      //WIFI Password

const char* websockets_server_host      = "88.187.38.210";  // Server adress
const uint16_t websockets_server_port   = 29920;            // Server port

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

EasyNex nextion(Serial2); // RX 16,  TX 17

////////////////////////////// TIMERS ///////////////////////////////
unsigned long network_timer = millis();
unsigned long timeout_timer = millis();
unsigned long timer = millis();
/////////////////////////////////////////////////////////////////////

using namespace websockets;
WebsocketsClient client;

String ORB_ID    = "ORB IVRY";
String ORB_CODE  = "";

int GAME_ID      = -1;
String GAME_NAME = "";

bool CONNECTED_TO_WIFI      = false;
bool CONNECTED_TO_SERVER    = false;

unsigned short status       = OCCUPIED;

enum PIECE_TYPE {
    pawn    = 0,
    knight  = 1,
    rook    = 2,
    bishop  = 3,
    king    = 4,
    queen   = 5,
    empty,

    
};

enum PIECE_COLOR {
    white,
    black,

};
struct piece {
    unsigned short color;
    unsigned short type;
    char string;
};
piece BOARD[8][8] = {{{black, rook, 'R'}, {black, knight, 'N'}, {black, bishop, 'B'}, {black, queen, 'Q'}, {black, king, 'K'}, {black, bishop, 'B'},{black, knight, 'N'},{black,rook, 'R'}  }, 
                     {{black, pawn, 'P'}, {black, pawn, 'P'},   {black, pawn, 'P'},   {black, pawn, 'P'},  {black, pawn, 'P'}, {black, pawn, 'P'},  {black, pawn, 'P'},  {black, pawn, 'P'} }, 
                     {{empty, empty, '*'},{empty, empty, '*'},  {empty, empty, '*'},  {empty, empty, '*'}, {empty, empty, '*'},{empty, empty, '*'}, {empty, empty, '*'}, {empty, empty, '*'}},
                     {{empty, empty, '*'},{empty, empty, '*'},  {empty, empty, '*'},  {empty, empty, '*'}, {empty, empty, '*'},{empty, empty, '*'}, {empty, empty, '*'}, {empty, empty, '*'}},
                     {{empty, empty, '*'},{empty, empty, '*'},  {empty, empty, '*'},  {empty, empty, '*'}, {empty, empty, '*'},{empty, empty, '*'}, {empty, empty, '*'}, {empty, empty, '*'}},
                     {{empty, empty, '*'},{empty, empty, '*'},  {empty, empty, '*'},  {empty, empty, '*'}, {empty, empty, '*'},{empty, empty, '*'}, {empty, empty, '*'}, {empty, empty, '*'}},
                     {{white, pawn, 'p'}, {white, pawn, 'p'},   {white, pawn, 'p'},   {white, pawn, 'p'},  {white, pawn, 'p'}, {white, pawn, 'p'},  {white, pawn, 'p'},  {white, pawn, 'p'} },
                     {{white, rook, 'r'}, {white, knight, 'n'}, {white, bishop, 'b'}, {white, queen, 'q'}, {white, king, 'k'}, {white, bishop, 'b'},{white, knight, 'n'},{white,rook, 'r'}  }
                    };




String get_board_string() {
    String board_string = "";
    for (int i =0;i<8;i+=1) {
        for (int j =0;j<8;j+=1) {
            board_string +=BOARD[i][j].string;
        }
    }
    return board_string;

}

String generate_orb_code() {
    String code = "";

    static const char alphanum[] =
    "0123456789"
    "ABCDEFHIJKLMNOPQRSTUVWXYZ";

    for (int i = 0; i < 4; ++i) {
        code += alphanum[random(sizeof(alphanum) - 1)];
    }
    
    return code;
    
}
void send_orb_data() {
    JsonDocument orb_data;

    orb_data["type"]    =  ORB_DATA;
    orb_data["orb_code"]= ORB_CODE ;
    orb_data["status"]  = status;
    String json_data;
    serializeJson(orb_data, json_data);

    client.send(json_data); 

}

void reset_orb() {
    status = IDLE;

    ///


    ///
    send_orb_data();

}


void handle_data(WebsocketsMessage packet) {
    String json_data = packet.data();

    //Serial.print("Got Message: ");
    //Serial.println(json_data);
    
    
    JsonDocument data;
    DeserializationError error = deserializeJson(data, json_data);

    // Test if parsing succeeds
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    int type = data["type"];
    

    switch (type)
    {
        case PING :
        //Serial.println("PING RECEIVED -> SENDING PONG");
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
            connect_data["orb_board"]  = get_board_string();
            String json_data;
            serializeJson(connect_data, json_data);

            client.send(json_data); 
        }
        break;

        case GAME_INFO :
        Serial.println("GAME INFO");
        GAME_ID     = data["info"]["GAME ID"];
        //GAME_NAME   = data["info"]["NAME"];
        break;


        case ORB_RESET :
        Serial.println("RESET INSTRUCTION");
        status = OCCUPIED;
        reset_orb();
        break;
        
        case MOVE :
        {
            status = OCCUPIED;
            Serial.println("MOVE INSTRUCTION");
            Serial.println(json_data);

            unsigned short xfrom, yfrom, xto, yto;
            xfrom = data["from"][0];
            yfrom = data["from"][1];
            xto   = data["to"][0];
            yto   = data["to"][1];

            piece piece_to_move = BOARD[xfrom][yfrom];
            piece square_to_move_to = BOARD[xto][yto];

            
            BOARD[xto][yto] = piece_to_move;
            piece empty_square {empty, empty, '*'};
            BOARD[xfrom][yfrom] = empty_square;
        

            String result = get_board_string();
            Serial.println(result);


            status = IDLE;
            send_orb_data();
        }
        break;

        default: break;
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
    bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
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
    
    ORB_CODE = "1100";
    Serial.begin(115200);
    nextion.begin(9600);

    
    Serial.println(ORB_CODE);
    Serial.println(get_board_string());
    nextion.writeStr("text_orb_code.txt", ORB_CODE);
    nextion.writeStr("t0.txt", "conecting...");
    nextion.writeNum("t0.pco", 65535);

    CONNECTED_TO_WIFI = connect_to_wifi();
    if (CONNECTED_TO_WIFI) {
        nextion.writeStr("t0.txt", "connected");
        nextion.writeNum("t0.pco", 2016);
        CONNECTED_TO_SERVER = connect_to_server();
        timeout_timer = millis();
    
    } else {
        nextion.writeStr("t0.txt", "no wifi");
        nextion.writeNum("t0.pco", 63488);
    }
    

}

void loop() {
    timer = millis();
    nextion.NextionListen();
    // let the websockets client check for incoming messages
    if (timer - network_timer > 500) {
        if(client.available()) {
            client.poll();
        }
        network_timer = timer;
    }

    if (CONNECTED_TO_SERVER) {
        if (timer - timeout_timer > 10000) { // TIMEOUT EVENT
            Serial.println("TIMEOUT EVENT");
            CONNECTED_TO_SERVER = false;
            client.close();
        }
    }
}

//HELLO