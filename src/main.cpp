#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "EasyNextionLibrary.h" 
const char* ssid = "AAA"; //Enter SSID 
const char* password = "grelo1234"; //Enter Password
const char* websockets_server_host = "88.187.38.210"; //Enter server adress
const uint16_t websockets_server_port = 29920; // Enter server port


EasyNex nextion(Serial2); // RX 16,  TX 17

////////////////////////////// TIMERS ///////////////////////////////
unsigned long network_timer = millis();
unsigned long timeout_timer = millis();
unsigned long timer = millis();
/////////////////////////////////////////////////////////////////////


using namespace websockets;
WebsocketsClient client;

enum MESSAGE_TYPE {
    IDENTIFICATION,
    PING,
    GAME_INFO,
    RESET,
    GAME_END,
    ORB_CONNECT,
    PLAYER_CONNECT,
    VIEWER_CONNECT,
    PONG,
    NEW_ORB_CODE, 
};

String ORB_ID    = "ORB IVRY";
String ORB_CODE  = "";
int GAME_ID      = -1;
String GAME_NAME = "";
bool CONNECTED_TO_WIFI      = false;
bool CONNECTED_TO_SERVER    = false;
unsigned short last_ping    = 0;

String BOARD[8][8] = {{"bR","bN" ,"bB", "bQ", "bK", "bB", "bN", "bR"}, 
                   {"bP", "bP", "bP", "bP", "bP", "bP", "bP", "bP"}, 
                   {"*", "*", "*", "*", "*", "*", "*", "*"},
                   {"*", "*", "*", "*", "*", "*", "*", "*"},
                   {"*", "*", "*", "*", "*", "*", "*", "*"},
                   {"*", "*", "*", "*", "*", "*", "*", "*"},
                   {"wP", "wP", "wP", "wP", "wP", "wP", "wP", "wP"},
                   {"wR","wN" ,"wB", "wQ", "wK", "wB", "wN", "wR"}};


String get_board_string() {
    String board_string = "";
    for (int i =0;i<8;i+=1) {
        for (int j =0;j<8;j+=1) {
            board_string +=BOARD[i][j];
        }
    }
    return board_string;

}



    

String generate_orb_code() {
    String code = "";

    static const char alphanum[] =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    for (int i = 0; i < 4; ++i) {
        code += alphanum[random(sizeof(alphanum) - 1)];
    }
    
    return code;
    
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
    client.send("{\"type\":8}"); // SEND PONG
    break;
    

    case IDENTIFICATION : 
    client.send("{\"type\":0,\"identifier\":\"ORB\"}"); // IDENTIFICATION
    Serial.println("INDENTIFICATION REQUEST");
    break;
    
    case ORB_CONNECT : {
    
    JsonDocument connect_data;

    Serial.println("ORB CONNECT");
    connect_data["orb_id"]       = ORB_ID;
    connect_data["orb_code"]     = ORB_CODE ;
    connect_data["orb_board"]    = get_board_string();
    String json_data;
    serializeJson(connect_data, json_data);

    client.send(json_data); // SEND PONG
    }
    break;

    case GAME_INFO :
    Serial.println("GAME INFO");
    GAME_ID     = data["info"]["GAME ID"];
    //GAME_NAME   = data["info"]["NAME"];
    break;


    case RESET :
    Serial.println("RESET INSTRUCTION");
    break;

    default: 
    break;
    }
    //client.send("Ok");
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
    
    ORB_CODE = generate_orb_code();
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