#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <EasyNextionLibrary.h> 
#include <AccelStepper.h>

#include <board.h>
#include <enums.h>
#include <config.h>

EasyNex nextion(Serial2); // RX 16,  TX 17

////////////////////////////// TIMERS ///////////////////////////////
unsigned long network_timer = millis();
unsigned long timeout_timer = millis();
unsigned long timer = millis();
/////////////////////////////////////////////////////////////////////

using namespace websockets;
WebsocketsClient client;

String ORB_CODE   = "";
String GAME_ID    = "";

bool CONNECTED_TO_WIFI      = false;
bool CONNECTED_TO_SERVER    = false;
unsigned short status       = OCCUPIED;
Board board;

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
    GAME_ID = "";
    
    ///
    board.resetBoard();
    nextion.writeStr("page 0");
    ORB_CODE= generate_orb_code();
    nextion.writeStr("txt_orb_code.txt", ORB_CODE);
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
            connect_data["orb_board"]  = board.getBoardString();
            String json_data;
            serializeJson(connect_data, json_data);

            client.send(json_data); 
        }
        break;

        case GAME_INFO :
        {
            Serial.println("GAME INFO");
            GAME_ID = (const char*)data["info"]["game_id"];
            nextion.writeStr("txt_game_id.txt", GAME_ID);
            bool local_game = data["info"]["local_game"];
            
            String p1_name, p2_name;
            if (local_game) {
                p1_name = (const char*)data["info"]["white_player"];
                p2_name = (const char*)data["info"]["black_player"];
            } else {
                p1_name = (const char*)data["info"]["white_orb"];
                p2_name = (const char*)data["info"]["black_orb"];
            }

            nextion.writeStr("txt_p1_name.txt", p1_name);
            nextion.writeStr("txt_p2_name.txt", p2_name);
        }
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
            
            int id =  board.getSquareNextionId({xfrom, yfrom});
            board.movePiece({xfrom, yfrom}, {xto, yto});
            board.printBoard();
            
            
            String from_square = board.getSquareString({xfrom, yfrom})+ ".picc";

            String to_square = board.getSquareString({xto, yto}) + ".picc";
            

            Serial.println(from_square);
            Serial.println(to_square);
            nextion.writeNum(to_square, id);
            
            nextion.writeNum(from_square, 1);
            /*
            char row[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
            for (int j = 0; j < 8; ++j) {
                for (int i = 0; i < 8; ++i) {

                    String square = String(row[i] + String(8-j) + ".pic");
                    Serial.println(square);
                    if (board.grid[i][j]) {
            
                        nextion.writeNum(square, board.grid[i][j]->getNextionId());
                    } else {
                        nextion.writeNum(square,17);
                    }
                }
            }*/

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
    board.printBoard();
    
    nextion.writeStr("txt_orb_code.txt", ORB_CODE);
    nextion.writeStr("txt_connected.txt", "conecting...");
    nextion.writeNum("txt_connected.pco", 65535);

    CONNECTED_TO_WIFI = connect_to_wifi();
    if (CONNECTED_TO_WIFI) {
        nextion.writeStr("txt_connected.txt", "connected");
        nextion.writeNum("txt_connected.pco", 2016);
        CONNECTED_TO_SERVER = connect_to_server();
        timeout_timer = millis();
    
    } else {
        nextion.writeStr("txt_connected.txt", "no wifi");
        nextion.writeNum("txt_connected.pco", 63488);
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
        if (timer - timeout_timer > 20000) { // TIMEOUT EVENT
            Serial.println("TIMEOUT EVENT");
            CONNECTED_TO_SERVER = false;
            client.close();
        }
    }
}
