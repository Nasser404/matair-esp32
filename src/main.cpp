#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <EasyNextionLibrary.h> 
#include <AccelStepper.h>

#include <board.h>
#include <enums.h>
#include <config.h>


/////////////////////////////////// NEXTION //////////////
EasyNex nextion(Serial2); // RX 16,  TX 17
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

String ORB_CODE   = "";
String GAME_ID    = "";
String PLAYER_NAMES[2] = {"", ""};

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
    ORB_CODE = generate_orb_code();

    
    ///
    send_orb_data();
    nextion.writeStr("page home_screen"); // For synchronizing Nextion page in case of reset to Arduino
    nextion.lastCurrentPageId = 1; // At the first run of the loop, the currentPageId and the lastCurrentPageId must have different values, due to run the function firstRefresh()
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
                    nextion.writeStr("txt_connected.txt", "not connected");
                    nextion.writeNum("txt_connected.pco", 63488);  
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
        nextion.writeNum("num_port.val",        websockets_server_port);
        break;
    }

    newPageLoaded = false;  // After we have updated the new page for the first time, we update the variable to false.
            // Now the values updated ONLY if the new value is different from the last Sent value.
            // See void refreshPage0()

    nextion.lastCurrentPageId = nextion.currentPageId; // Afer the refresh of the new page We make them equal,
                                    // in order to identify the next page change.
    }
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
            

            move_nextion_piece({xfrom, yfrom}, {xto, yto});
            
            board.movePiece({xfrom, yfrom}, {xto, yto});
            board.printBoard();

            

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
    
    ORB_CODE = generate_orb_code();
    Serial.begin(115200);
    nextion.begin(9600);



    
    
    Serial.println(ORB_CODE);
    board.printBoard();
    

    CONNECTED_TO_WIFI = connect_to_wifi();
    if (CONNECTED_TO_WIFI) {
        CONNECTED_TO_SERVER = connect_to_server();
        timeout_timer = millis();
    }

    nextion.writeStr("page home_screen"); // For synchronizing Nextion page in case of reset to Arduino
    nextion.lastCurrentPageId = 1; // At the first run of the loop, the currentPageId and the lastCurrentPageId must have different values, due to run the function firstRefresh()

}

void loop() {
    timer = millis();
    nextion.NextionListen();

    load_nextion_page();
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



void trigger0(){
    ESP.restart();
}