#include <Arduino.h>
#include <WiFi.h>

#include "Motor.hpp"

const char* WIFI_SSID = "WIFISSID";
const char* WIFI_PASS = "WIFIPASS";

String hostname = "tumbller";

Motor Motor;

/*You can set the speed: 0~255 */
#define SPEED 60

char strbuf[][50]={
  "FORWARD",
  "BACK",
  "LEFT",
  "RIGHT",
  "STOP"
};

WiFiServer server(80);

String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

String motorState;

void setup() {
  Motor.Pin_init();
  Motor.Stop(0);
  // Motor.Encoder_init();
  Serial.begin(115200);

  //Setup wifi
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str()); //define hostname
  Serial.println("Connecting to wifi");
  // Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS );
  int count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
    count++;
    if(count > 80)
    {
      Serial.print("\n");
    }
  }
    // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  // Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("hello tumbller!'");
  // delay(500);
  // for(int i = 0; i < 4; i++)
  // {
  //   Serial.println(strbuf[i]);
  //   (Motor.*(Motor.MOVE[i]))(SPEED);
  //   delay(1500);
  //   Serial.println(Motor.encoder_count_right_a);
  //   Serial.println(Motor.encoder_count_left_a);
  //   Motor.encoder_count_right_a=0;
  //   Motor.encoder_count_left_a=0;

  // }

  WiFiClient client = server.available();   // Listen for incoming clients

  if(client)
  {
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");
    String currentLine = "";                // make a String to hold incoming data from the client
    while(client.connected() && currentTime-previousTime <= timeoutTime )
    {
      currentTime = millis();
      if(client.available())
      {
        char c = client.read();
        Serial.write(c);
        header += c;
        if(c == '\n')
        { // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response
          if (currentLine.length() == 0)
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            //move the motors
            if (header.indexOf("GET /motor/forward") >= 0) {
              Serial.println(strbuf[0]);
              motorState = strbuf[0];
              (Motor.*(Motor.MOVE[0]))(SPEED);
            } else if (header.indexOf("GET /motor/back") >= 0) {
              Serial.println(strbuf[1]);
              motorState = strbuf[1];
              (Motor.*(Motor.MOVE[1]))(SPEED);
            } else if (header.indexOf("GET /motor/left") >= 0) {
              Serial.println(strbuf[2]);
              motorState = strbuf[2];
              (Motor.*(Motor.MOVE[2]))(SPEED);
            } else if (header.indexOf("GET /motor/right") >= 0) {
              Serial.println(strbuf[3]);
              motorState = strbuf[3];
              (Motor.*(Motor.MOVE[3]))(SPEED);
            } else if (header.indexOf("GET /motor/stop") >= 0) {
              Serial.println(strbuf[4]);
              motorState = strbuf[4];
              (Motor.*(Motor.MOVE[4]))(SPEED);
            }
            

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } 
          else 
          {      // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if ( c!= '\r')
        { // if you got anything else but a carriage return 
          currentLine += c;    // add it to the endof the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
