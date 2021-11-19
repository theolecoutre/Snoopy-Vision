//
// A simple server implementation showing how to:
//  * serve static messages
//  * read GET and POST parameters
//  * handle missing pages / 404s
//

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>

#define ANGLE_ON 90
#define ANGLE_OFF 0

AsyncWebServer server(80);
Servo servo;

const char * SSID = "spot-BD-02300009";
const char * PASSWORD = "fma5iuy5tws4";

const char* PARAM_MESSAGE = "message";

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void setup() {

    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    servo.attach(D0, 771, 2740);
    servo.write(0);

    digitalWrite(LED_BUILTIN, HIGH);


    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });

    server.on("/paint-on", HTTP_GET, [](AsyncWebServerRequest *request){
        servo.write(ANGLE_ON);
        digitalWrite(LED_BUILTIN, LOW);
        request->send(200, "text/plain", "Painting on.");
        Serial.println("Painting on.");
    });

    server.on("/paint-off", HTTP_GET, [](AsyncWebServerRequest *request){
        servo.write(ANGLE_OFF);
        digitalWrite(LED_BUILTIN, HIGH);
        request->send(200, "text/plain", "Painting off.");
        Serial.println("Painting off.");
    });

    // Send a GET request to <IP>/get?message=<message>

    server.onNotFound(notFound);

    server.begin();
}

void loop() {
}