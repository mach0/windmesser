#include <Arduino.h>
#include <EtherCard.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>

LiquidCrystal_I2C
    lcd(0x3F, 16,
        2); // set the LCD address to 0x3F for a 16 chars and 2 line display

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay =
    1000; // the debounce time; increase if the output flickers

int pinInterrupt = 2;
int Count = 0;

// Initialize EtherCard client and MQTT client
static byte mymac[] = {0xAA, 0xBB, 0x69,
                       0x2D, 0x30, 0x31}; // Change this to your own MAC address
byte Ethernet::buffer[700];
static uint32_t timer;
static uint32_t lastPublish = 0;
static uint32_t publishInterval = 5000; // Publish data every 5 seconds
static char mqttBuffer[20];
static char mqttBuffer2[20];
static char mqttServer[] = "your-mqtt-server.com";
static char mqttTopic[] = "your-mqtt-topic";
static uint16_t mqttPort = 1883;
static char mqttUser[] = "your-mqtt-username";
static char mqttPass[] = "your-mqtt-password";

EtherCard etherCard;
PubSubClient mqttClient;

void onChange()
{
  if (digitalRead(pinInterrupt) == LOW)
    Count++;
}

void setup()
{
  Serial.begin(115200);
  pinMode(pinInterrupt, INPUT_PULLUP); // set the interrupt pin

  lcd.init();
  lcd.clear();
  lcd.backlight(); // Make sure backlight is on
  lcd.setCursor(3, 0);
  lcd.print(F("Anemometer"));
  delay(3000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(F("Init Ethernet..."));
  delay(2000);
  lcd.clear();

  // Serial.println(F("[Arduino] Initializing Ethernet..."));

  // if (etherCard.begin(sizeof Ethernet::buffer, mymac, 10) == 0) {
  //   Serial.println(F("[Arduino] Failed to initialize Ethernet"));
  //   while(true);
  // }
  // if (!etherCard.dhcpSetup()) {
  //   Serial.println(F("[Arduino] Failed to get IP address using DHCP"));
  //   while(true);
  // }
  // etherCard.printIp("My IP: ", etherCard.myip);
  // etherCard.printIp("Netmask: ", etherCard.netmask);
  // etherCard.printIp("GW IP: ", etherCard.gwip);
  // etherCard.printIp("DNS IP: ", etherCard.dnsip);
  // etherCard.printIp("DHCP server: ", etherCard.dhcpip); // output IP address
  // of the DHCP server Enable

  attachInterrupt(digitalPinToInterrupt(pinInterrupt), onChange, FALLING);

  // Initialize MQTT client
  /*mqttClient.setServer(mqttServer, mqttPort);
  if (!mqttClient.connected()) {
    lcd.setCursor(0, 0);
    lcd.print(F("MQTT client not"));
    lcd.setCursor(0, 1);
    lcd.print(F("connected..."));
    Serial.println(F("[Arduino] MQTT client not connected to broker"));
    delay(2000);
    lcd.clear();
    //exit(1); //enable i
  } else {
    Serial.println(F("[Arduino] MQTT client connected to broker"));
  }
  */
}

void loop()
{
  timer = millis();
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    lastDebounceTime = millis();

    Serial.print((Count * 8.75) / 100);

    lcd.setCursor(0, 0);
    lcd.print(F("Wind Speed:"));
    lcd.setCursor(0, 1);
    lcd.print((Count * 8.75) / 100);

    Count = 0;

    Serial.println(F("m/s"));

    lcd.print(F("m/s"));
  }
  delay(1);
  /*
  // Read DHT sensor
  //float temperature = dht.readTemperature();
  //float humidity = dht.readHumidity();
    // Publish DHT sensor data to MQTT broker
    if (timer - lastPublish >= publishInterval) {
      //snprintf(mqttBuffer, sizeof(mqttBuffer), "%.2f", temperature);
      //snprintf(mqttBuffer2, sizeof(mqttBuffer2), "%.2f", humidity);
      if (mqttClient.connect("arduino-client", mqttUser, mqttPass)) {
        mqttClient.publish(mqttTopic, mqttBuffer);
        mqttClient.publish(mqttTopic, mqttBuffer2);
        Serial.println("[Arduino] Data published to MQTT broker");
      } else {
        Serial.println("[Arduino] Failed to connect to MQTT broker");
      }
      lastPublish = timer;
    }

  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    Serial.println("[Arduino] Reconnecting to MQTT broker...");
    if (mqttClient.connect("arduino-client", mqttUser, mqttPass)) {
     Serial.println("[Arduino] Connected to MQTT broker");
   } else {
      Serial.println("[Arduino] Failed to connect to MQTT broker");
    }
  }

  //mqttClient.loop();
*/
  // Handle incoming Ethernet packets
  //  etherCard.packetLoop(etherCard.packetReceive());
}
