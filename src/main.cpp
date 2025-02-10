/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
 * @Hardwares: M5Core + Unit CAN
 * @Platform Version: Arduino M5Stack Board Manager v2.1.3
 * @Dependent Library:
 * M5GFX@^0.2.3: https://github.com/m5stack/M5GFX
 * M5Unified@^0.2.2: https://github.com/m5stack/M5Unified
 * ESP32-Arduino-CAN：https://github.com/miwagner/ESP32-Arduino-CAN
 */

#include <M5Unified.h>
#include <M5GFX.h>
#include <WiFi.h>
#include <SD.h>
#include <time.h>
#include <esp_sntp.h>
#include <ESP32-TWAI-CAN.hpp>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <FS.h>
#include <ArduinoJson.h>
#include "secrets.h"
#define FORMAT_SPIFFS_IF_FAILED true

#define MAX_POWER_DISP 6000.0
#define MIN_POWER_DISP -10000.0
#define WIFI_WAIT 3

// choose your time zone from this list
// https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"
#define MY_NTP_SERVER "pool.ntp.org"

#ifdef RTC_IS_DS3231
#define NEED_RTC_LIB
#endif

#ifdef NEED_RTC_LIB
#include <RTClib.h>
#include <Wire.h>
#define RTCSYNC
#define HAVERTC
#ifdef RTC_IS_DS3231
RTC_DS3231 rtc;
TwoWire myWire(1);
#endif
#endif

#ifdef M5_RTC
#define HAVERTC
#define RTCSYNC
#endif

WiFiClient espClient;
PubSubClient MQTTclient(espClient);

char gears[5] = "RNDX";

M5GFX display;
M5Canvas canvas_error(&display);
M5Canvas canvas_time(&display);
M5Canvas canvas_power(&display);
M5Canvas canvas_state(&display);
M5Canvas canvas_SD_sym(&display);
M5Canvas canvas_bus_led(&display);
M5Canvas canvas_wifi(&display);
M5Canvas canvas_energy(&display);

unsigned long previousMillisDisplay = 0; // will store last time diplay was updates
const int interval = 1 * 200;            // interval at which display will be updated
uint32_t plotnum_counter = 0;            // reduces the update of the bus "LED"

uint32_t msg_counter = 1;
size_t bufferPointer = 0;
const size_t SDpacket = 4096 * 16;
uint8_t canBuffer[SDpacket + 100];
File myCanFile;
bool store = true;
bool got_files = false;

size_t mqttbufferPointer = 0;
const size_t mqttPacket = 512;
char mqttBuffer[mqttPacket];
unsigned long lastMillisMQTTLoop = 0; // will store last time diplay was updates
const int mqttLoopInterval = 1000;            // interval at which display will be updated

unsigned long previousMillisStore = 0; // will store last time diplay was updates
const int storeInterval = 5*60*1000;            // interval at which display will be updated

uint8_t brightness = 0;

struct car_data_struct
{
    struct disp_struct
    {
        bool ok;
        char gear;
        uint8_t range;
        bool hand_brake_active;
        bool ready;
        uint32_t lastMsgTime;
    };
    struct batt_struct
    {
        bool ok;
        float current;
        float voltage;
        float power;
        float last_power;
        float power_max;
        float power_min;
        uint8_t soc;
        uint32_t lastMsgTime;
    };
    struct batt_temp_strcut
    {
        bool ok;
        int8_t temp_1;
        int8_t temp_2;
        uint32_t lastMsgTime;
    };
    struct odo_struct
    {
        bool ok;
        float odometer;
        uint8_t speed;
        uint32_t lastMsgTime;
    };
    struct charge_struct
    {
        bool ok;
        uint16_t remain_time;
        uint32_t lastMsgTime;
    };
    struct state_struct
    {
        bool isCharging;
        uint32_t lastMsgTime;
    };
    struct long_state_struct
    {
        uint8_t soc;
        uint8_t socTS;
        float odometer;
        float odometerTS;
        uint8_t range;
        uint8_t rangeTS;
        int32_t smallEnergyChargeIn;
        int32_t smallEnergyBattOut;
        int32_t smallEnergyBattBal;
        int32_t smallEnergyRec;
        uint32_t energyChargeIn;
        uint32_t energyBattOut;
        uint32_t energyBattBal;
        uint32_t energyRec;
        uint32_t energyChargeInTS;
        uint32_t energyBattOutTS;
        uint32_t energyBattBalTS;
        uint32_t energyRecTS;
        time_t lastWrtTime;
        time_t lastSendTime;
        bool updated;
    };
    disp_struct display;
    batt_struct batt;
    batt_temp_strcut batt_temp;
    odo_struct odo;
    charge_struct charge;
    state_struct state;
    long_state_struct long_state;
};

car_data_struct car_data;

int charge_counter = 0;
bool busOk = false;

void init_gui(void);
void init_wifi(bool have_display);
void init_RTC(void);
void init_ntp(void);
void ntp_sync_callb(struct timeval *t);
bool setup_and_check_can(void);

void parse_can(CanFrame rx_frame, uint32_t currentmillis);
void decode_display(uint8_t data[8], uint32_t currentmillis);
void decode_batt48_state(uint8_t data[8], uint32_t currentmillis);
void decode_odo(uint8_t data[8], uint32_t currentmillis);
void decode_charge(uint8_t data[8], uint32_t currentmillis);
void decode_batt_temp(uint8_t data[8], uint32_t currentmillis);

void init_car_data(void);
void clear_car_data(void);
void check_for_charging(void);

void plot_power(void);
void plotEnergy(void);
void plot_state(void);
void plotBusOk(void);
void plotWifi(void);

void init_gui(void);
void write_sym(bool ok);
void init_sd(bool startup);
void closeSD();
void store_can(CanFrame rx_frame);

void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void writeFile(fs::FS &fs, const char *path, const char *message);

boolean reconnectMQTT(void);
void addToMessage(const char *data, size_t len);
void prepare_message();

void readState();
void writeState();
void prepareState();

void setup()
{
    brightness = M5.Lcd.getBrightness();
    M5.Lcd.setBrightness(0);

    auto cfg = M5.config();
    cfg.serial_baudrate = 9600;
    M5.begin(cfg);
    M5.Power.begin();
    Serial.begin(9600);
    init_car_data();
    busOk = setup_and_check_can();
    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
    {
        Serial.println("SPIFFS Mount Failed");
        display.begin();
        M5.Lcd.setBrightness(brightness);
        display.println("Formatting SPIFFS....");
        display.println("Please Wait 15 sec....");
        delay(15000);
        display.println("Rebooting in 2 sec....");
        delay(2000);
        ESP.restart();
    }
    readState();
    bool key_state = M5.BtnPWR.getState();
    if (!M5.Power.isCharging() & (M5.Power.getBatteryLevel() < 99) & (M5.Power.getBatteryLevel() != -1) & (key_state == 0))
    {
        if (!busOk)
        {
            time_t now;
            time(&now);
            if (car_data.long_state.lastSendTime + 3600 < now)
            {   
                uint8_t counter=0;
                init_wifi(false);
                MQTTclient.setServer(mqtt_server, 1883);
                while ((WiFi.status() != WL_CONNECTED) & (counter < 25)) {
                    counter++;
                    delay(200);
                }
                delay(100);
                if (WiFi.status() == WL_CONNECTED) {
                    if (reconnectMQTT())
                    {
                        prepare_message();
                        MQTTclient.publish("ami/state", mqttBuffer);
                        MQTTclient.publish("ami/state", "Going to sleep.");
                        MQTTclient.loop();
                        delay(200);
                        MQTTclient.loop();
                    }
                    else {
                    }
                }
                time(&car_data.long_state.lastSendTime);
                writeState();
                delay(10);
            }
            M5.Power.timerSleep(15);
        }
    }
    display.begin();
    display.powerSaveOff();
    M5.Lcd.setBrightness(brightness);

    SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);

    setenv("TZ", MY_TZ, 1);
    tzset();

    Serial.println("Basic Demo - AMI-Display");

    canvas_error.setColorDepth(8);
    canvas_error.createSprite(display.width(), display.height());
    canvas_error.setFont(&fonts::FreeMono9pt7b);
    canvas_error.setTextSize(1.0);
    canvas_error.setPaletteColor(1, GREEN);
    canvas_error.setPaletteColor(2, RED);
    canvas_error.setTextScroll(true);
    canvas_error.setTextColor(GREEN);
    canvas_error.println("Basic Demo - AMI-Display");
    canvas_error.pushSprite(0, 0);

    init_RTC();

    init_ntp();

    init_wifi(true);

#ifdef LOG_SD
    init_sd(true);
#endif

    MQTTclient.setServer(mqtt_server, 1883);

    delay(1000);
    canvas_error.clear();
    canvas_error.pushSprite(0, 0);
    canvas_error.deleteSprite();
    init_gui();
    write_sym(got_files);
    listDir(SPIFFS, "/", 0);
}

long lastReconnectAttempt = 0;
long lastMQTTMessage = 0;
uint16_t fail_counter = 0;
unsigned long last_can = 0;

void loop()
{
    CanFrame rxFrame;
    unsigned long currentMillis = millis();

    // Receive next CAN frame from queue
    if (ESP32Can.readFrame(rxFrame, 10))
    {
        busOk = true;
        fail_counter = 0;
        if (rxFrame.flags != TWAI_MSG_FLAG_RTR)
        {
            parse_can(rxFrame, millis());
#ifdef LOG_SD
            store_can(rxFrame);
#endif
        }
    }
    else
    {
        if (busOk)
        {
            fail_counter++;

            if (fail_counter < 10)
                delay(2);
            else if (fail_counter < 40)
                delay(20);
            else
            {
                busOk = false;
                last_can = currentMillis;
                clear_car_data();
            }
        }
    }

    if (currentMillis - previousMillisDisplay >= interval)
    {
        tm tm_now;
        time_t now;

        plotnum_counter++;
        previousMillisDisplay = currentMillis;
        time(&now);
        localtime_r(&now, &tm_now);

        canvas_time.clear();
        canvas_time.setCursor(0, 0);
        canvas_time.printf("%02i", tm_now.tm_hour);
        canvas_time.print(":");
        canvas_time.printf("%02i", tm_now.tm_min);
        canvas_time.print(":");
        canvas_time.printf("%02i", tm_now.tm_sec);
        canvas_time.printf(" %3i%%", M5.Power.getBatteryLevel());
        canvas_time.println("");
        canvas_time.pushSprite(0, 0);

        if ((!busOk) & (currentMillis > (last_can + 60 * 1000)) & got_files)
        {

            clear_car_data();
            if (car_data.long_state.updated) {
                writeState();
                // car_data.long_state.updated = false;
            }
            else {
                Serial.println("No Update in state!");
            }
            Serial.println("closinfg SD");
            closeSD();
            write_sym(got_files);
        }

        if ((!busOk) & (currentMillis > (last_can + 120 * 1000)))
        {
            if (!M5.Power.isCharging() & (M5.Power.getBatteryLevel() < 99) & (M5.Power.getBatteryLevel() != -1))
            {
                M5.Power.timerSleep(15);
            }
        }

        plotWifi();

        plotBusOk();

        plot_power();
        canvas_power.pushSprite(25, 38);

        plotEnergy();
        canvas_energy.pushSprite(215, 30);

        plot_state();
        canvas_state.pushSprite(0, 200);
    }

    if (currentMillis - lastMQTTMessage > 15000 | lastMQTTMessage == 0) {
        long now = millis();
        if (!MQTTclient.connected())
        {
            if (now - lastReconnectAttempt > 5000 | lastReconnectAttempt==0)
            {
                lastReconnectAttempt = now;
                // Attempt to reconnect
                if (reconnectMQTT())
                {
                    lastReconnectAttempt = 0;
                }
            }
        }
        else {
            lastReconnectAttempt = 0;
        }

        if (lastReconnectAttempt == 0)
        {
            if (now - lastMQTTMessage > 15000 | lastMQTTMessage == 0)
            {
                lastMQTTMessage = now;
                prepare_message();
                MQTTclient.publish("ami/state", mqttBuffer);
                time(&car_data.long_state.lastSendTime);
            }
        }
        else {
            lastMQTTMessage = now;
            if (lastMQTTMessage > 10000) lastMQTTMessage -= 10000;
        }
    }

    if (currentMillis - lastMillisMQTTLoop >= mqttLoopInterval) {
        lastMillisMQTTLoop = currentMillis;
        MQTTclient.loop();
    }

    if (currentMillis - previousMillisStore >= storeInterval) {
        previousMillisStore = millis();
        if (car_data.long_state.updated) {
            writeState();
            // car_data.long_state.updated = false;
        }
        else {
            Serial.println("No Update of data...");
        }
    }

}

void init_RTC()
{
#ifdef HAVERTC
    canvas_error.print("Initializing RTC... ");
    canvas_error.pushSprite(0, 0);
#ifdef NEED_RTC_LIB
    myWire.begin(RTC_SDA, RTC_SCL, 100000);
    if (!rtc.begin(&myWire))
#endif
#ifdef M5_RTC
        if (!M5.Rtc.begin())
#endif
        {
            Serial.println("Couldn't find RTC");
            Serial.flush();
            canvas_error.setTextColor(RED);
            canvas_error.println("   NOK!");
            canvas_error.pushSprite(0, 0);
            canvas_error.setTextColor(GREEN);
            while (1)
                delay(100);
        }
    canvas_error.println("    OK");
    canvas_error.pushSprite(0, 0);

    setenv("TZ", "UTC", 1);
    tzset();
#ifdef M5_RTC
    M5.Rtc.setSystemTimeFromRtc();
#endif
#ifdef NEED_RTC_LIB
    timeval epoch = {(long int)rtc.now().unixtime(), 0};
    settimeofday((const timeval *)&epoch, 0);
#endif
    setenv("TZ", MY_TZ, 1);
    tzset();
#endif
}

void init_wifi(bool have_display)
{
    WiFi.setAutoReconnect(true);
    WiFi.setHostname("ami-display");
    WiFi.setSleep(true);
    if (WiFi.status() != WL_CONNECTED)
    {
        if (have_display) {
            canvas_error.setTextColor(GREEN);
            canvas_error.print("Init Wifi...");
            canvas_error.pushSprite(0, 0);
        }

        WiFi.begin(ssid, password);

        if (have_display) {
            canvas_error.setTextColor(GREEN);
            canvas_error.println("   Backgrounding");
            canvas_error.pushSprite(0, 0);
        }
    }
}

bool setup_and_check_can()
{
    CanFrame rxFrame;
    uint8_t i = 0;
    uint8_t fail_counter = 0;
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    ESP32Can.setRxQueueSize(20);
    ESP32Can.setTxQueueSize(0);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
    ESP32Can.begin();

    for (i = 0; i < 2; i++)
    {
        ESP32Can.readFrame(rxFrame, 1);
    }

    for (i = 0; i < 10; i++)
    {
        if (ESP32Can.readFrame(rxFrame, 5))
        {
            busOk = true;
            return true;
        }
    }
    return false;
}

#ifdef LOG_SD
void init_sd(bool startup)
{
    uint8_t counter = 0;
    if (startup)
    {
        canvas_error.setTextColor(GREEN);
        canvas_error.print("Starting SD...");
        canvas_error.pushSprite(0, 0);

        while (false == SD.begin(SD_SPI_CS_PIN, SPI, 25000000))
        {
            delay(500);
            counter++;
            if (counter > 10)
            {
                canvas_error.setTextColor(RED);
                canvas_error.println("        NOK!");
                canvas_error.pushSprite(0, 0);
                canvas_error.setTextColor(GREEN);
                return;
            }
        }

        uint8_t cardType = SD.cardType();

        if (cardType == CARD_NONE)
        {
            canvas_error.setTextColor(RED);
            canvas_error.println("NO SD-Card found!");
            canvas_error.pushSprite(0, 0);
            canvas_error.setTextColor(GREEN);
            got_files = false;
            return;
        }

        canvas_error.println("          OK");
    }
    else
    {
        while (false == SD.begin(SD_SPI_CS_PIN, SPI, 25000000))
        {
            delay(100);
            counter++;
            if (counter >= 2)
            {
                got_files = false;
                return;
            }
        }
    }

    tm tm_now;
    time_t now;

    time(&now);
    localtime_r(&now, &tm_now);

    char filename[40];
    sprintf(filename, "/%04i-%02i-%02i_%02i-%02i-%02i.trc",
            tm_now.tm_year + 1900,
            tm_now.tm_mon + 1,
            tm_now.tm_mday,
            tm_now.tm_hour,
            tm_now.tm_min,
            tm_now.tm_sec);
    myCanFile = SD.open(filename, FILE_WRITE);
    myCanFile.println(";$FILEVERSION=1.1");
    got_files = true;
    Serial.println(filename);
    Serial.flush();
}
#endif

void ntp_sync_callb(struct timeval *t)
{
    Serial.println("System time synced!");
    Serial.println("Syncing RTC!");
    time_t myt = time(nullptr) + 1; // Advance one second.
    while (myt > time(nullptr))
    {
#ifdef M5_RTC
        M5.Rtc.setDateTime(gmtime(&myt));
#endif
#ifdef NEED_RTC_LIB
        rtc.adjust(DateTime(myt));
#endif
    }
}

void init_ntp()
{
    canvas_error.printf("Iniializing NTP...\n    %s\n", MY_NTP_SERVER);
    canvas_error.pushSprite(0, 0);
    configTzTime(MY_TZ, MY_NTP_SERVER);
    sntp_set_time_sync_notification_cb(ntp_sync_callb);
    sntp_set_sync_interval(60 * 60 * 1000);
}

void parse_can(CanFrame rx_frame, uint32_t currentmillis)
{
    switch (rx_frame.identifier)
    {
    case 0x713:
        decode_display(rx_frame.data, currentmillis);
        break;
    case 0x580:
        decode_batt48_state(rx_frame.data, currentmillis);
        check_for_charging();
        break;
    case 0x581:
        decode_odo(rx_frame.data, currentmillis);
        break;
    case 0x582:
        decode_charge(rx_frame.data, currentmillis);
        break;
    case 0x594:
        decode_batt_temp(rx_frame.data, currentmillis);
        break;
    default:
        break;
    }
}

void decode_display(uint8_t data[8], uint32_t currentmillis)
{
    car_data.display.gear = gears[(uint8_t)((data[0] >> 7) | ((data[1] & 1) << 1))];

    car_data.display.range = (uint8_t)(data[0] & 127);
    car_data.long_state.range = car_data.display.range;

    car_data.display.ready = (bool)(data[2] & 4);

    car_data.display.hand_brake_active = (bool)(data[3] & 32);

    car_data.display.ok = true;
}

void decode_batt48_state(uint8_t data[8], uint32_t currentmillis)
{
    car_data.batt.current = ((int16_t)(data[1] << 8 | data[0])) / 10.0;

    car_data.batt.voltage = ((int16_t)(data[3] << 8 | data[2])) / 100.0;

    // Smoothing of power calculation
    // Values are quite dynamic
    float help_power = car_data.batt.voltage * car_data.batt.current;
    float peak_power = 0.3 * car_data.batt.power + 0.7 * help_power;
    car_data.batt.power = 0.6 * car_data.batt.power + 0.4 * help_power;
    float akt_power = 0.5*help_power + 0.5* car_data.batt.last_power;
    if (car_data.batt.lastMsgTime > 0 &
        currentmillis > car_data.batt.lastMsgTime &
        (currentmillis - car_data.batt.lastMsgTime) < 500)
    {
        int32_t energy = (int32_t)(akt_power * (currentmillis - car_data.batt.lastMsgTime));
        if (!car_data.state.isCharging) {
            car_data.long_state.smallEnergyBattBal += -energy; // Energy negativ during discharge, but soting as positive value
            if (car_data.long_state.smallEnergyBattBal > 3600000) {
                car_data.long_state.energyBattBal++;
                car_data.long_state.smallEnergyBattBal -=3600000;
            }
            else if (car_data.long_state.smallEnergyBattBal < -3600000) {
                if (car_data.long_state.energyBattBal > 0)
                    car_data.long_state.energyBattBal--;
                car_data.long_state.smallEnergyBattBal +=3600000;
            }

            if (energy > 0) {
                car_data.long_state.smallEnergyRec += energy;
                if (car_data.long_state.smallEnergyRec > 3600000) {
                    car_data.long_state.energyRec++;
                    car_data.long_state.smallEnergyRec -=3600000;
                }
            } else
            {
                car_data.long_state.smallEnergyBattOut += -energy;
                if (car_data.long_state.smallEnergyBattOut > 3600000) {
                    car_data.long_state.energyBattOut++;
                    car_data.long_state.smallEnergyBattOut -=3600000;
                }
            }
            
        }
        else {
            car_data.long_state.smallEnergyChargeIn += energy; // Energy negativ during discharge, but soting as positive value
            if (car_data.long_state.smallEnergyChargeIn > 3600000) {
                car_data.long_state.energyChargeIn++;
                car_data.long_state.smallEnergyChargeIn -=3600000;
            }
        }
    }

    if (peak_power > car_data.batt.power_max)
    {
        car_data.batt.power_max = peak_power;
    }

    if (peak_power < car_data.batt.power_min)
    {
        car_data.batt.power_min = peak_power;
    }

    car_data.batt.soc = (uint8_t)data[5];
    car_data.long_state.soc = car_data.batt.soc;

    car_data.batt.lastMsgTime = currentmillis;
    car_data.batt.last_power = help_power;
    car_data.batt.ok = true;
    car_data.long_state.updated = true;
}

void decode_odo(uint8_t data[8], uint32_t currentmillis)
{
    car_data.odo.odometer = ((data[6] << 8 | data[5]) << 8 | data[4]) / 10.0;
    car_data.long_state.odometer = car_data.odo.odometer;

    car_data.odo.speed = (uint8_t)data[7];

    car_data.odo.ok = true;
    car_data.long_state.updated = true;
}

void decode_charge(uint8_t data[8], uint32_t currentmillis)
{
    car_data.charge.remain_time = (uint16_t)(data[1] << 8 | data[0]);
    car_data.charge.ok = false;
}

void decode_batt_temp(uint8_t data[8], uint32_t currentmillis)
{
    car_data.batt_temp.temp_1 = (int8_t)data[0];
    car_data.batt_temp.temp_2 = (int8_t)data[3];
    car_data.batt_temp.ok = true;
}

void check_for_charging()
{
    if (
        car_data.batt.ok &
        (car_data.batt.current > 0) &
        car_data.display.ok &
        (car_data.display.hand_brake_active) &
        car_data.odo.ok &
        (car_data.odo.speed) == 0)
    {
        charge_counter++;
        if (charge_counter >= 20)
        {
            car_data.state.isCharging = true;
        }
        if (charge_counter > 30)
        {
            charge_counter = 30;
        }
    }
    else
    {
        charge_counter -= 3;
        if (charge_counter < 0)
        {
            charge_counter = 0;
        }
        if (charge_counter == 0)
        {
            car_data.state.isCharging = false;
        }
    }
}

void init_car_data()
{
    clear_car_data();
    car_data.long_state.updated = false;
}

void clear_car_data()
{
    car_data.batt.current = 0.0;
    car_data.batt.power = 0.0;
    car_data.batt.power_max = 0.0;
    car_data.batt.power_min = 0.0;
    car_data.batt.soc = 0;
    car_data.batt.voltage = 0.0;
    car_data.batt.ok = false;

    car_data.batt_temp.temp_1 = 0;
    car_data.batt_temp.temp_2 = 0;
    car_data.batt_temp.ok = false;

    car_data.charge.remain_time = 0;
    car_data.charge.ok = false;

    car_data.odo.odometer = 0.0;
    car_data.odo.speed = 0;
    car_data.odo.ok = false;

    car_data.display.gear = 'X';
    car_data.display.hand_brake_active = true;
    car_data.display.range = 0;
    car_data.display.ready = false;
    car_data.display.ok = false;
    car_data.state.isCharging = false;
}

void init_gui()
{
    canvas_time.setColorDepth(1);
    canvas_time.createSprite(200, 16);
    canvas_time.setFont(&fonts::FreeMono12pt7b);
    canvas_time.setTextSize(1.0);
    canvas_time.setPaletteColor(1, GREEN);

    canvas_energy.setColorDepth(8);
    canvas_energy.createSprite(100, 160);
    canvas_energy.setFont(&fonts::FreeMono12pt7b);
    canvas_energy.setTextSize(1.0);
    canvas_energy.setPaletteColor(1, GREEN);
    canvas_energy.setPaletteColor(2, RED);


    canvas_power.setColorDepth(8);
    canvas_power.setPaletteColor(1, GREEN);
    canvas_power.setPaletteColor(2, RED);
    canvas_power.createSprite(160, 153);
    canvas_power.setFont(&fonts::FreeMonoBold18pt7b);
    canvas_power.setTextScroll(false);

    canvas_state.setColorDepth(8);
    canvas_state.setPaletteColor(1, GREEN);
    canvas_state.setPaletteColor(2, RED);
    canvas_state.createSprite(320, 50);
    canvas_state.setFont(&fonts::FreeMonoBold12pt7b);
    canvas_state.setTextScroll(false);

    canvas_bus_led.setColorDepth(8);
    canvas_bus_led.setPaletteColor(1, GREEN);
    canvas_bus_led.setPaletteColor(2, RED);
    canvas_bus_led.createSprite(20, 20);
    canvas_bus_led.setColor(RED);
    canvas_bus_led.fillCircle(10, 10, 4);
    canvas_bus_led.pushSprite(300, 0);

    canvas_wifi.setColorDepth(8);
    canvas_wifi.setPaletteColor(1, GREEN);
    canvas_wifi.setPaletteColor(2, DARKGREEN);
    canvas_wifi.setPaletteColor(3, RED);
    canvas_wifi.createSprite(24, 20);

#ifdef LOG_SD
    canvas_SD_sym.setColorDepth(8);
    canvas_SD_sym.setPaletteColor(1, GREEN);
    canvas_SD_sym.setPaletteColor(2, DARKGREEN);
    canvas_SD_sym.setPaletteColor(3, RED);
    canvas_SD_sym.createSprite(14, 20);
#endif
}

void plot_state()
{
    int color;
    String text;
    canvas_state.clear();
    canvas_state.setFont(&fonts::FreeMonoBold12pt7b);
    if (car_data.display.ready)
    {
        color = GREEN;
        text = "Ready";
        canvas_state.setTextSize(1);
        canvas_state.setCursor(10, 10);
    }
    else
    {
        color = RED;
        text = "Waiting";
        canvas_state.setTextSize(0.9);
        canvas_state.setCursor(7, 12);
    }
    canvas_state.fillRoundRect(0, 0, 100, 40, 5, color);
    canvas_state.setTextColor(BLACK);
    // canvas_state.setCursor(5, 10);
    canvas_state.print(text);

    if (car_data.display.hand_brake_active)
    {
        color = RED;
        text = "Brake!";
        canvas_state.setCursor(121, 10);
    }
    else
    {
        color = GREEN;
        text = "OK";
        canvas_state.setCursor(145, 10);
    }
    canvas_state.fillRoundRect(110, 0, 100, 40, 5, color);
    canvas_state.setTextColor(BLACK);

    canvas_state.setTextSize(1);
    canvas_state.print(text);
    if ((car_data.display.gear == 'N') || (car_data.display.gear == 'D'))
    {
        color = GREEN;
    }
    else
    {
        color = RED;
    }
    canvas_state.fillRoundRect(220, 0, 100, 40, 5, color);
    canvas_state.setTextColor(BLACK);
    canvas_state.setCursor(258, 2);
    canvas_state.setFont(&fonts::FreeMonoBold24pt7b);
    canvas_state.print(car_data.display.gear);
    canvas_state.setFont(&fonts::FreeMonoBold12pt7b);
}

void plot_power()
{
    int angle;
    canvas_power.clear();
    canvas_power.drawArc(75, 75, 75, 74, 180, 360, RED);
    canvas_power.drawArc(75, 75, 75, 74, 0, 180, GREEN);

    angle = (int)(180 + abs(car_data.batt.power_min) / abs(MIN_POWER_DISP) * 180.0);
    angle = angle > 360 ? 360 : angle;
    canvas_power.drawArc(75, 75, 75, 60, 180, angle, RED);

    angle = (int)(180 - car_data.batt.power_max / MAX_POWER_DISP * 180.0);
    angle = angle < 0 ? 0 : angle;
    canvas_power.drawArc(75, 75, 75, 60, angle, 180, GREEN);

    if (car_data.batt.power < 0)
    {
        angle = (int)(180 + abs(car_data.batt.power) / abs(MIN_POWER_DISP) * 180.0);
        angle = angle > 360 ? 360 : angle;
        canvas_power.fillArc(75, 75, 75, 60, 180, angle, RED);
        canvas_power.setTextColor(RED);
    }
    else
    {
        angle = (int)(180 - car_data.batt.power / MAX_POWER_DISP * 180.0);
        angle = angle < 0 ? 0 : angle;
        canvas_power.fillArc(75, 75, 75, 60, angle, 180, GREEN);
        canvas_power.setTextColor(GREEN);
    }

    canvas_power.setCursor(30, 60);
    canvas_power.setTextSize(1.0);

    canvas_power.printf("%2.1f", abs(car_data.batt.power) / 1000.0);
    int x = canvas_power.getCursorX();
    int y = canvas_power.getCursorY();
    canvas_power.setCursor(x + 6, y + 6);
    canvas_power.setTextSize(0.6);
    canvas_power.print("kW");

    canvas_power.setTextColor(RED);
    canvas_power.setCursor(120, 0);
    canvas_power.setTextSize(0.5);
    canvas_power.printf("%+2.1f", car_data.batt.power_min / 1000.0);

    canvas_power.setTextColor(GREEN);
    canvas_power.setCursor(120, 140);
    canvas_power.setTextSize(0.5);
    canvas_power.printf("%+2.1f", car_data.batt.power_max / 1000.0);
}

void plotEnergy(){
    canvas_energy.clear();
    canvas_energy.setCursor(0,0);
    canvas_energy.setTextColor(GREEN);
    canvas_energy.println("Energy");
    canvas_energy.printf("U%5i\n", (car_data.long_state.energyBattBal-car_data.long_state.energyBattBalTS));
    canvas_energy.printf("R%5i\n", (car_data.long_state.energyRec-car_data.long_state.energyRecTS));
    canvas_energy.printf("C%5i\n", (car_data.long_state.energyChargeIn-car_data.long_state.energyChargeInTS));
    canvas_energy.println("Temp");
    canvas_energy.printf("T1%3i'C\n", car_data.batt_temp.temp_1);
    canvas_energy.printf("T2%3i'C\n", car_data.batt_temp.temp_2);
}

void plotBusOk()
{
    if (busOk)
    {
        if (plotnum_counter % 10 == 0)
        {
            canvas_bus_led.clear();
            canvas_bus_led.pushSprite(300, 0);
        }
        else if (plotnum_counter % 5 == 0)
        {
            canvas_bus_led.setColor(GREEN);
            canvas_bus_led.fillCircle(10, 10, 4);
            canvas_bus_led.pushSprite(300, 0);
        }
    }
    else
    {
        canvas_bus_led.setColor(RED);
        canvas_bus_led.fillCircle(10, 10, 4);
        canvas_bus_led.pushSprite(300, 0);
    }
}

void plotWifi()
{
    canvas_wifi.clear();
    if (WiFi.status() != WL_CONNECTED)
    {
        canvas_wifi.setColor(RED);
        canvas_wifi.drawArc(12, 18, 0, 18, 230, 310);
        canvas_wifi.pushSprite(245, 0);
    }
    else
    {
        int strength = WiFi.RSSI();
        if (strength != 0)
        {
            canvas_wifi.setColor(DARKGREEN);
            strength = (int)((60 + (strength + 30)) / 2.5);
            if (strength > 19)
                strength = 18;
            else if (strength < 0)
                strength = 0;
            canvas_wifi.drawArc(12, 18, 0, 18, 230, 310);
            canvas_wifi.fillArc(12, 18, 0, strength, 230, 310);
            canvas_wifi.pushSprite(245, 0);
        }
        else
        {
            canvas_wifi.setColor(RED);
            canvas_wifi.drawArc(12, 18, 0, 18, 230, 310);
            canvas_wifi.pushSprite(245, 0);
        }
    }
}

boolean reconnectMQTT()
{
    if (MQTTclient.connect("ami_mqtt", mqtt_username, mqtt_password))
    {
        // Once connected, publish an announcement...
        MQTTclient.publish("ami/state", "reconnected");
    }
    return MQTTclient.connected();
}

void addToMessage(const char *data, size_t len)
{
    uint16_t i = 0;
    for (i = 0; i < len; i++)
    {
        mqttBuffer[mqttbufferPointer] = data[i];
        mqttbufferPointer++;
    }
}

void prepare_message()
{
    mqttbufferPointer = 0;
    char data[100];
    uint8_t i, j, str_len;
    addToMessage("{", 1);
    str_len = sprintf(data, "\"soc\": %i, ", car_data.long_state.soc);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"eBR\": %.1f, ", car_data.long_state.energyRec/1000.0);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"eBO\": %.1f, ", car_data.long_state.energyBattOut/1000.0);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"eBB\": %.1f, ", car_data.long_state.energyBattBal/1000.0);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"eCI\": %.1f, ", car_data.long_state.energyChargeIn/1000.0);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"odo\": %.1f,", car_data.long_state.odometer);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"rng\": %i,", car_data.long_state.range);
    addToMessage(data, str_len);

    if (car_data.batt.ok)
    {
        str_len = sprintf(data, "\"cur\": %.2f, ", car_data.batt.current);
        addToMessage(data, str_len);
        str_len = sprintf(data, "\"volt\": %.2f,", car_data.batt.voltage);
        addToMessage(data, str_len);
        str_len = sprintf(data, "\"pwr\": %.0f,", car_data.batt.power);
        addToMessage(data, str_len);
        str_len = sprintf(data, "\"pwrmax\": %.0f,", car_data.batt.power_max);
        addToMessage(data, str_len);
        str_len = sprintf(data, "\"pwrmin\": %.0f,", car_data.batt.power_min);
        addToMessage(data, str_len);
        str_len = sprintf(data, "\"chrgn\": %i,", car_data.state.isCharging);
        addToMessage(data, str_len);
    }

    str_len = sprintf(data, "\"m5soc\": %i,", M5.Power.getBatteryLevel());
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"rssi\": %i,", WiFi.RSSI());
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"wfiok\": %i,", WiFi.status() == WL_CONNECTED);
    addToMessage(data, str_len);
    mqttbufferPointer--;
    addToMessage("}", 1);
    mqttBuffer[mqttbufferPointer] = (char)0;
    mqttbufferPointer++;
}

#ifdef LOG_SD
bool write_sym_state = false;
void write_sym(bool ok)
{
    int color;
    if (ok)
    {
        if (write_sym_state)
        {
            color = GREEN;
            write_sym_state = false;
        }
        else
        {
            color = DARKGREEN;
            write_sym_state = true;
        }
    }
    else
    {
        color = RED;
    }

    canvas_SD_sym.fillRoundRect(0, 0, 14, 20, 3, color);
    canvas_SD_sym.drawLine(11, 0, 11, 6, BLACK);
    canvas_SD_sym.drawLine(11, 6, 14, 9, BLACK);
    canvas_SD_sym.floodFill(12, 1, BLACK);
    canvas_SD_sym.pushSprite(280, 0);
}

void closeSD()
{
    got_files = false;
    myCanFile.flush();
    myCanFile.close();
    SD.end();
}

void write_CAN_buffer()
{
    if (got_files)
    {
        size_t written = myCanFile.write(canBuffer, bufferPointer);
        myCanFile.flush();
        // Serial.println("Written.");

        if (written != bufferPointer)
        {
            closeSD();
        }
        write_sym(written == bufferPointer);
        bufferPointer = 0;
    }
    else
    {
        init_sd(false);
        if (got_files)
        {
            write_CAN_buffer();
        }
        bufferPointer = 0;
    }
}

void store_can(CanFrame rx_frame)
{
    char data[100];
    uint8_t i, j, str_len;

    str_len = sprintf(data, "%8d) %10.1f Rx %04X  %1i ", msg_counter, micros() / 1000.0, rx_frame.identifier, rx_frame.data_length_code);
    for (i = 0; i < str_len; i++)
    {
        canBuffer[bufferPointer] = data[i];
        bufferPointer++;
    }
    msg_counter++;
    for (i = 0; i < rx_frame.data_length_code; i++)
    {
        sprintf(data, " %02X", rx_frame.data[i]);
        for (j = 0; j < 3; j++)
        {
            canBuffer[bufferPointer] = data[j];
            bufferPointer++;
        }
    }
    canBuffer[bufferPointer] = '\n';
    bufferPointer++;
    if (bufferPointer >= SDpacket)
    {
        write_CAN_buffer();
    }
}
#endif

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("- failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("- file written");
    }
    else
    {
        Serial.println("- write failed");
    }
    file.close();
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("- failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.path(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}
void readState() {
    JsonDocument doc;
    if (SPIFFS.exists("/state.json")) {
        File file = SPIFFS.open("/state.json", "r");
        DeserializationError error = deserializeJson(doc, file);
        file.close();
        if (error)
            Serial.println(F("Failed to read file, using default configuration"));
        else {
            car_data.long_state.soc = doc["soc"] | 0;
            car_data.long_state.odometer = doc["odo"] | 0.0;
            car_data.long_state.range = doc["range"] | 0;
            car_data.long_state.smallEnergyChargeIn = doc["smEChrgIn"] | 0;
            car_data.long_state.smallEnergyBattOut = doc["smEBatOut"] | 0;
            car_data.long_state.smallEnergyRec = doc["smERec"] | 0;
            car_data.long_state.energyChargeIn = doc["eChrgIn"] | 0;
            car_data.long_state.energyBattOut = doc["eBatOut"] | 0;
            car_data.long_state.energyBattBal = doc["eBatBal"] | 0;
            car_data.long_state.energyRec = doc["eRec"] | 0;
            car_data.long_state.lastWrtTime = (time_t)doc["lstWrtT"] | (time_t)0;
            car_data.long_state.lastSendTime = (time_t)doc["lstSndT"] | (time_t)0;

            car_data.long_state.energyChargeInTS = car_data.long_state.energyChargeIn;
            car_data.long_state.energyBattOutTS = car_data.long_state.energyBattOut;
            car_data.long_state.energyBattBalTS = car_data.long_state.energyBattBal;
            car_data.long_state.energyRecTS = car_data.long_state.energyRec;
            car_data.long_state.updated = false;
        }
    }
    else
    {
        Serial.println("Could not read data!");
    }
    car_data.long_state.updated = false;
}

void writeState() {
    time(&car_data.long_state.lastWrtTime);
    prepareState();
    Serial.println("Write State");
    Serial.println(mqttBuffer);
    car_data.long_state.updated = false;
    writeFile(SPIFFS, "/state.json", mqttBuffer);
    // File file = SPIFFS.open("/state.json", "w");
    // for (uint16_t i=0; i<mqttbufferPointer; i++ ) {
    //     file.write(mqttBuffer[mqttbufferPointer]);
    // }
    // file.close();
    
}

void prepareState()
{
    mqttbufferPointer = 0;
    char data[100];
    uint8_t i, j, str_len;
    addToMessage("{", 1);

    str_len = sprintf(data, "\"soc\": %i, ", car_data.long_state.soc);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"odo\": %.1f,", car_data.long_state.odometer);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"range\": %i,", car_data.long_state.range);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"smEChrgIn\": %i,", (uint8_t) round(car_data.long_state.smallEnergyChargeIn/1000.0));
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"smEBatOut\": %i,", (uint8_t) round(car_data.long_state.smallEnergyBattOut/1000.0));
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"smERec\": %i,", (uint8_t) round(car_data.long_state.smallEnergyRec/1000.0));
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"eChrgIn\": %i,", car_data.long_state.energyChargeIn);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"eBatOut\": %i,", car_data.long_state.energyBattOut);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"eBatBal\": %i,", car_data.long_state.energyBattBal);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"eRec\": %i,", car_data.long_state.energyRec);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"lstWrtT\": %i,", car_data.long_state.lastWrtTime);
    addToMessage(data, str_len);
    str_len = sprintf(data, "\"lstSndT\": %i,", car_data.long_state.lastSendTime);
    addToMessage(data, str_len);
    mqttbufferPointer--;
    addToMessage("}", 1);
    mqttBuffer[mqttbufferPointer] = (char)0;
    mqttbufferPointer++;
}
