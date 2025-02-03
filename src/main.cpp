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
#include "secrets.h"

#define MAX_POWER_DISP 6000.0
#define MIN_POWER_DISP -10000.0

#define WIFI_WAIT 3

#define MY_NTP_SERVER "pool.ntp.org"

// choose your time zone from this list
// https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"

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

String daysNames[] = {
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday"};
String monthsNames[] = {
    "-",
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December"};

char gears[5] = "RNDX";

M5GFX display;
M5Canvas canvas_error(&display);
M5Canvas canvas_time(&display);
M5Canvas canvas_power(&display);
M5Canvas canvas_state(&display);
M5Canvas canvas_SD_sym(&display);
M5Canvas canvas_bus_led(&display);

uint32_t msg_counter = 1;

unsigned long previousMillis = 0; // will store last time a CAN Message was send
const int interval = 1 * 200;     // interval at which send CAN Messages (milliseconds)

unsigned long previousMillis_WifiReconnect = 0; // will store last time a CAN Message was send
const int interval_WifiReconnect = 30 * 1000;   // interval at which send CAN Messages (milliseconds)

uint8_t count = 0;

#ifdef RTCSYNC
unsigned long previousMillis_RTCsync = 0;
const int interval_RTCsync = 60 * 1000; // interval at which send CAN Messages (milliseconds)
bool was_NTPoffline = true;
#endif

uint32_t plotnum_counter = 0;

size_t bufferPointer = 0;
const size_t SDpacket = 4096 * 8;
uint8_t canBuffer[SDpacket + 100];
bool busOk = false;
File myfile;
bool store = true;
bool got_files = false;

typedef struct
{
    struct
    {
        char gear;
        uint8_t range;
        bool hand_brake_active;
        bool ready;
    } display;
    struct
    {
        float current;
        float voltage;
        float power;
        float power_max;
        float power_min;
        uint8_t soc;
        float voltage12;
    } batt;
    struct
    {
        float odometer;
        uint8_t speed;
    } odo;
    struct
    {
        uint16_t remain_time;
    } charge;
} car_data_struct;

car_data_struct car_data;

void clear_car_data()
{
    car_data.batt.current = 0.0;
    car_data.batt.power = 0.0;
    car_data.batt.power_max = 0.0;
    car_data.batt.power_min = 0.0;
    car_data.batt.soc = 0;
    car_data.batt.voltage12 = 0.0;
    car_data.batt.voltage = 0.0;

    car_data.charge.remain_time = 0;

    car_data.odo.odometer = 0.0;
    car_data.odo.speed = 0;

    car_data.display.gear = 'X';
    car_data.display.hand_brake_active = true;
    car_data.display.range = 0;
    car_data.display.ready = false;
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
    auto now = M5.Rtc.getDateTime();
    while (M5.Rtc.getDateTime().time.seconds == now.time.seconds)
    {
        delay(1);
    }
    M5.Rtc.setSystemTimeFromRtc();
    setenv("TZ", MY_TZ, 1);
    tzset();
#endif
}

void init_wifi()
{

    if (WiFi.status() != WL_CONNECTED)
    {
        canvas_error.setTextColor(GREEN);
        canvas_error.print("Connecting to Wifi...");
        canvas_error.pushSprite(0, 0);

        WiFi.begin(ssid, password);
        uint8_t counter = 0;
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(100);
            counter++;
            if (counter > WIFI_WAIT * 10)
            {
                break;
            }
        }
        if (WiFi.status() != WL_CONNECTED)
        {
            canvas_error.setTextColor(RED);
            canvas_error.println(" NOK!");
            canvas_error.pushSprite(0, 0);
            canvas_error.setTextColor(GREEN);
        }
        else
        {
            canvas_error.setTextColor(GREEN);
            canvas_error.println("   OK");
            canvas_error.pushSprite(0, 0);
        }
    }
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
            tm_now.tm_year+1900,
            tm_now.tm_mon,
            tm_now.tm_mday,
            tm_now.tm_hour,
            tm_now.tm_min,
            tm_now.tm_sec);
    myfile = SD.open(filename, FILE_WRITE);
    myfile.println(";$FILEVERSION=1.1");
    got_files = true;
    Serial.println(filename);
    Serial.flush();
}
#endif

void reconnect_wifi()
{
    WiFi.disconnect();
    WiFi.reconnect();
}

void ntp_sync_callb(struct timeval *t)
{
    Serial.println("System time synced!");
    Serial.println("Syncing RTC!");
    time_t myt = time(nullptr) + 1; // Advance one second.
    while (myt > time(nullptr))
    M5.Rtc.setDateTime(gmtime(&myt));
}

void init_ntp()
{
    canvas_error.printf("Iniializing NTP...\n    %s\n", MY_NTP_SERVER);
    canvas_error.pushSprite(0,0);
    configTzTime(MY_TZ, MY_NTP_SERVER);
    sntp_set_time_sync_notification_cb(ntp_sync_callb);
    sntp_set_sync_interval(60*60*1000);
}

void decode_display(uint8_t data[8])
{
    car_data.display.gear = gears[(uint8_t)((data[0] >> 7) | ((data[1] & 1) << 1))];

    car_data.display.range = (uint8_t)(data[0] & 127);

    car_data.display.ready = (bool)(data[2] & 4);

    car_data.display.hand_brake_active = (bool)(data[3] & 32);
}

void decode_batt48_state(uint8_t data[8])
{
    car_data.batt.current = ((int16_t)(data[1] << 8 | data[0])) / 10.0;

    car_data.batt.voltage = ((int16_t)(data[3] << 8 | data[2])) / 100.0;

    // Smoothing of power calculation
    // Values are quite dynamic
    float help_power = car_data.batt.voltage * car_data.batt.current;
    float peak_power = 0.3 * car_data.batt.power + 0.7 * help_power;
    car_data.batt.power = 0.6 * car_data.batt.power + 0.4 * help_power;

    if (peak_power > car_data.batt.power_max)
    {
        car_data.batt.power_max = peak_power;
    }

    if (peak_power < car_data.batt.power_min)
    {
        car_data.batt.power_min = peak_power;
    }

    car_data.batt.soc = (uint8_t)data[5];
}

void decode_odo(uint8_t data[8])
{
    car_data.odo.odometer = ((data[6] << 8 | data[5]) << 8 | data[4]) / 10.0;

    car_data.odo.speed = (uint8_t)data[7];
}

void decode_charge(uint8_t data[8])
{
    car_data.charge.remain_time = (uint16_t)(data[1] << 8 | data[0]);
}

void decode_batt12_state(uint8_t data[8])
{
    car_data.batt.voltage12 = (data[1] << 8 | data[0]) / 100.0;
}

uint16_t read_state = 0;
void parse_can(CanFrame rx_frame)
{
    switch (rx_frame.identifier)
    {
    case 0x713:
        decode_display(rx_frame.data);
        break;
    case 0x580:
        decode_batt48_state(rx_frame.data);
        break;
    case 0x581:
        decode_odo(rx_frame.data);
        break;
    case 0x582:
        decode_charge(rx_frame.data);
        break;
    case 0x593:
        decode_batt12_state(rx_frame.data);
        break;
    default:
        break;
    }
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
    myfile.flush();
    myfile.close();
    SD.end();
}

void write_CAN_buffer()
{
    if (got_files)
    {
        size_t written = myfile.write(canBuffer, bufferPointer);
        myfile.flush();
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

bool lastOn = true;
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

void init_gui()
{
    canvas_time.setColorDepth(1);
    canvas_time.createSprite(150, 16);
    canvas_time.setFont(&fonts::FreeMono12pt7b);
    canvas_time.setTextSize(1.0);
    canvas_time.setPaletteColor(1, GREEN);

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

#ifdef LOG_SD
    canvas_SD_sym.setColorDepth(8);
    canvas_SD_sym.setPaletteColor(1, GREEN);
    canvas_SD_sym.setPaletteColor(2, DARKGREEN);
    canvas_SD_sym.setPaletteColor(3, RED);
    canvas_SD_sym.createSprite(14, 20);

#endif
}
void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
    Serial.begin(19200);
    M5.Power.begin();
    display.begin();

    setenv("TZ", MY_TZ, 1);
    tzset();

    clear_car_data();

    Serial.println("Basic Demo - AMI-Display");

    canvas_error.setColorDepth(8);
    canvas_error.createSprite(display.width(), display.height());
    canvas_error.setFont(&fonts::Font0);
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

    init_wifi();

#ifdef LOG_SD
    init_sd(true);
#endif

    canvas_error.print("Init CAN Module...");
    canvas_error.pushSprite(0, 0);

    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    ESP32Can.setRxQueueSize(20);
    ESP32Can.setTxQueueSize(5);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

    if (ESP32Can.begin())
    {
        canvas_error.println("      OK");
    }
    else
    {
        canvas_error.setColor(RED);
        canvas_error.println("    NOK!");
        canvas_error.setColor(GREEN);
    }
    canvas_error.pushSprite(0, 0);

    delay(1000);
    canvas_error.clear();
    canvas_error.pushSprite(0, 0);
    canvas_error.deleteSprite();
    init_gui();
    write_sym(got_files);
}

uint16_t fail_counter = 0;
unsigned long last_can = 0;
tm tm_now;
time_t now;
void loop()
{
    CanFrame rxFrame;
    unsigned long currentMillis = millis();

    // if (WiFi.status() != WL_CONNECTED) {
    //     if (currentMillis - previousMillis_WifiReconnect >= interval_WifiReconnect
    //         || currentMillis < previousMillis_WifiReconnect) {
    //             reconnect_wifi();
    //     }
    // }

    // Receive next CAN frame from queue
    if (ESP32Can.readFrame(rxFrame, 10))
    {
        busOk = true;
        fail_counter = 0;
        if (rxFrame.flags != TWAI_MSG_FLAG_RTR)
        {
            parse_can(rxFrame);
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

            if (fail_counter < 10) delay(2);
            else if (fail_counter < 40) delay(20);
            else
            {
                busOk = false;
                last_can = currentMillis;
                clear_car_data();
                delay(90);
            }
        }
        else delay(90);
    }

    if (currentMillis - previousMillis >= interval)
    {
        plotnum_counter++;
        previousMillis = currentMillis;
        time(&now);
        localtime_r(&now, &tm_now);

        canvas_time.clear();
        canvas_time.setCursor(0, 0);
        canvas_time.printf("%02i", tm_now.tm_hour);
        canvas_time.print(":");
        canvas_time.printf("%02i", tm_now.tm_min);
        canvas_time.print(":");
        canvas_time.printf("%02i", tm_now.tm_sec);
        canvas_time.println("");
        canvas_time.pushSprite(0, 0);

        if ((!busOk) & (currentMillis > (last_can + 60 * 1000)) & got_files)
        {
            clear_car_data();
            Serial.println("closinfg SD");
            closeSD();
            write_sym(got_files);
        }

        plotBusOk();

        plot_power();
        canvas_power.pushSprite(75, 40);

        plot_state();
        canvas_state.pushSprite(0, 200);
    }
}

void managing_loop()
{
}