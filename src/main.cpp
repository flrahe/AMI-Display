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

#include <CAN_config.h>
#include <ESP32CAN.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <M5GFX.h>
#include <RTClib.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFiMulti.h>
#include <time.h>
#include "secrets.h"


#define CAN_SPEED CAN_SPEED_500KBPS
#define CAN_TX_PIN GPIO_NUM_26
#define CAN_RX_PIN GPIO_NUM_36

#define MAX_POWER_DISP 6000.0
#define MIN_POWER_DISP -10000.0

#define WIFI_WAIT 2



#define RTC_IS_DS3231
#ifdef RTC_IS_DS3231
    #define RTC_SDA GPIO_NUM_17
    #define RTC_SCL GPIO_NUM_16
#endif


#ifdef RTC_IS_DS3231
    RTC_DS3231 rtc;
    TwoWire myWire(1);
    #define RTCSYNC
    #define HAVERTC
#endif






#ifdef RTCSYNC
    WiFiUDP ntpUDP;
    NTPClient timeClient(ntpUDP, "de.pool.ntp.org");
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

CAN_device_t CAN_cfg;             // CAN Config
const int rx_queue_size = 10;     // Receive Queue size

unsigned long previousMillis = 0; // will store last time a CAN Message was send
const int interval = 1 * 200;   // interval at which send CAN Messages (milliseconds)

unsigned long previousMillis_WifiReconnect = 0; // will store last time a CAN Message was send
const int interval_WifiReconnect = 30 * 1000;   // interval at which send CAN Messages (milliseconds)

uint8_t count = 0;

#ifdef RTCSYNC
unsigned long previousMillis_RTCsync = 0;
const int interval_RTCsync = 60 * 1000; // interval at which send CAN Messages (milliseconds)
bool was_NTPoffline = true;
#endif

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


void init_RTC()
{
#ifdef RTC_IS_DS3231
    canvas_error.print("Initializing RTC... ");
    canvas_error.pushSprite(0, 0);
    myWire.begin(RTC_SDA, RTC_SCL, 100000);
    if (!rtc.begin(&myWire))
    {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        canvas_error.setTextColor(RED);
        canvas_error.println(" NOK!");
        canvas_error.pushSprite(0, 0);
        canvas_error.setTextColor(GREEN);
        while (1)
            delay(100);
    }
    canvas_error.println(" Ok");
    canvas_error.pushSprite(0, 0);

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
            if (counter > WIFI_WAIT*10)
            {
                break;
            }
        }
        if (WiFi.status() != WL_CONNECTED) {
            canvas_error.setTextColor(RED);
            canvas_error.println(" NOK!");
            canvas_error.pushSprite(0, 0);
            canvas_error.setTextColor(GREEN);
        }
        else {
            canvas_error.setTextColor(GREEN);
            canvas_error.println(" OK");
            canvas_error.pushSprite(0, 0);
        }
    }
}

void reconnect_wifi()
{
    WiFi.disconnect();
    WiFi.reconnect();
}

void sync_rtc()
{
#ifdef RTCSYNC
    if (was_NTPoffline)
    {
        timeClient.begin(); // Start NTP client
        was_NTPoffline = false;
    }

    timeClient.update();                                  // Retrieve current epoch time from NTP server
    unsigned long unix_epoch = timeClient.getEpochTime(); // Get epoch time
    rtc.adjust(DateTime(unix_epoch));
#endif
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
    float help_power =  car_data.batt.voltage * car_data.batt.current;
    float peak_power = 0.3*car_data.batt.power + 0.7*help_power;
    car_data.batt.power = 0.6*car_data.batt.power + 0.4*help_power;

    if (peak_power > car_data.batt.power_max )
    {
        car_data.batt.power_max = peak_power;
    }

    if (peak_power < car_data.batt.power_min )
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

void parse_can(CAN_frame_t *rx_frame)
{
    switch (rx_frame->MsgID)
    {
    case 0x713:
        decode_display(rx_frame->data.u8);
        break;
    case 0x580:
        decode_batt48_state(rx_frame->data.u8);
        break;
    case 0x581:
        decode_odo(rx_frame->data.u8);
        break;
    case 0x582:
        decode_charge(rx_frame->data.u8);
        break;
    case 0x593:
        decode_batt12_state(rx_frame->data.u8);
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
    if (car_data.display.ready)
    {
        color = GREEN;
        text = "Ready";
    }
    else {
        color = RED;
        text = "Waiting";
    }
    canvas_state.fillRoundRect(0, 0, 100, 40, 5, color);
    canvas_state.setTextColor(BLACK);
    canvas_state.setCursor(5, 10);
    canvas_state.setTextSize(1);
    canvas_state.print(text);

    if (car_data.display.hand_brake_active)
    {
        color = RED;
        text = "Break!";
    }
    else {
        color = GREEN;
        text = "OK";
    }
    canvas_state.fillRoundRect(110, 0, 100, 40, 5, color);
    canvas_state.setTextColor(BLACK);
    canvas_state.setCursor(135, 10);
    canvas_state.setTextSize(1);
    canvas_state.print(text);
    if ((car_data.display.gear == 'N') || (car_data.display.gear == 'D'))
    {
        color = GREEN;
    }
    else {
        color = RED;
    }
    canvas_state.fillRoundRect(220, 0, 100, 40, 5, color);
    canvas_state.setTextColor(BLACK);
    canvas_state.setCursor(255, 5);
    canvas_state.setTextSize(2);
    canvas_state.print(car_data.display.gear);

}

void plot_power()
{
    int angle;
    canvas_power.clear();
    canvas_power.drawArc(75, 75, 75, 74, 180, 360, RED);
    canvas_power.drawArc(75, 75, 75, 74, 0, 180, GREEN);

    angle = (int)(180 + abs(car_data.batt.power_min)/abs(MIN_POWER_DISP)*180.0);
    angle = angle > 360? 360 : angle;
    canvas_power.drawArc(75, 75, 75, 60, 180, angle, RED);

    angle = (int)(180 - car_data.batt.power_max/MAX_POWER_DISP*180.0);
    angle = angle < 0? 0 : angle;
    canvas_power.drawArc(75, 75, 75, 60, angle, 180, GREEN);

    if (car_data.batt.power < 0){
        angle = (int)(180 + abs(car_data.batt.power)/abs(MIN_POWER_DISP)*180.0);
        angle = angle > 360? 360 : angle;
        canvas_power.fillArc(75, 75, 75, 60, 180, angle, RED);
        canvas_power.setTextColor(RED);
    }
    else
    {
        angle = (int)(180 - car_data.batt.power/MAX_POWER_DISP*180.0);
        angle = angle < 0? 0 : angle;
        canvas_power.fillArc(75, 75, 75, 60, angle, 180, GREEN);
        canvas_power.setTextColor(GREEN);
    }  

    canvas_power.setCursor(30, 60);
    canvas_power.setTextSize(1.0);
    
    canvas_power.printf("%2.1f", abs(car_data.batt.power)/1000.0);
    int x = canvas_power.getCursorX();
    int y = canvas_power.getCursorY();
    canvas_power.setCursor(x+6, y+6);
    canvas_power.setTextSize(0.6);
    canvas_power.print("kW");

    canvas_power.setTextColor(RED);
    canvas_power.setCursor(120, 0);
    canvas_power.setTextSize(0.5);
    canvas_power.printf("%+2.1f", car_data.batt.power_min/1000.0);

    canvas_power.setTextColor(GREEN);
    canvas_power.setCursor(120, 140);
    canvas_power.setTextSize(0.5);
    canvas_power.printf("%+2.1f", car_data.batt.power_max/1000.0);


}

void setup()
{
    M5.begin();
    Serial.begin(115200); // This high baud rate is recommended if you want to
    // debug CAN data. These are coming at quite high rates.

    Serial.println("Basic Demo - AMI-Display");

    display.begin();

    canvas_time.setColorDepth(1); // mono color
    canvas_time.createSprite(150, 16);
    // canvas.setFont(&fonts::Font0);
    canvas_time.setFont(&fonts::FreeMono12pt7b);
    canvas_time.setTextSize(1.0);
    canvas_time.setPaletteColor(1, GREEN);

    canvas_error.setColorDepth(8); // mono color
    canvas_error.createSprite(display.width(), display.height());
    // canvas.setFont(&fonts::Font0);
    canvas_error.setFont(&fonts::FreeMono9pt7b);
    canvas_error.setTextSize(1.0);
    canvas_error.setPaletteColor(1, GREEN);
    canvas_error.setPaletteColor(1, RED);
    canvas_error.setTextScroll(true);
    canvas_error.setTextColor(GREEN);
    canvas_error.println("Basic Demo - AMI-Display");
    canvas_error.pushSprite(0,0);

    canvas_power.createSprite(160, 153);
    canvas_power.setFont(&fonts::FreeMonoBold18pt7b);
    canvas_power.setTextScroll(false);

    canvas_state.createSprite(320, 50);
    canvas_state.setFont(&fonts::FreeMonoBold9pt7b);
    canvas_state.setTextScroll(false);

    init_wifi();

    CAN_cfg.speed = CAN_SPEED;
    CAN_cfg.tx_pin_id = CAN_TX_PIN;
    CAN_cfg.rx_pin_id = CAN_RX_PIN;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module
    canvas_error.print("Init CAN Module...");
    canvas_error.pushSprite(0, 0);
    ESP32Can.CANInit();
    canvas_error.println(" Ok?");
    canvas_error.pushSprite(0, 0);

    init_RTC();
    if (WiFi.status() == WL_CONNECTED)
    {
        /* Pushing that date/time to the RTC */
        sync_rtc();
    }
    delay(1000);
    canvas_error.clear();
    canvas_error.pushSprite(0,0);

}


void loop()
{
    CAN_frame_t rx_frame;

    unsigned long currentMillis = millis();

    // if (WiFi.status() != WL_CONNECTED) {
    //     if (currentMillis - previousMillis_WifiReconnect >= interval_WifiReconnect
    //         || currentMillis < previousMillis_WifiReconnect) {
    //             reconnect_wifi();
    //     }
    // }

    // Receive next CAN frame from queue
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 1 * portTICK_PERIOD_MS) == pdTRUE)
    {
        if (rx_frame.FIR.B.RTR != CAN_RTR)
        {
            parse_can(&rx_frame);
        }
    }
    else
    {
        delay(2);
    }

    // // Display CAN Data
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;
        #ifdef HAVERTC
        DateTime dt = rtc.now();
        #else
        unsigned long dt = millis()/1000;
        #endif

        canvas_time.clear();
        canvas_time.setCursor(0,0);
        #ifdef HAVERTC
        canvas_time.printf("%02i", dt.hour());
        canvas_time.print(":");
        canvas_time.printf("%02i", dt.minute());
        canvas_time.print(":");
        canvas_time.printf("%02i", dt.second());
        #else
        canvas_time.printf("%02i", dt/(60*60));
        canvas_time.print(":");
        canvas_time.printf("%02i", (dt/60)%60);
        canvas_time.print(":");
        canvas_time.printf("%02i", dt%60);
        #endif
        canvas_time.println("");
        canvas_time.pushSprite(0, 0);

        plot_power();
        canvas_power.pushSprite(75, 40);

        plot_state();
        canvas_state.pushSprite(0, 200);
    }
    // delay(1);

    // if (currentMillis - previousMillis_RTCsync >= interval_RTCsync || currentMillis < previousMillis_RTCsync) {
    //     previousMillis_RTCsync = currentMillis;
    //     if (WiFi.status() == WL_CONNECTED) {
    //         sync_rtc();
    //         canvas.println("Synced RTC");
    //         canvas.pushSprite(0, 0);
    //     }
    //     else
    //     {
    //         canvas.println("Was offline");
    //         canvas.pushSprite(0, 0);
    //         was_NTPoffline = true;
    //     }
    // }
}

void managing_loop()
{
}