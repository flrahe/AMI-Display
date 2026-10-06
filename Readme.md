# Display and Logging for Citroen AMI / Opel Rocks-e


# ${\color{red}Warning!}$
${\color{red}Working\space with \space the \space CAN \space on \space your \space car \space can \space be \space very \space dangerous.}$
${\color{red}This \space is \space just \space for \space research \space purpose.}$
${\color{red}If \space you \space use \space this, \space you'll \space do \space it \space at \space your \space own \space risk.}$

## Introduction
The intetion is to display and gather more information from the Citroen AMI. Ist hat two CAN interfaces at the ODB2 plug. The standard one at pins 6 (CAN High)/ 14 (CAN Low) with signal groud at pin 5.

On this bus already some reasearch has been done. This project extends this by two means:
* Cleaning up the signal definitions
* Using this information to decode the data on a microcontroller and display it

I'm no professional programmer. So the code will not be very optimized and not very clean. If there is real interest I welcome other to help this project growing.

<img src="./doc/images/AMI-Display.jpg" width=250>

Picture of power fed into the battery during charging.

## Outlook
This actual state is very basic and just the work of a few hours. Since there is interest in this state, the project is published as it is now. There is no good definitions of pins etc. This all needs to be done. Actually its less than a prove of conceptC and heavily work in progress.

First intention is to get things running. In the second step it might be made "nicer".

A few things which are planned to be included:
* Publishing different data via MQTT:
    * directly via WIFI, if available
    * via bluetooth to a smartphone which acts as bridge to MQTT
* Logging of CAN raw data (maybe just reduced due to performace/space issues)
* Using data in the car. i.e. Use the gear information to trigger a reverse camera
* Deeper alaysis of the "performance" (maybe with erxternal programs which analyze log data)
    * Battery health
        * Inner resistance
        * Monitoring of capacity
    * Perfoemace reduction monitoring (i.e. with lower temperatures)
    * Cumulation of consumed and charged power (energy)
    * Monitoring of 12V battery. Some people reported it got drained.

## Setup
This project is a PlatformIO project in VSCode. "Just" clone or download this repository and open it in VSCode with PlatformIO plugin installed. This hopefully installs all needed libraries and frameworks.

There are three profiles defined in [platformio.ini](platformio.ini) which contain most configuration for the three devices
- Core
- Core2
- CoreS3.

Unfortunetely I can only test on Core and CoreS3.

### secrets.h
You have to create a secrets.h file in te src directory. An example is put in [secrets_example.h](./src/secrets_example.h). Here you have to put your wifi and MQTT credentials. The secrets.h is in the gitignore not to accidentially upload it to github.

## Wiring and Hardware
Actually it is based on an m5Stack core (tested on m5Stack grey), which I had laying around. Since I'm using M5Unified, M5GFX and not using any of the additional hardware of the m5stack grey, it should work on other m5Stacks. For the actual sleep behaviour see [Power and sleep](#power-and-sleep). Unfortunately it is seems to be broken on eraly m5. Mine takes 10mA in sleep mode, which is way too high for longer use. It makes even no difference in deep sleep or light sleep.

### Addidiona hardware needed
#### RTC
Since there is no RTC in the m5 Core, I added a DS3231 wired to a second i2c bus. The second bus is needed, because there exists already i2c devices on the first bus having the same adress.

 Wiring:
* PIN17 -> SDA of RTC
* PIN16 -> SCL of RTC

There exists very small ones of these RTC modules, where you can solder the backup battery to the site, so it fits on an m5stack bus shield.

Newer m5 have an RTC included, but it is actually not used in this version of the software. Actually you may need to manually deactivate the clock stuff or rewitre it (plans for future).

#### Can module
I'm using [Unit CAN](https://docs.m5stack.com/en/unit/can) for CAN communication. I tested the COMMU module, but I could not get the MCP2515 working. Several other reported problems with that module too. The Unit CAN worked immediatley and so far reliable. And ***"The built-in DC-DC isolated power chip can isolate noise and interference and prevent damage to sensitive circuits."*** So it seems to be a good choice.

The module is connected via UART:
* RX of module -> PIN35 of m5
* TX of module -> PIN25 of m5

The wiring of the can side is straightforward:
* H -> CAN High at ODB2 (pin 6)
* L -> CAN Low at  ODB2 (pin 14)
* G -> Signal Ground at ODB" (pin 5).

## MQTT
When WiFi is available, the data is published to the MQTT broker from [secrets.h](./src/secrets_example.h).

### ami/state
A JSON message every 15 s and immediately when the charging state changes, e.g.:

```json
{"soc": 85, "soctr": 0, "eBR": 5.6, "eBRtr": 0.00, "eBO": 71.4, "eBOtr": 0.00, "eBB": 66.0, "eBBtr": 0.00,
 "eCI": 62.3, "eCItr": 0.00, "odo": 4696.2, "odotr": 0.0, "rng": 58, "state": "stopped",
 "cur": 0.00, "pwr": 0, "chrgn": 0, "rdy": 0, "gear": "X", "spd": 0, "m5soc": 100, "rssi": -41, "wfiok": 1}
```

* `soc`, `odo`, `rng` and the energy counters (`eBR`, `eBO`, `eBB`, `eCI`, in kWh) keep their last value while the car is off. The `...tr` values are the trip values.
* `rng` is left out as long as no valid range is known.
* The live values `state` (`running`/`stopped`), `cur`, `pwr`, `chrgn`, `rdy`, `gear` and `spd` are always sent. They are reset to 0 / `X` when the related CAN frames are missing for 2 s.
* `volt`, `pwrmax` and `pwrmin` are only sent while battery frames are received.
* Before powering off (battery operation only) `{"state": "off", ...}` is sent.

### ami/evcc/*
Single retained values, mainly for [evcc](https://evcc.io), but usable by any other system:

| Topic | Value |
|---|---|
| `ami/evcc/soc` | state of charge in % |
| `ami/evcc/range` | remaining range in km |
| `ami/evcc/odometer` | odometer in km |
| `ami/evcc/status` | `C` = charging, `A` = not charging |

Because they are retained, the last values are still available while the car is parked and the M5Stack is off. Values are only published if they are valid (> 0), so a start without state file does not overwrite them. A change of the charging state is sent immediately.

The car can only tell charging / not charging, not whether the cable is plugged in. Therefore `A` is reported when not charging.

## Remaining range while charging
The car sends a range of 0 while charging and switching off (0 is only valid with an empty battery). In this case the range is estimated from the soc using a table with the range for every 5 % soc (0, 5, ... 100 %):

* Between two points the range is interpolated linearly and rounded to full km.
* The table is learned while driving: once per soc change, the first valid range from the car corrects the two neighbouring points by 5 % of the error (`RANGE_LEARN_RATE`), weighted by their distance. So it adapts slowly and single unusual trips don't change it much.
* It starts linear with 70 km at 100 %.
* The table is stored as `rngtbl` in `state.json` on the SD card. Delete this entry to restart learning.
* While the car reports a valid range (driving), this value is used unchanged. The estimate is also used after start and when the CAN bus stops, so the stored range is updated after charging.

## evcc integration
The car can be added to evcc as custom vehicle using the `ami/evcc/*` topics. The MQTT broker must be configured in evcc (`mqtt:` section).

```yaml
vehicles:
  - name: woodstock
    type: custom
    title: Woodstock
    capacity: 5.5 # kWh
    features:
      - streaming
    soc:
      source: mqtt
      topic: ami/evcc/soc
    range:
      source: mqtt
      topic: ami/evcc/range
    odometer:
      source: mqtt
      topic: ami/evcc/odometer
    status:
      source: mqtt
      topic: ami/evcc/status
```

* `features: streaming` is needed, otherwise evcc reads the soc only while charging (default poll mode `charging`) or at most every 60 min. With it the values are read in every cycle, as long as the vehicle is assigned to a loadpoint.
* `status` is used by evcc to identify the vehicle when charging starts. The charging control itself uses the charger status.
* evcc calculates the remaining charge time itself from soc, capacity and charge power. A remaining time from the vehicle is not used. The slower charging of the last ~10 % is not covered by evcc's model (it is fixed in the code and made for much higher charge powers), so plan with some margin.

## Power and sleep
* Running from USB / car power (without battery) the M5Stack never sleeps. After the car stops sending CAN data it gets power for about 2 more minutes and keeps sending MQTT messages, so the state `stopped` is reported.
* Running from battery, it powers off 2 minutes after the last CAN frame (`M5.Power.powerOff()`). The CoreS3 (AXP2101) powers on again when USB / car power returns or by the power button.

## CAN data
In CAN_information [CAN_information](./CAN_information/) you'll find the actual dbc/sym files which help decoding the can frames.
