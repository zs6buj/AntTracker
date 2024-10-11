# CRSF DECODER

Eric Stockenstrom, November 2023

This library came about because I simply could not find a quick and easy CRSF/ELRS decoding library. While the Betaflight and the other mainstream libraries are exemplary in their design and execution, they are tightly integrated into multi-platform, multi_protocol designs, and require time (often in short supply) to research and implement. The idea here was to create a terse library to read and decode the essentials quickly. This code was written at short notice and has not been thoroughly tested, so it comes with the usual caveats. The scope included only basic telemetry and RC values, and no attempt was made at this time to decode advanced features like routing, device control and the like. 

Please report issues, and PRs are encouraged.

terseCRSF was coded for a generic ESP32 platform, but should be easily adaptable. The CRSF telemetry stream was tested using Betaflight v4.4.2 running on a Holybro Kakute H7 V2, and the CRSF RC byte stream was tested on a Radiomaster TX16S running EdgeTX v2.9.1.

CRSF signals typically originate from either a "transmitter box" co-located with the pilot, or a flight-controller (FC) located remotely on a UAV ('plane, drone, rover..). 

In this scenario, assuming CRSF or ELRS is the protocol of choice, the transmitter box typically outputs an RC mode encoded crsf byte stream, and the FC typically outputs a telemetry mode crsf byte stream at 420000 b/s. However, the remote controller (transmitter box), having received ELRS telemetry and running EdgeTX or OpenTX, can also mirror/stream CRSF telemetry from one of the available UARTS, albeit at a slower 115200 b/s. Similarly, the receiver, co-located with the FC and having received ELRS RC telemetry, can stream radio control(RC) style CRSF

This code may be built to operate in either RC mode, to read and decode RC PWM values, or the code may be built to operate in TELEMETRY mode, to read and decode telemetry values.

The header file terseCRSF.h contains the #define RC_BUILD macro, to instruct an RC mode build. In the abscence of this macro, the default build is TELEMETRY mode.
Assuming TELEMETRY MODE, the macro TELEMETRY_SOURCE must be used to select either (1)Betaflight/Cleanflight (from the flight controller) or (2)EdgeTX/OpenTX (from the remote controller - transmitter box).

In the RC build, by enabling the #define SEND_SBUS macro, an SBUS signal can be output on the pin designated sbus_txPin.

In addition, the header file contains pin numbers and other setup information pertinent to CRSF and platform setup. They should work as they stand, but the user may wish to make changes here and there once the basic example demo is shown to work.

A vscode/platformio demo example can be found in the examples folder. Simply open the folder, build, flash and run.

Activate these demo macros in the header to show how the decoded data members of the CRSF class can be accessed. For each sensor ID type, the pertinent data values will be printed out on the monitor.

#define DEMO_PWM_VALUES

#define DEMO_SBUS

#define DEMO_CRSF_GPS

#define DEMO_CRSF_BATTERY

#define DEMO_CRSF_ATTITUDE

#define DEMO_CRSF_FLIGHT_MODE


These three macros may be helpful, and are self explanatory:

#define SHOW_BUFFER

#define SHOW_BYTE_STREAM

#define SHOW_LOOP_PERIOD
