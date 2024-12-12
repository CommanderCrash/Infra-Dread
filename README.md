
##  Infra-Dread   v1.3                                                          

infra-dread is a Python script designed for Raspberry Pi to Brute force and send infrared (IR) codes using a connected IR transmitter. The script allows you to send specific IR codes in sequence, randomize the codes, or count up from a given starting code. It provides flexibility in customizing the header pulse, header space, one pulse, one space, zero pulse, zero space, pulse trail, and gap durations, enabling compatibility with a wide range of IR devices.

Hardware Setup:
        Connect an IR transmitter to the Raspberry Pi GPIO pin (specified in the transmit_pin variable) that supports hardware PWM.
        Make sure you have the necessary hardware and wiring connections to send IR signals successfully.

Install Dependencies:
        Before using the script, ensure that the required packages are installed on your Raspberry Pi.
        The script relies on the RPi.GPIO library, which should be pre-installed on most Raspberry Pi systems. If it's not already installed, you can install it using the following command:
```
sudo apt-get update
sudo apt-get install python3-rpi.gpio
```
Run the Script:

 Save the script on your Raspberry Pi. (Only tested with a raspberry pi)
 Open a terminal and navigate to the directory containing the script.
 To view available options, run:


Command Line Arguments:
```
python3 infra-dread.py -h
usage: infra-dread.py [-h] [-l LENGTH] [-r] [-m CODE] [-p PREAMBLE]
                      [-x REPEAT] [--header_pulse HEADER_PULSE]
                      [--header_space HEADER_SPACE] [--one_pulse ONE_PULSE]
                      [--one_space ONE_SPACE] [--zero_pulse ZERO_PULSE]
                      [--zero_space ZERO_SPACE] [--ptrail PTRAIL] [--gap GAP]
                      [--frequency FREQUENCY] [--duty DUTY] [-sl START_FROM]

Send IR codes in sequence, random, or count-up mode.

optional arguments:
  -h, --help            show this help message and exit
  -l LENGTH, --length LENGTH
                        Number of bits for the IR codes (default: 32).
  -r, --random          Enable random mode (default is counting-up).
  -m CODE, --code CODE  IR code to send in hex format (e.g., 0x02A1).
  -p PREAMBLE, --preamble PREAMBLE
                        Fixed preamble IR code to send in hex format (e.g.,
                        0x7FFFF).
  -x REPEAT, --repeat REPEAT
                        Number of times to repeat sending the code (default:
                        1).
  --header_pulse HEADER_PULSE
                        Header pulse duration (microseconds, default: 4058).
  --header_space HEADER_SPACE
                        Header space duration (microseconds, default: 3964).
  --one_pulse ONE_PULSE
                        One pulse duration (microseconds, default: 514).
  --one_space ONE_SPACE
                        One space duration (microseconds, default: 1980).
  --zero_pulse ZERO_PULSE
                        Zero pulse duration (microseconds, default: 514).
  --zero_space ZERO_SPACE
                        Zero space duration (microseconds, default: 981).
  --ptrail PTRAIL       Pulse trail duration (microseconds, default: 514).
  --gap GAP             Gap duration (microseconds, default: 64729).
  --frequency FREQUENCY
                        Carrier frequency (Hz, default: 38000).
  --duty DUTY           Duty cycle for the PWM signal (default: 50.0).
  -sl START_FROM, --start_from START_FROM
                        Start counting up from the specified hex code.
```


LIRC conf file to get info on remote to bruteforce
```
  bits           24
  flags SPACE_ENC|CONST_LENGTH
  eps            30
  aeps          100

  header       4058  3964
  one           514  1980
  zero          514   981
  ptrail        514
  gap          64729
```
So command will be
```
python3 infra-dread.py --header_pulse 4058 --header_space 3964 --one_pulse 514 --one_space 1980 --zero_pulse 514 --zero_space 981 --ptrail 514 --gap 64729 -l 24
```
or
```
python3 infra-dread.py --header_pulse 4058 --header_space 3964 --one_pulse 514 --one_space 1980 --zero_pulse 514 --zero_space 981 --ptrail 514 --gap 64729 -r -l 24
```
Notes:
    The script is designed for Raspberry Pi with an IR transmitter connected to the specified GPIO pin pin 17 (change the transmit_pin variable if necessary).
    Ensure that the GPIO pin supports hardware PWM for accurate IR transmission.
    For successful IR transmission, configure the duration of the header pulse, header space, one pulse, one space, zero pulse, zero space, pulse trail, and gap according to the requirements of the target IR device. You can take a look at a lirc.conf remote file for all vaules for arguments like header gap pulse etc....
    Use the appropriate options to send specific IR codes, randomize codes, or count up from a given starting code.
    The script does not require any additional Python packages beyond the standard RPi.GPIO library.

License:

This script is provided under the MIT License. Feel free to use, modify, and distribute it as per the terms of the MIT License.
Author:

The script is developed by Commander Crash of 29A Society

