#!/usr/bin/env python3
# Infra Dread v1.3(PIGPIO)
# By Commander Crash of 29A Society

import pigpio
import time
import argparse
import random
import sys
from collections import Counter

# Color definitions for terminal output
MAGENTA = "\033[95m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
WHITE = "\033[0m"

# Message of the Day (motd) in red
motd = f"""{RED}
  _____        __                  _____                     _ 
 |_   _|      / _|                |  __ \                   | |
   | |  _ __ | |_ _ __ __ _ ______| |  | |_ __ ___  __ _  __| |
   | | | '_ \|  _| '__/ _` |______| |  | | '__/ _ \/ _` |/ _` |
  _| |_| | | | | | | | (_| |      | |__| | | |  __/ (_| | (_| |
 |_____|_| |_|_| |_|  \__,_|      |_____/|_|  \___|\__,_|\__,_|
                                                               
{WHITE}"""
print(motd)

def initialize_gpio(gpio_pin, frequency, duty_cycle):
    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpio daemon. Ensure it is running.")
        sys.exit(1)
    pi.set_mode(gpio_pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(gpio_pin, frequency)
    pi.set_PWM_dutycycle(gpio_pin, duty_cycle)
    return pi

def send_ir_signal(pi, gpio, frequency, code, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode):
    def send_pulse(duration_us):
        pi.hardware_PWM(gpio, frequency, 500000)  # 50% duty cycle
        time.sleep(duration_us / 1_000_000.0)
        pi.hardware_PWM(gpio, frequency, 0)  # turn off

    def send_space(duration_us):
        time.sleep(duration_us / 1_000_000.0)

    if view_mode == 'b':
        print(f"{GREEN}Sending IR code: {MAGENTA}{bin(code)}{WHITE}")
    elif view_mode == 'h':
        print(f"{GREEN}Sending IR code: {YELLOW}0x{code:0{code_length//4}X}{WHITE}")

    send_pulse(header_pulse)
    send_space(header_space)

    for i in range(code_length):
        if code & (1 << (code_length - 1 - i)):
            send_pulse(one_pulse)
            send_space(one_space)
        else:
            send_pulse(zero_pulse)
            send_space(zero_space)

    send_pulse(ptrail)
    send_space(gap)

def send_fixed_code(preamble_code, preamble_length, code, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode, pi, gpio, frequency, repeat):
    for _ in range(repeat):
        if preamble_code is not None:
            send_ir_signal(pi, gpio, frequency, preamble_code, preamble_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode)
        send_ir_signal(pi, gpio, frequency, code, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode)
        time.sleep(0.05)  # Reduced sleep interval

def count_up_from_hex_starting(starting_code, preamble_code, preamble_length, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode, pi, gpio, frequency, repeat):
    code = int(starting_code, 16)
    tried_codes = set()
    total_possible_codes = 2 ** code_length

    try:
        while len(tried_codes) < total_possible_codes:
            if code not in tried_codes:
                for _ in range(repeat):
                    if preamble_code is not None:
                        send_ir_signal(pi, gpio, frequency, preamble_code, preamble_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode)
                    send_ir_signal(pi, gpio, frequency, code, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode)
                tried_codes.add(code)

            code += 1
            if code >= 2 ** code_length:
                break  # Exit the loop when all possible codes are tried

            time.sleep(0.05)  # Reduced sleep interval
    except KeyboardInterrupt:
        print("\nExiting the script.")
    finally:
        pi.stop()

def count_up_from_zero(preamble_code, preamble_length, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode, pi, gpio, frequency, repeat):
    code = 0
    tried_codes = set()
    total_possible_codes = 2 ** code_length

    try:
        while len(tried_codes) < total_possible_codes:
            if code not in tried_codes:
                for _ in range(repeat):
                    if preamble_code is not None:
                        send_ir_signal(pi, gpio, frequency, preamble_code, preamble_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode)
                    send_ir_signal(pi, gpio, frequency, code, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode)
                tried_codes.add(code)

            code += 1
            if code >= total_possible_codes:
                code = 0

            time.sleep(0.05)  # Reduced sleep interval
    except KeyboardInterrupt:
        print("\nExiting the script.")
    finally:
        pi.stop()

def random_mode(preamble_code, preamble_length, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode, pi, gpio, frequency, repeat):
    total_possible_codes = 2 ** code_length
    tried_codes = set()

    try:
        while len(tried_codes) < total_possible_codes:
            code = random.randint(0, total_possible_codes - 1)
            if code not in tried_codes:
                for _ in range(repeat):
                    if preamble_code is not None:
                        send_ir_signal(pi, gpio, frequency, preamble_code, preamble_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode)
                    send_ir_signal(pi, gpio, frequency, code, code_length, header_pulse, header_space, one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode)
                tried_codes.add(code)

            time.sleep(0.05)  # Reduced sleep interval
    except KeyboardInterrupt:
        print("\nExiting the script.")
    finally:
        pi.stop()

def parse_arguments():
    parser = argparse.ArgumentParser(description="Send and receive IR codes.")
    parser.add_argument("--gpio", type=int, default=18, help="GPIO pin number for sending IR (default: 18).")
    parser.add_argument("--recv_gpio", type=int, help="GPIO pin number for receiving IR.")
    parser.add_argument("-l", "--length", type=int, default=32, help="Number of bits for the IR codes (default: 32).")
    parser.add_argument("-r", "--random", action="store_true", help="Enable random mode (default is counting-up).")
    parser.add_argument("-m", "--code", type=str, help="IR code to send in hex format (e.g., 0x02A1).")
    parser.add_argument("-p", "--preamble", type=str, help="Fixed preamble IR code to send in hex format (e.g., 0x7FFFF).")
    parser.add_argument("-x", "--repeat", type=int, default=1, help="Number of times to repeat sending the code (default: 1).")
    parser.add_argument("--header_pulse", type=int, default=4058, help="Header pulse duration (microseconds, default: 4058).")
    parser.add_argument("--header_space", type=int, default=3964, help="Header space duration (microseconds, default: 3964).")
    parser.add_argument("--one_pulse", type=int, default=0, help="One pulse duration (microseconds, default: 514).")
    parser.add_argument("--one_space", type=int, default=0, help="One space duration (microseconds, default: 1980).")
    parser.add_argument("--zero_pulse", type=int, default=514, help="Zero pulse duration (microseconds, default: 514).")
    parser.add_argument("--zero_space", type=int, default=981, help="Zero space duration (microseconds, default: 981).")
    parser.add_argument("--ptrail", type=int, default=514, help="Pulse trail duration (microseconds, default: 514).")
    parser.add_argument("--gap", type=int, default=64729, help="Gap duration (microseconds, default: 64729).")
    parser.add_argument("--frequency", type=int, default=38000, help="Carrier frequency (Hz, default: 38000).")
    parser.add_argument("--duty", type=float, default=50.0, help="Duty cycle for the PWM signal (default: 50.0).")
    parser.add_argument("-sl", "--start_from", type=str, help="Start counting up from the specified hex code.")
    parser.add_argument("-v", "--view", type=str, choices=['b', 'h'], default='h', help="Output view mode: 'b' for binary, 'h' for hex (default: 'h').")
    parser.add_argument("--receive", action="store_true", help="Enable receiving mode to capture IR signals.")
    parser.add_argument("--test", action="store_true", help="Run test mode.")
    return parser.parse_args()

class IRReceiver:
    def __init__(self, pi, gpio, callback):
        self.pi = pi
        self.gpio = gpio
        self.callback = callback
        self._last_tick = None
        self._in_code = False

        self._cb = self.pi.callback(self.gpio, pigpio.EITHER_EDGE, self._pulse_cb)

    def _pulse_cb(self, gpio, level, tick):
        if self._last_tick is not None:
            pulse_length = pigpio.tickDiff(self._last_tick, tick)
            self._last_tick = tick
            if self._in_code:
                self.callback(gpio, level, pulse_length)
            else:
                if pulse_length > 1500:  # Adjust threshold as per your protocol
                    self._in_code = True
                    self.callback(gpio, level, pulse_length)
        else:
            self._last_tick = tick

    def cancel(self):
        self._cb.cancel()

def receive_ir_signal(pi, recv_gpio):
    received_signals = []

    def ir_callback(gpio, level, pulse_length):
        if level == 1:
            received_signals.append(pulse_length)
        elif level == 0:
            received_signals.append(-pulse_length)

    ir_receiver = IRReceiver(pi, recv_gpio, ir_callback)

    print(f"{YELLOW}Waiting to receive IR signal... Press Ctrl+C to stop.{WHITE}")
    try:
        while True:
            time.sleep(1)
            if received_signals:
                print(f"{GREEN}Received {len(received_signals)} pulses so far{WHITE}")
    except KeyboardInterrupt:
        print(f"\n{GREEN}IR signal received:{WHITE}")
        print(received_signals)
    finally:
        ir_receiver.cancel()
    
    return received_signals

def analyze_signal(received_signals):
    if not received_signals:
        print(f"{RED}Error: No signals received.{WHITE}")
        sys.exit(1)

    header_pulse = abs(received_signals[0])
    header_space = abs(received_signals[1])
    ptrail = abs(received_signals[-1])

    pulse_space_pairs = []

    for i in range(2, len(received_signals) - 1, 2):
        pulse = abs(received_signals[i])
        space = abs(received_signals[i + 1])
        pulse_space_pairs.append((pulse, space))

    # Find the most common pulse and space values for "one" and "zero" bits
    pulse_space_counter = Counter(pulse_space_pairs)
    most_common_pairs = pulse_space_counter.most_common(2)

    if len(most_common_pairs) >= 2:
        (one_pulse, one_space), _ = most_common_pairs[0]
        (zero_pulse, zero_space), _ = most_common_pairs[1]
    else:
        print(f"{RED}Error: Unable to determine one and zero pulse/space pairs.{WHITE}")
        sys.exit(1)

    gap = 0
    if len(received_signals) > 2:
        gap = abs(received_signals[-2])

    bit_count = len(pulse_space_pairs)

    return {
        "header_pulse": header_pulse,
        "header_space": header_space,
        "one_pulse": one_pulse,
        "one_space": one_space,
        "zero_pulse": zero_pulse,
        "zero_space": zero_space,
        "ptrail": ptrail,
        "gap": gap,
        "bit_count": bit_count
    }


def test_mode(pi, gpio_tx, gpio_rx, frequency, tolerance=120):
    test_params = [
        {"header_pulse": 9000, "header_space": 4000},
        {"header_pulse": 8000, "header_space": 3000},
        {"header_pulse": 2000, "header_space": 2000}
    ]
    one_pulse = 550
    one_space = 1620
    zero_pulse = 650
    zero_space = 600
    ptrail = 666
    gap = 66612
    code = 0x1  # Simple test code
    code_length = 32

    for params in test_params:
        send_ir_signal(pi, gpio_tx, frequency, code, code_length, 
                       params["header_pulse"], params["header_space"], 
                       one_pulse, one_space, zero_pulse, zero_space, 
                       ptrail, gap, 'h')
        time.sleep(1)  # Give time to receive the signal

    received_signals = receive_ir_signal(pi, gpio_rx)

    signal_params = analyze_signal(received_signals)

    def check_tolerance(sent, received, tolerance):
        return abs(sent - received) <= tolerance

    for i, params in enumerate(test_params):
        if not check_tolerance(params["header_pulse"], signal_params["header_pulse"], tolerance):
            print(f"{RED}Test {i+1} Failed: Header pulse mismatch (Sent: {params['header_pulse']}, Received: {signal_params['header_pulse']}){WHITE}")
        if not check_tolerance(params["header_space"], signal_params["header_space"], tolerance):
            print(f"{RED}Test {i+1} Failed: Header space mismatch (Sent: {params['header_space']}, Received: {signal_params['header_space']}){WHITE}")

    pi.stop()

def main():
    args = parse_arguments()
    pi = initialize_gpio(args.gpio, args.frequency, args.duty)

    if args.test:
        if not args.recv_gpio:
            print(f"{RED}Error: You must specify a GPIO pin for receiving IR with --recv_gpio.{WHITE}")
            sys.exit(1)
        
        test_mode(pi, args.gpio, args.recv_gpio, args.frequency)
        sys.exit(0)

    if args.receive:
        if not args.recv_gpio:
            print(f"{RED}Error: You must specify a GPIO pin for receiving IR with --recv_gpio.{WHITE}")
            sys.exit(1)
        
        received_signals = receive_ir_signal(pi, args.recv_gpio)
        
        if not received_signals:
            print(f"{RED}Error: No signals received. Ensure the IR receiver is connected and a signal is being sent.{WHITE}")
            pi.stop()
            sys.exit(1)
        
        signal_params = analyze_signal(received_signals)
        
        print(f"{GREEN}Extracted Signal Parameters:{WHITE}")
        for key, value in signal_params.items():
            print(f"{key}: {value} microseconds")

        pi.stop()
        sys.exit(0)

    preamble_code = int(args.preamble, 16) if args.preamble else None
    preamble_length = len(bin(preamble_code)) - 2 if preamble_code else 0

    repeat = args.repeat

    if args.code:
        send_fixed_code(preamble_code, preamble_length, int(args.code, 16), args.length, args.header_pulse, args.header_space, args.one_pulse, args.one_space, args.zero_pulse, args.zero_space, args.ptrail, args.gap, args.view, pi, args.gpio, args.frequency, repeat)
    elif args.random:
        random_mode(preamble_code, preamble_length, args.length, args.header_pulse, args.header_space, args.one_pulse, args.one_space, args.zero_pulse, args.zero_space, args.ptrail, args.gap, args.view, pi, args.gpio, args.frequency, repeat)
    elif args.start_from:
        count_up_from_hex_starting(args.start_from, preamble_code, preamble_length, args.length, args.header_pulse, args.header_space, args.one_pulse, args.one_space, args.zero_pulse, args.zero_space, args.ptrail, args.gap, args.view, pi, args.gpio, args.frequency, repeat)
    else:
        count_up_from_zero(preamble_code, preamble_length, args.length, args.header_pulse, args.header_space, args.one_pulse, args.one_space, args.zero_pulse, args.zero_space, args.ptrail, args.gap, args.view, pi, args.gpio, args.frequency, repeat)

    pi.stop()

if __name__ == "__main__":
    main()
