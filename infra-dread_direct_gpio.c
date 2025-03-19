/*
 * Infra Dread v2.0 (Direct GPIO version)
 * By Commander Crash of 29A Society
 * Uses direct GPIO access for better timing precision
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <argp.h>
#include <pthread.h>

// Raspberry Pi 4 GPIO registers
#define BCM2711_PERI_BASE    0xFE000000
#define GPIO_BASE            (BCM2711_PERI_BASE + 0x200000)
#define BLOCK_SIZE           (4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET *(gpio+7)  // sets   bits which are 1, ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1, ignores bits which are 0
#define GPIO_READ(g) (*(gpio+13)&(1<<(g))) // 0 if LOW, (1<<g) if HIGH

// Color definitions for terminal output
#define MAGENTA "\033[95m"
#define RED     "\033[91m"
#define GREEN   "\033[92m"
#define YELLOW  "\033[93m"
#define WHITE   "\033[0m"

// Used for signal handling
volatile sig_atomic_t keep_running = 1;

// GPIO memory mapping
static volatile uint32_t *gpio = NULL;
static int mem_fd = -1;

// Structure to hold program arguments
struct arguments {
    int gpio_pin;
    int recv_gpio;
    int length;
    bool random_mode;
    char* code;
    char* preamble;
    int repeat;
    int header_pulse;
    int header_space;
    int one_pulse;
    int one_space;
    int zero_pulse;
    int zero_space;
    int ptrail;
    int gap;
    int frequency;
    float duty;
    char* start_from;
    char view_mode;
    bool receive;
    bool test;
};

// Default values for command-line options
static struct arguments arguments = {
    .gpio_pin = 18,
    .recv_gpio = 19,
    .length = 32,
    .random_mode = false,
    .code = NULL,
    .preamble = NULL,
    .repeat = 1,
    .header_pulse = 4058,
    .header_space = 3964,
    .one_pulse = 0,
    .one_space = 0,
    .zero_pulse = 514,
    .zero_space = 981,
    .ptrail = 514,
    .gap = 64729,
    .frequency = 38000,
    .duty = 50.0,
    .start_from = NULL,
    .view_mode = 'h',
    .receive = false,
    .test = false
};

// Signal handler for clean exit on Ctrl+C
void signal_handler(int sig) {
    (void)sig; // Prevent unused parameter warning
    keep_running = 0;
}

// Initialize direct GPIO access
int setup_gpio() {
    printf("%sInitializing GPIO (direct access)...%s\n", YELLOW, WHITE);
    // Open /dev/mem
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0) {
        printf("%sFailed to open /dev/mem. Try running with sudo.%s\n", RED, WHITE);
        return -1;
    }
    // mmap GPIO
    void *gpio_map = mmap(
        NULL,                   // Any address in our space will do
        BLOCK_SIZE,             // Map length
        PROT_READ|PROT_WRITE,   // Enable reading & writing to mapped memory
        MAP_SHARED,             // Shared with other processes
        mem_fd,                 // File to map
        GPIO_BASE               // Offset to GPIO peripheral
    );
    close(mem_fd); // No need to keep mem_fd open after mmap
    mem_fd = -1;
    if (gpio_map == MAP_FAILED) {
        printf("%sFailed to map GPIO memory.%s\n", RED, WHITE);
        return -1;
    }
    // Always use volatile pointer!
    gpio = (volatile uint32_t *)gpio_map;
    printf("%sGPIO initialized successfully (direct access).%s\n", GREEN, WHITE);
    return 0;
}

// Clean up GPIO resources
void cleanup_gpio() {
    if (gpio != NULL) {
        munmap((void*)gpio, BLOCK_SIZE);
        gpio = NULL;
    }
    if (mem_fd != -1) {
        close(mem_fd);
        mem_fd = -1;
    }
}

// Microsecond sleep function for accurate timing
void sleep_us(int us) {
    struct timespec ts;
    ts.tv_sec = us / 1000000;
    ts.tv_nsec = (us % 1000000) * 1000;
    nanosleep(&ts, NULL);
}

// Generate carrier wave for a specific duration using software PWM
void generate_carrier(int gpio_pin, int duration_us, int frequency) {
    // Calculate period in microseconds
    int period_us = 1000000 / frequency;
    int high_time = period_us / 2; // 50% duty cycle
    int low_time = period_us - high_time;
    int cycles = duration_us / period_us;
    for (int i = 0; i < cycles; i++) {
        GPIO_SET = 1 << gpio_pin;
        sleep_us(high_time);
        GPIO_CLR = 1 << gpio_pin;
        sleep_us(low_time);
    }
    // Handle remaining time
    int remainder = duration_us % period_us;
    if (remainder > 0) {
        if (remainder > high_time) {
            GPIO_SET = 1 << gpio_pin;
            sleep_us(high_time);
            GPIO_CLR = 1 << gpio_pin;
            sleep_us(remainder - high_time);
        } else {
            GPIO_SET = 1 << gpio_pin;
            sleep_us(remainder);
            GPIO_CLR = 1 << gpio_pin;
        }
    }
}

// Send an IR signal with proper carrier modulation
void send_ir_signal(int gpio_pin, int frequency, uint64_t code, int code_length,
                   int header_pulse, int header_space, int one_pulse, int one_space,
                   int zero_pulse, int zero_space, int ptrail, int gap, char view_mode) {
    // Display the code being sent in the chosen format
    if (view_mode == 'b') {
        printf("%sSending IR code: %s", GREEN, MAGENTA);
        for (int i = code_length - 1; i >= 0; i--) {
            printf("%lu", (unsigned long)((code >> i) & 1));
        }
        printf("%s\n", WHITE);
    } else if (view_mode == 'h') {
        int hex_digits = (code_length + 3) / 4; // Round up to nearest multiple of 4
        printf("%sSending IR code: %s0x%0*lX%s\n", GREEN, YELLOW, hex_digits, (unsigned long)code, WHITE);
    }
    // Configure GPIO
    INP_GPIO(gpio_pin);
    OUT_GPIO(gpio_pin);
    // Send header pulse (carrier on)
    generate_carrier(gpio_pin, header_pulse, frequency);
    // Header space (no carrier)
    GPIO_CLR = 1 << gpio_pin;
    sleep_us(header_space);
    // Send data bits
    uint64_t mask = 1ULL << (code_length - 1);
    for (int i = 0; i < code_length; i++) {
        if (code & mask) {
            // Send a "1" bit
            generate_carrier(gpio_pin, one_pulse, frequency);
            // Space (no carrier)
            GPIO_CLR = 1 << gpio_pin;
            sleep_us(one_space);
        } else {
            // Send a "0" bit
            generate_carrier(gpio_pin, zero_pulse, frequency);
            // Space (no carrier)
            GPIO_CLR = 1 << gpio_pin;
            sleep_us(zero_space);
        }
        mask >>= 1;
    }
    // Send trailing pulse
    generate_carrier(gpio_pin, ptrail, frequency);
    // Final gap (no carrier)
    GPIO_CLR = 1 << gpio_pin;
    sleep_us(gap);
}

// Send a fixed IR code with proper handling of repeats
void send_fixed_code(uint64_t preamble_code, int preamble_length, uint64_t code, int code_length,
                    int header_pulse, int header_space, int one_pulse, int one_space,
                    int zero_pulse, int zero_space, int ptrail, int gap, char view_mode,
                    int gpio_pin, int frequency, int repeat) {
    // Send the full code first
    if (preamble_length > 0) {
        send_ir_signal(gpio_pin, frequency, preamble_code, preamble_length, header_pulse, header_space,
                      one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
    }
    send_ir_signal(gpio_pin, frequency, code, code_length, header_pulse, header_space,
                  one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
    // For repeats, many protocols use a specific repeat frame format
    int repeat_gap = gap / 3; // Typically shorter gap between repeats
    // Send repeat frames if needed
    for (int i = 0; i < repeat - 1; i++) {
        // Wait a shorter time between frames
        GPIO_CLR = 1 << gpio_pin;
        sleep_us(repeat_gap);
        // Some protocols use special repeat codes instead of retransmitting the full code
        if (preamble_length > 0) {
            send_ir_signal(gpio_pin, frequency, preamble_code, preamble_length, header_pulse, header_space,
                          one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
        }
        send_ir_signal(gpio_pin, frequency, code, code_length, header_pulse, header_space,
                      one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
    }
}

// Count up from a specific hex code
void count_up_from_hex_starting(const char* starting_code, uint64_t preamble_code, int preamble_length,
                               int code_length, int header_pulse, int header_space,
                               int one_pulse, int one_space, int zero_pulse, int zero_space,
                               int ptrail, int gap, char view_mode, int gpio_pin, int frequency, int repeat) {
    uint64_t code;
    uint64_t max_code = 1ULL << code_length;
    // Convert the starting code from hex to integer
    if (strncmp(starting_code, "0x", 2) == 0) {
        sscanf(starting_code + 2, "%lx", (unsigned long*)&code);
    } else {
        sscanf(starting_code, "%lx", (unsigned long*)&code);
    }
    // Use a bit array to track sent codes
    size_t bit_array_size = (max_code + 7) / 8; // Number of bytes needed
    uint8_t* tried_codes = (uint8_t*)calloc(bit_array_size, 1);
    if (tried_codes == NULL) {
        printf("%sError: Not enough memory to track tried codes%s\n", RED, WHITE);
        return;
    }
    size_t tried_count = 0;
    // Set up signal handling for clean exit
    signal(SIGINT, signal_handler);
    while (keep_running && tried_count < max_code) {
        // Check if this code has been tried before
        size_t byte_index = code / 8;
        uint8_t bit_mask = 1 << (code % 8);
        if ((tried_codes[byte_index] & bit_mask) == 0) {
            // Mark this code as tried
            tried_codes[byte_index] |= bit_mask;
            tried_count++;
            // Send the code
            for (int i = 0; i < repeat; i++) {
                if (preamble_length > 0) {
                    send_ir_signal(gpio_pin, frequency, preamble_code, preamble_length, header_pulse, header_space,
                                 one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
                }
                send_ir_signal(gpio_pin, frequency, code, code_length, header_pulse, header_space,
                             one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
            }
        }
        // Move to the next code
        code++;
        if (code >= max_code) {
            break;
        }
        // Sleep between codes
        usleep(50000); // 50ms
    }
    free(tried_codes);
    if (!keep_running) {
        printf("\nExiting the program.\n");
    }
}

// Count up from zero
void count_up_from_zero(uint64_t preamble_code, int preamble_length, int code_length,
                       int header_pulse, int header_space, int one_pulse, int one_space,
                       int zero_pulse, int zero_space, int ptrail, int gap, char view_mode,
                       int gpio_pin, int frequency, int repeat) {
    uint64_t code = 0;
    uint64_t max_code = 1ULL << code_length;
    // Use a bit array to track sent codes
    size_t bit_array_size = (max_code + 7) / 8; // Number of bytes needed
    uint8_t* tried_codes = (uint8_t*)calloc(bit_array_size, 1);
    if (tried_codes == NULL) {
        printf("%sError: Not enough memory to track tried codes%s\n", RED, WHITE);
        return;
    }
    size_t tried_count = 0;
    // Set up signal handling for clean exit
    signal(SIGINT, signal_handler);
    while (keep_running && tried_count < max_code) {
        // Check if this code has been tried before
        size_t byte_index = code / 8;
        uint8_t bit_mask = 1 << (code % 8);
        if ((tried_codes[byte_index] & bit_mask) == 0) {
            // Mark this code as tried
            tried_codes[byte_index] |= bit_mask;
            tried_count++;
            // Send the code
            for (int i = 0; i < repeat; i++) {
                if (preamble_length > 0) {
                    send_ir_signal(gpio_pin, frequency, preamble_code, preamble_length, header_pulse, header_space,
                                 one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
                }
                send_ir_signal(gpio_pin, frequency, code, code_length, header_pulse, header_space,
                             one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
            }
        }
        // Move to the next code
        code++;
        if (code >= max_code) {
            code = 0; // Wrap around for continuous operation
        }
        // Sleep between codes
        usleep(50000); // 50ms
    }
    free(tried_codes);
    if (!keep_running) {
        printf("\nExiting the program.\n");
    }
}

// Send random IR codes
void random_mode_ir(uint64_t preamble_code, int preamble_length, int code_length,
                int header_pulse, int header_space, int one_pulse, int one_space,
                int zero_pulse, int zero_space, int ptrail, int gap, char view_mode,
                int gpio_pin, int frequency, int repeat) {
    uint64_t max_code = 1ULL << code_length;
    // Use a bit array to track sent codes
    size_t bit_array_size = (max_code + 7) / 8; // Number of bytes needed
    uint8_t* tried_codes = (uint8_t*)calloc(bit_array_size, 1);
    if (tried_codes == NULL) {
        printf("%sError: Not enough memory to track tried codes%s\n", RED, WHITE);
        return;
    }
    size_t tried_count = 0;
    // Seed the random number generator
    srand(time(NULL));
    // Set up signal handling for clean exit
    signal(SIGINT, signal_handler);
    while (keep_running && tried_count < max_code) {
        // Generate a random code
        uint64_t code = ((uint64_t)rand() << 32 | rand()) % max_code;
        // Check if this code has been tried before
        size_t byte_index = code / 8;
        uint8_t bit_mask = 1 << (code % 8);
        if ((tried_codes[byte_index] & bit_mask) == 0) {
            // Mark this code as tried
            tried_codes[byte_index] |= bit_mask;
            tried_count++;
            // Send the code
            for (int i = 0; i < repeat; i++) {
                if (preamble_length > 0) {
                    send_ir_signal(gpio_pin, frequency, preamble_code, preamble_length, header_pulse, header_space,
                                 one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
                }
                send_ir_signal(gpio_pin, frequency, code, code_length, header_pulse, header_space,
                             one_pulse, one_space, zero_pulse, zero_space, ptrail, gap, view_mode);
            }
        }
        // Sleep between codes
        usleep(50000); // 50ms
    }
    free(tried_codes);
    if (!keep_running) {
        printf("\nExiting the program.\n");
    }
}

// Thread data for IR receiving
typedef struct {
    int gpio_pin;
    int* signals;
    size_t size;
    size_t capacity;
    volatile bool running;
} IRReceiverThreadData;

// IR receiver thread function
void* ir_receiver_thread(void* arg) {
    IRReceiverThreadData* data = (IRReceiverThreadData*)arg;
    int gpio_pin = data->gpio_pin;
    uint32_t last_tick = 0;
    bool last_level = false;
    bool in_code = false;

    // Configure GPIO for input
    INP_GPIO(gpio_pin);

    // Initialize with the current GPIO state
    last_level = GPIO_READ(gpio_pin) ? true : false;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    last_tick = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;

    while (data->running && keep_running) {
        // Read current GPIO state
        bool current_level = GPIO_READ(gpio_pin) ? true : false;

        // Check for level change
        if (current_level != last_level) {
            // Calculate current time in microseconds
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint32_t current_tick = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;

            uint32_t duration = current_tick - last_tick;

            // Filter out extremely long durations that likely indicate gaps between signals
            if (duration < 10000000) { // 10 second maximum to avoid the huge initial value
                // Store pulse duration with correct sign (positive for mark, negative for space)
                int pulse_length = last_level ? duration : -duration;

                // Add to signals array if we're already in a code or if this is a potential start of code
                if (in_code || (duration > 1500 && duration < 10000)) {
                    // If not in code yet and this is a potential header pulse/space, mark as in code
                    if (!in_code) {
                        in_code = true;
                    }

                    // Add to signals array
                    if (data->size >= data->capacity) {
                        data->capacity *= 2;
                        data->signals = (int*)realloc(data->signals, data->capacity * sizeof(int));
                        if (data->signals == NULL) {
                            fprintf(stderr, "Error: Memory allocation failed\n");
                            data->running = false;
                            return NULL;
                        }
                    }
                    data->signals[data->size++] = pulse_length;
                }
            }

            last_tick = current_tick;
            last_level = current_level;
        }

        // Check for end of code (long pause)
        if (in_code) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint32_t current_tick = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;

            if (!current_level && (current_tick - last_tick) > 50000) { // 50ms silence
                in_code = false;
            }
        }

        // Small sleep to prevent CPU hogging
        usleep(100); // 100 microseconds - balance between CPU usage and resolution
    }
    return NULL;
}

// IR receiver implementation for direct GPIO
void receive_ir_signal(int recv_gpio) {
    printf("%sReceiving IR signals on GPIO %d using direct GPIO access...%s\n", YELLOW, recv_gpio, WHITE);
    printf("%sPress Ctrl+C to stop capturing.%s\n", YELLOW, WHITE);
    // Configure GPIO pin for input
    INP_GPIO(recv_gpio);
    // Initialize signals storage
    int* signals = (int*)malloc(1024 * sizeof(int));
    if (signals == NULL) {
        printf("%sError: Memory allocation failed%s\n", RED, WHITE);
        return;
    }
    // Set up IR receiver thread
    IRReceiverThreadData thread_data = {
        .gpio_pin = recv_gpio,
        .signals = signals,
        .size = 0,
        .capacity = 1024,
        .running = true
    };
    pthread_t receiver_thread;
    if (pthread_create(&receiver_thread, NULL, ir_receiver_thread, &thread_data) != 0) {
        printf("%sError: Failed to create receiver thread%s\n", RED, WHITE);
        free(signals);
        return;
    }
    // Set up signal handler for clean exit
    signal(SIGINT, signal_handler);
    // Wait for IR signals
    while (keep_running) {
        sleep(1);
        if (thread_data.size > 0) {
            printf("%sReceived %zu pulses so far%s\n", GREEN, thread_data.size, WHITE);
        }
    }
    // Clean up receiver thread
    thread_data.running = false;
    pthread_join(receiver_thread, NULL);
    // Display received signals
    printf("\n%sIR signal received:%s\n", GREEN, WHITE);
    for (size_t i = 0; i < thread_data.size; i++) {
        printf("%d ", signals[i]);
        if ((i + 1) % 10 == 0) {
            printf("\n");
        }
    }
    printf("\n");
    // Analyze the signals
    if (thread_data.size > 0) {
        // Skip any initial very large values that might be garbage
        size_t start_idx = 0;
        while (start_idx < thread_data.size && abs(signals[start_idx]) > 100000) {
            start_idx++;
        }
        // If we have valid data
        if (start_idx < thread_data.size - 3) {
            // Extract header pulse and space (after skipping any garbage)
            int header_pulse = abs(signals[start_idx]);
            int header_space = abs(signals[start_idx + 1]);
            int ptrail = abs(signals[thread_data.size - 1]);
            // Process pulse-space pairs to find one and zero patterns
            typedef struct {
                int pulse;
                int space;
                int count;
            } PulseSpacePair;
            #define MAX_PAIRS 256
            PulseSpacePair pairs[MAX_PAIRS] = {0};
            int unique_pairs = 0;
            for (size_t i = start_idx + 2; i < thread_data.size - 1; i += 2) {
                int pulse = abs(signals[i]);
                int space = abs(signals[i + 1]);
                // Skip unusually large values that might be noise or gaps
                if (pulse > 10000 || space > 10000) continue;
                // Check if this pair exists in our collection
                bool found = false;
                for (int j = 0; j < unique_pairs; j++) {
                    // Allow small tolerance for timing variations
                    if (abs(pairs[j].pulse - pulse) < 100 && abs(pairs[j].space - space) < 100) {
                        pairs[j].count++;
                        found = true;
                        break;
                    }
                }
                // Add new pair if not found
                if (!found && unique_pairs < MAX_PAIRS) {
                    pairs[unique_pairs].pulse = pulse;
                    pairs[unique_pairs].space = space;
                    pairs[unique_pairs].count = 1;
                    unique_pairs++;
                }
            }
            // Sort pairs by frequency (most common first)
            for (int i = 0; i < unique_pairs; i++) {
                for (int j = i + 1; j < unique_pairs; j++) {
                    if (pairs[j].count > pairs[i].count) {
                        PulseSpacePair temp = pairs[i];
                        pairs[i] = pairs[j];
                        pairs[j] = temp;
                    }
                }
            }
            // Set one and zero pulse/space values from the most common pairs
            int one_pulse = 0, one_space = 0, zero_pulse = 0, zero_space = 0;
            if (unique_pairs >= 2) {
                // Typically, ones have longer spaces than zeros in IR protocols
                if (pairs[0].space > pairs[1].space) {
                    one_pulse = pairs[0].pulse;
                    one_space = pairs[0].space;
                    zero_pulse = pairs[1].pulse;
                    zero_space = pairs[1].space;
                } else {
                    one_pulse = pairs[1].pulse;
                    one_space = pairs[1].space;
                    zero_pulse = pairs[0].pulse;
                    zero_space = pairs[0].space;
                }
            } else if (unique_pairs == 1) {
                // Only one pair found - assume it's the most common bit type
                zero_pulse = pairs[0].pulse;
                zero_space = pairs[0].space;
                one_pulse = zero_pulse;  // Assume similar pulse width
                one_space = zero_space * 2;  // Assume longer space for ones
            } else {
                printf("%sError: Unable to determine pulse/space pairs.%s\n", RED, WHITE);
                free(signals);
                return;
            }
            // Set gap value
            int gap = 0;
            for (int i = thread_data.size - 1; i >= 0; i--) {
                // Look for a significant gap in the signal
                if (abs(signals[i]) > 2000 && abs(signals[i]) < 100000) {
                    gap = abs(signals[i]);
                    break;
                }
            }
            // Calculate bit count (excluding header and potential trailing bits)
            int bit_count = ((thread_data.size - start_idx) - 3) / 2;
            // Display results
            printf("%sExtracted IR Signal Parameters:%s\n", GREEN, WHITE);
            printf("header_pulse: %d microseconds\n", header_pulse);
            printf("header_space: %d microseconds\n", header_space);
            printf("one_pulse: %d microseconds\n", one_pulse);
            printf("one_space: %d microseconds\n", one_space);
            printf("zero_pulse: %d microseconds\n", zero_pulse);
            printf("zero_space: %d microseconds\n", zero_space);
            printf("ptrail: %d microseconds\n", ptrail);
            printf("gap: %d microseconds\n", gap);
            printf("bit_count: %d\n", bit_count);
        } else {
            printf("%sError: Not enough valid signal data to analyze.%s\n", RED, WHITE);
        }
    }
    // Clean up memory
    free(signals);
}

// Motd banner
void print_motd() {
    printf("%s\n"
           "  _____        __                  _____                     _\n"
           " |_   _|      / _|                |  __ \\                   | |\n"
           "   | |  _ __ | |_ _ __ __ _ ______| |  | |_ __ ___  __ _  __| |\n"
           "   | | | '_ \\|  _| '__/ _` |______| |  | | '__/ _ \\/ _` |/ _` |\n"
           "  _| |_| | | | | | | | (_| |      | |__| | | |  __/ (_| | (_| |\n"
           " |_____|_| |_|_| |_|  \\__,_|      |_____/|_|  \\___|\\__,_|\\__,_|v2.0\n"
           "%s\n", RED, WHITE);
}

// Argp option keys
#define OPT_GPIO           'g'
#define OPT_RECV_GPIO      'R'
#define OPT_LENGTH         'l'
#define OPT_RANDOM         'r'
#define OPT_CODE           'm'
#define OPT_PREAMBLE       'p'
#define OPT_REPEAT         'x'
#define OPT_HEADER_PULSE   1000
#define OPT_HEADER_SPACE   1001
#define OPT_ONE_PULSE      1002
#define OPT_ONE_SPACE      1003
#define OPT_ZERO_PULSE     1004
#define OPT_ZERO_SPACE     1005
#define OPT_PTRAIL         1006
#define OPT_GAP            1007
#define OPT_FREQUENCY      'f'
#define OPT_DUTY           'd'
#define OPT_START_FROM     's'
#define OPT_VIEW           'v'
#define OPT_RECEIVE        'i'
#define OPT_TEST           't'

// Option parser for argp
static error_t parse_opt(int key, char* arg, struct argp_state* state) {
    struct arguments* arguments = state->input;
    switch (key) {
        case OPT_GPIO:
            arguments->gpio_pin = atoi(arg);
            break;
        case OPT_RECV_GPIO:
            arguments->recv_gpio = atoi(arg);
            break;
        case OPT_LENGTH:
            arguments->length = atoi(arg);
            break;
        case OPT_RANDOM:
            arguments->random_mode = true;
            break;
        case OPT_CODE:
            arguments->code = arg;
            break;
        case OPT_PREAMBLE:
            arguments->preamble = arg;
            break;
        case OPT_REPEAT:
            arguments->repeat = atoi(arg);
            break;
        case OPT_HEADER_PULSE:
            arguments->header_pulse = atoi(arg);
            break;
        case OPT_HEADER_SPACE:
            arguments->header_space = atoi(arg);
            break;
        case OPT_ONE_PULSE:
            arguments->one_pulse = atoi(arg);
            break;
        case OPT_ONE_SPACE:
            arguments->one_space = atoi(arg);
            break;
        case OPT_ZERO_PULSE:
            arguments->zero_pulse = atoi(arg);
            break;
        case OPT_ZERO_SPACE:
            arguments->zero_space = atoi(arg);
            break;
        case OPT_PTRAIL:
            arguments->ptrail = atoi(arg);
            break;
        case OPT_GAP:
            arguments->gap = atoi(arg);
            break;
        case OPT_FREQUENCY:
            arguments->frequency = atoi(arg);
            break;
        case OPT_DUTY:
            arguments->duty = atof(arg);
            break;
        case OPT_START_FROM:
            arguments->start_from = arg;
            break;
        case OPT_VIEW:
            if (arg[0] == 'b' || arg[0] == 'h') {
                arguments->view_mode = arg[0];
            } else {
                printf("%sInvalid view mode. Using default 'h'.%s\n", RED, WHITE);
            }
            break;
        case OPT_RECEIVE:
            arguments->receive = true;
            break;
        case OPT_TEST:
            arguments->test = true;
            break;
        default:
            return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

// Command-line options definition
static struct argp_option options[] = {
    {"gpio",         OPT_GPIO,        "PIN",  0, "GPIO pin number for sending IR (default: 18)", 0},
    {"recv_gpio",    OPT_RECV_GPIO,   "PIN",  0, "GPIO pin number for receiving IR", 0},
    {"length",       OPT_LENGTH,      "BITS", 0, "Number of bits for the IR codes (default: 32)", 0},
    {"random",       OPT_RANDOM,      0,      0, "Enable random mode (default is counting-up)", 0},
    {"code",         OPT_CODE,        "HEX",  0, "IR code to send in hex format (e.g., 0x02A1)", 0},
    {"preamble",     OPT_PREAMBLE,    "HEX",  0, "Fixed preamble IR code to send in hex format", 0},
    {"repeat",       OPT_REPEAT,      "NUM",  0, "Number of times to repeat sending the code (default: 1)", 0},
    {"header_pulse", OPT_HEADER_PULSE,"US",   0, "Header pulse duration (microseconds, default: 4058)", 0},
    {"header_space", OPT_HEADER_SPACE,"US",   0, "Header space duration (microseconds, default: 3964)", 0},
    {"one_pulse",    OPT_ONE_PULSE,   "US",   0, "One pulse duration (microseconds, default: 0)", 0},
    {"one_space",    OPT_ONE_SPACE,   "US",   0, "One space duration (microseconds, default: 0)", 0},
    {"zero_pulse",   OPT_ZERO_PULSE,  "US",   0, "Zero pulse duration (microseconds, default: 514)", 0},
    {"zero_space",   OPT_ZERO_SPACE,  "US",   0, "Zero space duration (microseconds, default: 981)", 0},
    {"ptrail",       OPT_PTRAIL,      "US",   0, "Pulse trail duration (microseconds, default: 514)", 0},
    {"gap",          OPT_GAP,         "US",   0, "Gap duration (microseconds, default: 64729)", 0},
    {"frequency",    OPT_FREQUENCY,   "HZ",   0, "Carrier frequency (Hz, default: 38000)", 0},
    {"duty",         OPT_DUTY,        "PCT",  0, "Duty cycle for the PWM signal (default: 50.0)", 0},
    {"start_from",   OPT_START_FROM,  "HEX",  0, "Start counting up from the specified hex code", 0},
    {"view",         OPT_VIEW,        "MODE", 0, "Output view mode: 'b' for binary, 'h' for hex (default: 'h')", 0},
    {"receive",      OPT_RECEIVE,     0,      0, "Enable receiving mode to capture IR signals", 0},
    {"test",         OPT_TEST,        0,      0, "Run test mode", 0},
    {0}
};

// Argp parser configuration
static struct argp argp = {options, parse_opt, 0, "Send and receive IR codes (direct GPIO version).", 0, 0, 0};

int main(int argc, char* argv[]) {
    // Print the banner
    print_motd();

    // Parse command line arguments
    argp_parse(&argp, argc, argv, 0, 0, &arguments);

    // Initialize GPIO
    if (setup_gpio() < 0) {
        return 1;
    }

    // If receive mode is enabled
    if (arguments.receive) {
        if (arguments.recv_gpio <= 0) {
            printf("%sError: You must specify a GPIO pin for receiving IR with --recv_gpio.%s\n", RED, WHITE);
            cleanup_gpio();
            return 1;
        }
        receive_ir_signal(arguments.recv_gpio);
        cleanup_gpio();
        return 0;
    }

    // If test mode is enabled
    if (arguments.test) {
        if (arguments.recv_gpio <= 0) {
            printf("%sError: You must specify a GPIO pin for receiving IR with --recv_gpio.%s\n", RED, WHITE);
            cleanup_gpio();
            return 1;
        }
        printf("%sRunning IR test mode with transmitter on GPIO %d and receiver on GPIO %d...%s\n",
               YELLOW, arguments.gpio_pin, arguments.recv_gpio, WHITE);

        // Use some standard test parameters
        int one_pulse = 550;
        int one_space = 1620;
        int zero_pulse = 650;
        int zero_space = 600;
        int ptrail = 666;
        int gap = 10000;
        uint64_t code = 0x1234;  // Test code
        int code_length = 16;

        // Send a test signal
        printf("%sSending test code 0x1234...%s\n", GREEN, WHITE);
        send_ir_signal(arguments.gpio_pin, arguments.frequency, code, code_length,
                     arguments.header_pulse, arguments.header_space,
                     one_pulse, one_space, zero_pulse, zero_space,
                     ptrail, gap, 'h');

        // Then try to receive
        printf("%sNow trying to receive IR signals...%s\n", GREEN, WHITE);
        receive_ir_signal(arguments.recv_gpio);

        cleanup_gpio();
        return 0;
    }

    // Set GPIO pin as output
    INP_GPIO(arguments.gpio_pin);
    OUT_GPIO(arguments.gpio_pin);

    // Set default values for one_pulse and one_space if they are 0
    if (arguments.one_pulse == 0) {
        arguments.one_pulse = 514;
    }
    if (arguments.one_space == 0) {
        arguments.one_space = 1980;
    }

    // Parse preamble code if provided
    uint64_t preamble_code = 0;
    int preamble_length = 0;
    if (arguments.preamble != NULL) {
        if (strncmp(arguments.preamble, "0x", 2) == 0) {
            sscanf(arguments.preamble + 2, "%lx", (unsigned long*)&preamble_code);
        } else {
            sscanf(arguments.preamble, "%lx", (unsigned long*)&preamble_code);
        }

        // Calculate bit length
        uint64_t temp = preamble_code;
        while (temp > 0) {
            preamble_length++;
            temp >>= 1;
        }
    }

    // Choose operating mode based on arguments
    if (arguments.code != NULL) {
        // Send a fixed code
        uint64_t code;
        if (strncmp(arguments.code, "0x", 2) == 0) {
            sscanf(arguments.code + 2, "%lx", (unsigned long*)&code);
        } else {
            sscanf(arguments.code, "%lx", (unsigned long*)&code);
        }

        send_fixed_code(preamble_code, preamble_length, code, arguments.length,
                       arguments.header_pulse, arguments.header_space,
                       arguments.one_pulse, arguments.one_space,
                       arguments.zero_pulse, arguments.zero_space,
                       arguments.ptrail, arguments.gap, arguments.view_mode,
                       arguments.gpio_pin, arguments.frequency, arguments.repeat);
    } else if (arguments.random_mode) {
        // Random mode
        random_mode_ir(preamble_code, preamble_length, arguments.length,
                   arguments.header_pulse, arguments.header_space,
                   arguments.one_pulse, arguments.one_space,
                   arguments.zero_pulse, arguments.zero_space,
                   arguments.ptrail, arguments.gap, arguments.view_mode,
                   arguments.gpio_pin, arguments.frequency, arguments.repeat);
    } else if (arguments.start_from != NULL) {
        // Count up from a specified hex code
        count_up_from_hex_starting(arguments.start_from, preamble_code, preamble_length,
                                  arguments.length, arguments.header_pulse, arguments.header_space,
                                  arguments.one_pulse, arguments.one_space,
                                  arguments.zero_pulse, arguments.zero_space,
                                  arguments.ptrail, arguments.gap, arguments.view_mode,
                                  arguments.gpio_pin, arguments.frequency, arguments.repeat);
    } else {
        // Default: count up from zero
        count_up_from_zero(preamble_code, preamble_length, arguments.length,
                          arguments.header_pulse, arguments.header_space,
                          arguments.one_pulse, arguments.one_space,
                          arguments.zero_pulse, arguments.zero_space,
                          arguments.ptrail, arguments.gap, arguments.view_mode,
                          arguments.gpio_pin, arguments.frequency, arguments.repeat);
    }

    // Clean up GPIO
    cleanup_gpio();
    return 0;
}
