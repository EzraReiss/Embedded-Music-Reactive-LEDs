// Music Reactive LED Controller for FRDM-KL46Z
// Uses microphone to capture audio and drives RGB LEDs based on music
// Original code by Nils Napp (May 2023), based on example by Evan Greavu
// Hardware: FRDM-KL46Z with microphone on PORTE20 and RGB LEDs on PORTB1-3

/* Include required header files */
#include <pin_mux.h>      /* Pin multiplexer configuration */
#include <clock_config.h> /* Clock configuration */
#include <stdio.h>        /* Standard I/O functions */
#include <board.h>        /* Board-specific definitions */
#include <MKL46Z4.h>      /* Microcontroller-specific registers */
#include <fsl_debug_console.h> /* defines bool */
#include <time.h>         /* Time-related functions */
#include <math.h>         /* Math functions (sqrt, floor, etc.) */
#include <stdlib.h>       /* Standard library (rand, etc.) */

/* System clock configuration */
#ifndef SystemCoreClock
#define SystemCoreClock 48000000U  /* 48MHz system clock frequency */
#endif

// Configuration Constants
/* Audio sampling configuration */
#define BITS_IN_SAMPLES 3          /* Power of 2 for sample buffer size */
#define SAMPLE_SIZE 8              /* 2^BITS_IN_SAMPLES = 8 samples in circular buffer */
#define N_BIT_SAMPLE_AVERAGE 2     /* Power of 2 for averaging samples */
#define N_SAMPLE_AVERAGE (1 << N_BIT_SAMPLE_AVERAGE)  /* Number of samples to average (4) */

/* Audio processing thresholds */
#define SOUND_THRESHOLD 300        /* Minimum amplitude to trigger LED response */
#define MAX_AMPLITUDE 500          /* Maximum amplitude value to consider (clamp higher values) */
#define BRIGHTNESS_FACTOR 22.3f    /* Scaling factor for brightness calculation */

/* Color animation parameters */
#define COLOR_TRANSITION_TIME 200  /* Duration for color transitions in milliseconds */
#define COLOR_CHANGE_DELAY_MS 250  /* Minimum time between color changes */
#define COLOR_TRANSITION_STEPS 100 /* Number of steps for smooth color transitions */

/* Boolean constants */
#define false 0                    /* Boolean false value */
#define true 1                     /* Boolean true value */

/* System timing */
#undef CLOCKS_PER_SEC              /* Undefine potentially existing value */
#define CLOCKS_PER_SEC 1000        /* Define system tick rate (1ms per tick) */

/* Macros for PWM control - these avoid function call overhead */
#define set_red(v)   (TPM1->CONTROLS[1].CnV = (v))    /* Red LED on TPM1_CH1 */
#define set_green(v) (TPM2->CONTROLS[0].CnV = (v))    /* Green LED on TPM2_CH0 */
#define set_blue(v)  (TPM2->CONTROLS[1].CnV = (v))    /* Blue LED on TPM2_CH1 */

/* Math helper functions */
#define min(a,b) (((a) < (b)) ? (a) : (b))  /* Minimum of two values */

// Type definitions
/* Forward declarations for struct types */
typedef struct HSV HSV;
typedef struct RGB RGB;

/* RGB color structure (0-255 values) */
struct RGB {
    int r;  /* Red component */
    int g;  /* Green component */
    int b;  /* Blue component */
};

/* HSV color structure (more intuitive for color manipulation) */
struct HSV {
    float h;  /* Hue (0-360 degrees) */
    float s;  /* Saturation (0-1) */
    float v;  /* Value/Brightness (0-1) */
};

// Global variables
/* Audio sampling buffer */
uint16_t g_mic_samples[SAMPLE_SIZE];  /* Circular buffer to hold amplitude samples */
uint8_t g_sample_index = 0;           /* Current position in the circular buffer */

/* LED pin configuration for FRDM-KL46Z */
const int g_red_pin = 1;    /* PTB1 for red channel */
const int g_green_pin = 2;  /* PTB2 for green channel */
const int g_blue_pin = 3;   /* PTB3 for blue channel */

/* Global millisecond counter, updated in SysTick_Handler */
volatile uint32_t g_ms_ticks = 0;  /* Definition of the previously declared extern variable */

/* Color state variables */
clock_t g_color_change_delay = 0;  /* Timestamp of last color change */
uint8_t g_red;                     /* Current red component (0-255) */
uint8_t g_green;                   /* Current green component (0-255) */
uint8_t g_blue;                    /* Current blue component (0-255) */
HSV g_hsv = {0, 1, 1};             /* Current HSV color (initialized to red with full saturation and brightness) */

/* Music detection state */
bool g_is_music = false;                /* Flag indicating active music */
clock_t g_last_music_check = 0;         /* Timestamp of last music check */
clock_t g_last_music_detection = 0;     /* Timestamp of last detected music */
clock_t g_last_brightness_update = 0;   /* Timestamp of last brightness update */

// Function prototypes
/* Hardware initialization */
void setup_adc(void);               /* Initialize and calibrate ADC */
void init_pwm(void);                /* Initialize PWM for LED control */
void init_systick(void);            /* Initialize system tick timer */
void init_pit_us(void);             /* Initialize PIT for microsecond timing */
void initialize_hardware(void);     /* Master initialization function */

/* Audio processing */
uint16_t sample_microphone(void);   /* Sample the microphone and calculate amplitude */
uint16_t get_mean(void);            /* Calculate average amplitude from sample buffer */

/* LED control */
void set_duty_cycle(int color_pin, uint8_t duty_cycle);  /* Set PWM duty cycle for specified pin */
void set_rgb_color(int r, int g, int b);                /* Set RGB LED color */

/* Color processing and effects */
RGB hsv_to_rgb(HSV hsv);                                /* Convert HSV to RGB color space */
void transition_color(RGB from_color, RGB to_color, int transition_time);  /* Smooth transition between colors */
void process_audio_sample(uint16_t amplitude, uint16_t mean);  /* Process audio and update LEDs */
void handle_color_change(uint16_t amplitude, uint16_t mean);   /* Handle beat detection and color changes */
void handle_brightness_change(uint16_t amplitude, uint16_t mean);  /* Handle brightness modulation */

/* Utility functions */
void delay_ms(uint32_t ms);                 /* Millisecond delay function */
int random_range(int min, int max);         /* Generate random number in range [min, max] */
int map_value(int x, int in_min, int in_max, int out_min, int out_max);  /* Map value from one range to another */
uint32_t get_microseconds(void);            /* Get microsecond timer value */

// System time management
/* Global millisecond counter, updated in SysTick_Handler */
// extern volatile uint32_t g_ms_ticks;

// Returns the current system time in milliseconds
clock_t clock(void)
{
    return g_ms_ticks;  /* Return the current millisecond tick count */
}

// Audio processing functions
// Samples the microphone and calculates audio amplitude
uint16_t sample_microphone(void) {
    unsigned int mic_val = 0;        /* Variable to hold raw ADC value */
    int min_val = 1024;              /* Initialize to max possible ADC value */
    int max_val = 0;                 /* Initialize to min possible ADC value */
    uint16_t sample_accum = 0;       /* Accumulator for sample averaging */
    uint8_t sample_accum_index = 0;  /* Counter for number of samples accumulated */
    
    /* Collect and process multiple audio samples to find amplitude */
    for (int i = 0; i < (256 << N_BIT_SAMPLE_AVERAGE); i++) {
        /* Trigger ADC sample on channel 8 (connected to microphone) */
        ADC0->SC1[0] = ADC_SC1_ADCH(8);
        
        /* Wait for conversion to complete by checking COCO flag */
        while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
        
        /* Read ADC result from the result register */
        mic_val = ADC0->R[0];
        
        /* Accumulate samples for averaging */
        sample_accum += mic_val;       /* Add sample to accumulator */
        sample_accum_index += 1;       /* Increment sample counter */
        
        /* Process after accumulating N_SAMPLE_AVERAGE samples */
        if (sample_accum_index == N_SAMPLE_AVERAGE) {
            sample_accum_index = 0;    /* Reset the counter */
            
            /* Calculate average by dividing by number of samples (using bit shift) */
            mic_val = sample_accum >> N_BIT_SAMPLE_AVERAGE;
            
            /* Track min and max values to calculate amplitude */
            max_val = (mic_val > max_val) ? mic_val : max_val;  /* Update max if needed */
            min_val = (mic_val < min_val) ? mic_val : min_val;  /* Update min if needed */
            
            /* Reset the accumulator for next batch */
            sample_accum = 0;
        }
    }
    
    /* Calculate amplitude as difference between max and min values */
    uint16_t amplitude = max_val - min_val;
    
    /* Store in circular buffer and update index */
    g_mic_samples[g_sample_index] = amplitude;  /* Store amplitude at current position */
    
    /* Update index with wrap-around at end of buffer */
    g_sample_index = (g_sample_index == SAMPLE_SIZE - 1) ? 0 : g_sample_index + 1;
    
    return amplitude;  /* Return the calculated amplitude */
}

// Processes audio sample and controls LED effects
void process_audio_sample(uint16_t amplitude, uint16_t mean) {
    /* Get current time for timing-related decisions */
    clock_t current_time = clock();
    
    /* 
     * Condition for beat detection and color change:
     * 1. Current amplitude exceeds mean by sufficient margin (beat detected)
     * 2. Enough time has passed since last color change
     * 3. Average sound level is above threshold (actual music playing)
     */
    if ((amplitude > mean + mean / 5) &&                         /* Beat detection threshold */
        (current_time - g_color_change_delay > COLOR_CHANGE_DELAY_MS) && /* Timing condition */
        (mean > SOUND_THRESHOLD)) {                              /* Sound level threshold */
        
        /* Handle color change on beat detection */
        handle_color_change(amplitude, mean);
    }
    /* Condition for brightness modulation (sound detected but not a beat) */
    else if (mean > SOUND_THRESHOLD) {
        /* Modulate brightness based on current amplitude */
        handle_brightness_change(amplitude, mean);
    }
    /* No sound detected - turn off LEDs */
    else {
        set_rgb_color(0, 0, 0);  /* Set all channels to zero (LEDs off) */
    }
}

// Changes LED color in response to beat detection
void handle_color_change(uint16_t amplitude, uint16_t mean) {
    /* Generate a significant hue change (40-190 degrees) for visual impact */
    int hue_change = random_range(40, 190);
    
    /* Store current color for transition effect */
    RGB old_color = hsv_to_rgb(g_hsv);
    
    /* Compute new color by rotating hue (keeping within 0-360 degrees) */
    g_hsv.h = fmod((g_hsv.h + hue_change), 360);
    
    /* Convert new HSV color to RGB for the LEDs */
    RGB rgb = hsv_to_rgb(g_hsv);
    
    /* Clear LEDs before transition (visual reset) */
    set_rgb_color(0, 0, 0);
    
    /* For moderate beats, apply smooth transition */
    if (amplitude < mean + mean / 3.4) {
        /* Smooth transition between old and new color */
        transition_color(old_color, rgb, COLOR_TRANSITION_TIME);
    } 
    /* For stronger beats, apply faster transition with delay */
    else {
        /* Simple delay for more intense beats - creates "flash" effect */
        delay_ms(100);
    }
    
    /* Store new color values for brightness modulation */
    g_red = rgb.r;    /* Store red component */
    g_green = rgb.g;  /* Store green component */
    g_blue = rgb.b;   /* Store blue component */
    
    /* Update timestamp of last color change to enforce minimum delay between changes */
    g_color_change_delay = clock();
}

/**
 * @brief Modulate LED brightness based on audio amplitude
 * 
 * Scales the RGB color brightness based on current audio level,
 * creating a pulsing effect that follows the music.
 * 
 * @param amplitude Current audio amplitude
 * @param mean Average amplitude over recent samples
 */
void handle_brightness_change(uint16_t amplitude, uint16_t mean) {
    /* 
     * Calculate brightness factor (0-255) from audio amplitude:
     * 1. Limit amplitude to MAX_AMPLITUDE
     * 2. Subtract half the mean to create a more dynamic range
     * 3. Apply square root for more natural-feeling brightness response
     * 4. Scale by BRIGHTNESS_FACTOR
     * 5. Map to 0-255 range for PWM duty cycle
     */
    float brightness_factor = (sqrt((min(MAX_AMPLITUDE, amplitude) - mean / 2)) / BRIGHTNESS_FACTOR) * 255;
    
    /* Scale each color component by brightness factor */
    uint8_t r = floor((g_red * brightness_factor) / 255);    /* Scale red component */
    uint8_t g = floor((g_green * brightness_factor) / 255);  /* Scale green component */
    uint8_t b = floor((g_blue * brightness_factor) / 255);   /* Scale blue component */
    
    /* Apply scaled color to LEDs */
    set_rgb_color(r, g, b);
    
    /* Update timestamp of last brightness update for timing tracking */
    g_last_brightness_update = clock();
}

/**
 * @brief Calculate mean amplitude from sample buffer
 * 
 * Averages the values in the amplitude buffer to get a baseline
 * for beat detection and other audio processing.
 * 
 * @return Mean amplitude value
 */
uint16_t get_mean(void)
{
    uint16_t mean = 0;  /* Initialize sum to zero */
    
    /* Sum all values in sample buffer */
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        mean += g_mic_samples[i];  /* Add each sample to the sum */
    }
    
    /* Divide by number of samples (using bit shift for efficiency) */
    mean = mean >> BITS_IN_SAMPLES;  /* Equivalent to mean / SAMPLE_SIZE */
    
    return mean;  /* Return the calculated mean */
}

/*******************************************************************************
 * Hardware initialization functions
 ******************************************************************************/

// Master initialization function that sets up all required hardware.
void initialize_hardware(void) {
    BOARD_InitBootPins();      /* Initialize board pins from pin_mux.h */
    BOARD_InitBootClocks();    /* Initialize system clocks */
    BOARD_InitDebugConsole();  /* Initialize debug console (if needed) */
    setup_adc();               /* Initialize the ADC for microphone input */
    init_systick();            /* Initialize system tick timer for timing */
    init_pwm();                /* Initialize PWM for LED control */
}

//configures ADC0 with appropriate settings for audio sampling,
//performs hardware calibration, and connects to the microphone pin.
void setup_adc(void)
{
    int cal_v;  /* Calibration value variable */

    /* Enable clock gate for ADC0 in the System Integration Module */
    SIM->SCGC6 |= (1 << 27);  /* Bit 27 is ADC0 clock gate control */
    
    /* Configure ADC for:
     * - 10-bit mode (MODE=0b10)
     * - Bus clock source (ADICLK=0b00)
     * - No clock division (ADIV=0b00) */
    ADC0->CFG1 = ADC_CFG1_ADICLK(0b00) | ADC_CFG1_MODE(0b10) | ADC_CFG1_ADIV(0b00);

    /* Calibrate ADC for accuracy */
    ADC0->SC3 = 0;                     /* Clear SC3 register */
    ADC0->SC3 |= ADC_SC3_AVGS(0b10);   /* 32-sample hardware averaging during calibration */
    ADC0->SC3 |= ADC_SC3_CAL_MASK;     /* Start calibration process */
    
    /* Wait for calibration to complete by checking COCO flag */
    while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));

    /* Calculate and write calibration values to plus-side registers */
    cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
    cal_v = cal_v >> 1 | 0x8000;  /* Divide by 2 and set 15th bit */
    ADC0->PG = cal_v;             /* Set plus-side gain calibration */

    /* Calculate and write calibration values to minus-side registers */
    cal_v = 0;                    /* Reset calibration value */
    cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
    cal_v = cal_v >> 1 | 0x8000;  /* Divide by 2 and set 15th bit */
    ADC0->MG = cal_v;             /* Set minus-side gain calibration */

    /* Disable hardware averaging for normal conversions to get raw samples */
    ADC0->SC3 = 0;

    /* Configure PTE20 as analog input for microphone:
     * 1. Enable PORTE clock gate
     * 2. Set pin mux to 0 (analog mode) */
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;  /* Enable PORTE clock */
    PORTE->PCR[20] = PORT_PCR_MUX(0);     /* Set pin 20 to analog mode */
}

//initializes PWM for LED control
void init_pwm(void)
{
    /* Enable clock for PORTB (LEDs are on PTB1-3) */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    
    /* Enable clock for TPM1 and TPM2 modules */
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;

    /* Select FLL clock source for TPM modules (TPMSRC=1) */
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    /* Configure pins for TPM alternate function (ALT3) */
    PORTB->PCR[1] = PORT_PCR_MUX(3);   /* PTB1 → TPM1_CH1 (Red) */
    PORTB->PCR[2] = PORT_PCR_MUX(3);   /* PTB2 → TPM2_CH0 (Green) */
    PORTB->PCR[3] = PORT_PCR_MUX(3);   /* PTB3 → TPM2_CH1 (Blue) */

    /* Set PWM period to 255 (8-bit resolution) for both timers */
    TPM1->MOD = TPM2->MOD = 255;

    /* Configure for edge-aligned PWM, high-true pulses:
     * - MSB: Edge-aligned PWM
     * - ELSB: High-true pulses */
    TPM1->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;  /* Red channel */
    TPM2->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;  /* Green channel */
    TPM2->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;  /* Blue channel */

    /* Start timers with prescaler of 1:
     * - CMOD=1: Enable clock */
    TPM1->SC = TPM_SC_CMOD(1);  /* Start TPM1 */
    TPM2->SC = TPM_SC_CMOD(1);  /* Start TPM2 */
}

//sets the PWM duty cycle for a specific color pin
void set_duty_cycle(int color_pin, uint8_t duty_cycle) {
    /* Set channel value register to the duty cycle value */
    TPM1->CONTROLS[color_pin].CnV = duty_cycle;
}

// sets the RGB LED color
void set_rgb_color(int r, int g, int b) {
    set_red(r);     /* Set red component */
    set_green(g);   /* Set green component */
    set_blue(b);    /* Set blue component */
}

/*******************************************************************************
 * Timing and utility functions
 ******************************************************************************/
// initializes the systick timer for millisecond timing
void init_systick(void) {
    SystemCoreClockUpdate();                      /* Update core clock frequency */
    
    /* Set reload value for 1ms tick interval:
     * (SystemCoreClock / 1000) - 1 ticks = 1ms */
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    
    SysTick->VAL = 0;  /* Clear current value */
    
    /* Configure SysTick control register:
     * - CLKSOURCE=1: Use processor clock
     * - TICKINT=1: Enable interrupt
     * - ENABLE=1: Enable counter */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk |
                   SysTick_CTRL_ENABLE_Msk;
}

// SysTick interrupt handler
void SysTick_Handler(void) {
    g_ms_ticks++;  /* Increment the millisecond counter */
}

// Delay this many milliseconds
void delay_ms(uint32_t ms) {
    uint32_t start = g_ms_ticks;  /* Capture starting time */
    while ((g_ms_ticks - start) < ms) { }  /* Wait until enough time has passed */
}

// generates a random number in the specified range
int random_range(int min, int max) {
    /* Generate random number and scale to the desired range */
    return min + rand() % (max - min + 1);
}

// initializes the PIT for microsecond timing
void init_pit_us(void) {
    /* Enable the PIT clock in the System Integration Module */
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    
    /* Enable PIT module (0 = enable timers in debug mode) */
    PIT->MCR = 0;
    
    /* Configure channel 0 for free-running mode with 1µs tick:
     * Reload value = (SystemCoreClock / 1000000) - 1 for a 1µs period */
    PIT->CHANNEL[0].LDVAL = SystemCoreClock/1000000 - 1;
    
    /* Enable the timer (TEN=1) without enabling interrupts */
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN_MASK;
}

// gets the current microsecond counter value
uint32_t get_microseconds(void) {
    /* PIT timer counts down, so calculate elapsed time by subtracting current value:
     * elapsed = LoadValue - CurrentValue */
    uint32_t elapsed = (SystemCoreClock/1000000) - PIT->CHANNEL[0].CVAL;
    return elapsed; 
}

/*******************************************************************************
 * Color conversion and effects
 ******************************************************************************/
// converts HSV to RGB
RGB hsv_to_rgb(HSV hsv)
{
    float r, g, b;  /* RGB component values (0.0-1.0) */
    int i;          /* Hue sector (0-5) */
    float f, p, q, t;  /* Intermediate values for calculation */

    /* Special case: if saturation is 0, color is grayscale */
    if (hsv.s == 0)
    {
        r = g = b = hsv.v;  /* All components equal to value */
        /* Convert to 0-255 range and return */
        return (RGB){(int)(r * 255), (int)(g * 255), (int)(b * 255)};
    }

    /* Convert hue to 0-6 range for algorithm */
    hsv.h /= 60;         /* Now in range 0-6 */
    i = floor(hsv.h);    /* Integer part (sector 0-5) */
    f = hsv.h - i;       /* Fractional part (position within sector) */
    
    /* Calculate intermediate values for color conversion */
    p = hsv.v * (1 - hsv.s);                 /* Value scaled by saturation */
    q = hsv.v * (1 - hsv.s * f);             /* Value scaled by saturation and fractional part */
    t = hsv.v * (1 - hsv.s * (1 - f));       /* Value scaled by saturation and complement of fractional part */

    /* Map HSV to RGB based on hue sector (0-5) */
    switch (i)
    {
    case 0:  /* 0-60 degrees: red to yellow */
        r = hsv.v;  /* Red at maximum */
        g = t;      /* Green increasing */
        b = p;      /* Blue at minimum */
        break;
    case 1:  /* 60-120 degrees: yellow to green */
        r = q;      /* Red decreasing */
        g = hsv.v;  /* Green at maximum */
        b = p;      /* Blue at minimum */
        break;
    case 2:  /* 120-180 degrees: green to cyan */
        r = p;      /* Red at minimum */
        g = hsv.v;  /* Green at maximum */
        b = t;      /* Blue increasing */
        break;
    case 3:  /* 180-240 degrees: cyan to blue */
        r = p;      /* Red at minimum */
        g = q;      /* Green decreasing */
        b = hsv.v;  /* Blue at maximum */
        break;
    case 4:  /* 240-300 degrees: blue to magenta */
        r = t;      /* Red increasing */
        g = p;      /* Green at minimum */
        b = hsv.v;  /* Blue at maximum */
        break;
    default: /* 300-360 degrees: magenta to red */
        r = hsv.v;  /* Red at maximum */
        g = p;      /* Green at minimum */
        b = q;      /* Blue decreasing */
        break;
    }
    
    /* Convert from 0-1 to 0-255 range for LED PWM control */
    return (RGB){(int)(r * 255), (int)(g * 255), (int)(b * 255)};
}

// maps a value from one range to another
int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
    /* Linear mapping equation: y = (x-in_min)*(out_max-out_min)/(in_max-in_min) + out_min */
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// transitions smoothly between two colors
void transition_color(RGB from_color, RGB to_color, int transition_time)
{
    /* Number of steps for the transition */
    int steps = COLOR_TRANSITION_STEPS;
    
    /* Increment through steps, calculating intermediate colors */
    for (int i = 0; i <= steps; i++)
    {
        /* Map current step to intermediate RGB values */
        int r = map_value(i, 0, steps, from_color.r, to_color.r);  /* Red component */
        int g = map_value(i, 0, steps, from_color.g, to_color.g);  /* Green component */
        int b = map_value(i, 0, steps, from_color.b, to_color.b);  /* Blue component */

        /* Apply intermediate color to LEDs */
        set_rgb_color(r, g, b);

        /* Delay for smooth transition */
        delay_ms(transition_time / steps);  /* Equal time slice for each step */
    }
}

/*******************************************************************************
 * Main 
 ******************************************************************************/
int main(void)
{
    /* Initialize all hardware components */
    initialize_hardware();

    /* Main processing loop - runs indefinitely */
    while (1)
    {
        /* Sample the microphone and get amplitude */
        uint16_t amplitude = sample_microphone();
        
        /* Calculate average amplitude from history */
        uint16_t mean = get_mean();
        
        /* Process audio and update LEDs accordingly */
        process_audio_sample(amplitude, mean);
    }
    
    return 0;  /* Never reached (infinite loop) */
}


