/* ADC Example code for ECE3140
 *
 * Nils Napp May 2023
 *
 * Based on example by Evan Greavu
 */

#include <pin_mux.h>
#include <clock_config.h>
#include <stdio.h>
#include <board.h>
#include <MKL46Z4.h>
#include <fsl_debug_console.h>
#include <rolling_buffer.h>
#include <time.h>
#include <math.h>
#include <stdlib.h> // For random number generation

#define BITS_IN_SAMPLES 3
#define SAMPLE_SIZE 8 // 2^BITS_IN_SAMPLES = 8 samples
#define false 0
#define true 1
#define N_BIT_SAMPLE_AVERAGE 3
#define N_SAMPLE_AVERAGE 1 << N_BIT_SAMPLE_AVERAGE
//#define bool uint8_t
#define min(a,b) (((a) < (b)) ? (a) : (b))

#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 1000

typedef struct HSV HSV;
typedef struct RGB RGB;

uint16_t mic_samples[SAMPLE_SIZE]; // Array to hold the samples
uint8_t sampleIndex = 0;

struct RGB
{
	int r;
	int g;
	int b;
};

struct HSV
{
	float h;
	float s;
	float v;
};
//red should be PTB1 on the FRDM-KL46Z
//green should be PTB2 on the FRDM-KL46Z
//blue should be PTB3 on the FRDM-KL46Z
const int redPin = 1;   // PTB1
const int greenPin = 2; // PTB2
const int bluePin = 3;  // PTB3
clock_t colorChangeDelay = 0;
uint8_t red;
uint8_t blue;
uint8_t green;
HSV hsv;
int redTally;
int greenTally;
int blueTally;

bool isMusic = false;
clock_t lastMusicCheck = 0;
clock_t lastMusicDetection = 0;
clock_t lastBrightnessUpdate = 0;


extern volatile uint32_t msTicks;
clock_t clock(void)
{
    return msTicks;
}

void setup_ADC()
{


	int cal_v;

	//Enable clock gate for ADC0
	SIM->SCGC6 |= (1 << 27);
	ADC0->CFG1 = ADC_CFG1_ADICLK(0b00) | ADC_CFG1_MODE(0b10) | ADC_CFG1_ADIV(0b00);
	// Setup ADC
	// ADC0->CFG1 = 0;  // Default everything.
	// ADC0->CFG1 |= ADC_CFG1_ADICLK(0b00); // Use bus clock.
	// ADC0->CFG1 |= ADC_CFG1_MODE(0b11); //we are using 12bit
	// 									 // 00 for 8-bit
	//                                      // 01 for 12-bit
	//                                      // 10 for 10-bit
	                                     // 11 for 16-bit

	//Calibrate
	ADC0->SC3 = 0;
	ADC0->SC3 |= ADC_SC3_AVGS(0b10);  // SelectMaximum Hardware Averaging (32) see 28.3.7 for details
//	ADC0->SC3 |= ADC_SC3_AVGE_MASK;   // Enable Hardware Averaging
	ADC0->SC3 |= ADC_SC3_CAL_MASK;    // Start Calibration

	// Wait for calibration to complete
	while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));

	//Assume calibration worked, or check ADC_SC3_CALF

	// Calibration Complete, write calibration registers.
	cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->PG = cal_v;

	cal_v = 0;
	cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->MG = cal_v;


    // Turn off averaging for conversions
    ADC0->SC3 = 0;

    // Enable PORTE for analog mic input
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[20] = PORT_PCR_MUX(0);

	return;
}

uint16_t getMean()
{
	uint16_t mean = 0;
	for (int i = 0; i < SAMPLE_SIZE; i++)
	{
		mean += mic_samples[i];
	}
	mean = mean >> BITS_IN_SAMPLES; // Divide by 2^BITS_IN_SAMPLES to get the mean
	return mean;
}

static void pwm_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;

    /* FLL clock to all TPMs */
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    /* Pin mux: ALT3 = TPM */
    PORTB->PCR[1] = PORT_PCR_MUX(3);   /* PTB1 → TPM1_CH1 (Red)   */
    PORTB->PCR[2] = PORT_PCR_MUX(3);   /* PTB2 → TPM2_CH0 (Green) */
    PORTB->PCR[3] = PORT_PCR_MUX(3);   /* PTB3 → TPM2_CH1 (Blue)  */

    /* Same 8‑bit period on both timers */
    TPM1->MOD = TPM2->MOD = 255;

    /* Edge‑aligned, high‑true PWM channels */
    TPM1->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM2->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM2->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

    /* Start timers, prescaler ÷1 */
    TPM1->SC = TPM_SC_CMOD(1);
    TPM2->SC = TPM_SC_CMOD(1);
}

/* direct 0‑255 duty writes */
#define setRed(v)   (TPM1->CONTROLS[1].CnV = (v))
#define setGreen(v) (TPM2->CONTROLS[0].CnV = (v))
#define setBlue(v)  (TPM2->CONTROLS[1].CnV = (v))

//takes in a pin corresponding to a color and updates the duty cycle of that pin
void set_duty_cycle(int color_pin, uint8_t duty_cycle) {
	// Set the duty cycle for the specified color pin
	TPM1->CONTROLS[color_pin].CnV = duty_cycle; // Set the duty cycle value
	return;
}

void set_rgb_color(int r, int g, int b) {
	setRed(255 - r);   // Set the red LED duty cycle
	setGreen(255 - g); // Set the green LED duty cycle
	setBlue(255 - b);  // Set the blue LED duty cycle
	return;
}

volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
    msTicks++;
}

void init_systick(void) {
    SystemCoreClockUpdate();                       // Ensure SystemCoreClock is correct
    SysTick->LOAD  = (SystemCoreClock / 1000) - 1; // 1 ms tick
    SysTick->VAL   = 0;                            // Clear current value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk
                   | SysTick_CTRL_TICKINT_Msk
                   | SysTick_CTRL_ENABLE_Msk;
}

void delay_ms(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) { }
}

int random(int min, int max) {
    return min + rand() % (max - min + 1);
}

RGB hsvToRgb(HSV hsv)
{
	float r, g, b;
	int i;
	float f, p, q, t;

	if (hsv.s == 0)
	{
		r = g = b = hsv.v;
		return (RGB){(int)(r * 255), (int)(g * 255), (int)(b * 255)};
	}

	hsv.h /= 60;
	i = floor(hsv.h);
	f = hsv.h - i;
	p = hsv.v * (1 - hsv.s);
	q = hsv.v * (1 - hsv.s * f);
	t = hsv.v * (1 - hsv.s * (1 - f));

	switch (i)
	{
	case 0:
		r = hsv.v;
		g = t;
		b = p;
		break;
	case 1:
		r = q;
		g = hsv.v;
		b = p;
		break;
	case 2:
		r = p;
		g = hsv.v;
		b = t;
		break;
	case 3:
		r = p;
		g = q;
		b = hsv.v;
		break;
	case 4:
		r = t;
		g = p;
		b = hsv.v;
		break;
	default:
		r = hsv.v;
		g = p;
		b = q;
		break;
	}
	return 	(RGB){(int)(r * 255), (int)(g * 255), (int)(b * 255)};	;
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void transitionColor(RGB fromColor, RGB toColor, int transitionTime)
{
	int steps = 100;
	for (int i = 0; i <= steps; i++)
	{
		// Calculate intermediate color values
		int r = map(i, 0, steps, fromColor.r, toColor.r);
		int g = map(i, 0, steps, fromColor.g, toColor.g);
		int b = map(i, 0, steps, fromColor.b, toColor.b);

		// Write the inverted intermediate color values to the LED

		set_rgb_color(r, g, b); // Set the LED color

		delay_ms(transitionTime / (steps));
	}
}

#include "MKL46Z4.h"

void init_PIT_us(void) {
    // Enable the PIT clock; channel 0 will be used as a free-running timer
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    // Enable PIT module (0 = enable timers in debug mode)
    PIT->MCR = 0;
    // Configure channel 0 for free-running mode with a 1µs tick
    // Assuming SystemCoreClock is the bus clock in Hz (e.g. 48000000 Hz)
    // The LDVAL is the number of ticks before the counter resets. With a 1µs tick,
    // the reload value should be SystemCoreClock / 1000000 - 1.
    PIT->CHANNEL[0].LDVAL = SystemCoreClock/1000000 - 1;
    // Enable the timer (TEN) but do not enable interrupts (no TIEN)
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN_MASK;
}

// Returns microseconds since PIT channel 0 started running
uint32_t getMicroseconds(void) {
    // PIT timer counts down, so calculate elapsed time by subtracting the current value
    // from the reload value. If continuous measurement is needed, accumulate over rollovers.
    uint32_t elapsed = (SystemCoreClock/1000000) - PIT->CHANNEL[0].CVAL;
    return elapsed; 
    // You'll likely need additional code to handle rollover if timing longer intervals.
}


int main(void)
{
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
	setup_ADC();
//	init_PIT_us(); // Initialize PIT for microsecond timing
	init_systick(); // Initialize SysTick for delay functions
	pwm_init(); // Set up the LED pins for PWM
	HSV hsv = {0, 1, 1}; // Initialize HSV color with full saturation and value
	while (1)
	{

		unsigned int mic_val = 0;
		int mn = 1024;
		int mx = 0;
		int delta = 0;
		uint16_t sample_accum = 0;
		uint8_t sample_accum_index = 0;
		for (int i = 0; i < (256 << N_SAMPLE_AVERAGE); i++)
		{
			ADC0->SC1[0] = ADC_SC1_ADCH(8); //trigger a sample read I think
			while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));// wait for conversion complete
			mic_val = ADC0->R[0];
			sample_accum += mic_val;
			sample_accum_index += 1;
			if (sample_accum_index == N_SAMPLE_AVERAGE) {
				sample_accum_index = 0;
				mic_val = sample_accum >> N_BIT_SAMPLE_AVERAGE;
				mx = (mic_val > mx) ? mic_val : mx;
				mn = (mic_val < mn) ? mic_val : mn;
				sample_accum = 0;
			}

		}
		delta = mx - mn;
		mic_samples[sampleIndex] = delta;
		sampleIndex = (sampleIndex == SAMPLE_SIZE - 1) ? 0 : sampleIndex + 1;
		clock_t curr_time = clock();
		uint16_t mean = getMean();


	if ((delta > mean + mean / 5) && (curr_time - colorChangeDelay > 250) && (mean > 425))
	{
		int hueChange = random(40, 190);		// Ensure a significant change in color
		RGB oldColor = hsvToRgb(hsv);			// Store the current color
		hsv.h = fmod((hsv.h + hueChange), 360); // New hue value
		RGB rgb = hsvToRgb(hsv); // Generate the new color
		set_rgb_color(0, 0, 0); // Turn off the LEDs
		if (delta < mean + mean / 3.4)
		{
			transitionColor(oldColor, rgb, 200); // Transition with inverted colors
		}
		else
		{
			delay_ms(50);
		}
		red = rgb.r;
		green = rgb.g;
		blue = rgb.b;
//
		colorChangeDelay = clock();
	}
	else if (mean > 425) //min threshold amplitude
	{
		float brightnessFactor = (sqrt((min(500, delta) - mean / 2)) / 22.3) * 255;
		uint8_t r = floor((red * brightnessFactor) / 255);
		uint8_t g = floor((green * brightnessFactor) / 255);
		uint8_t b = floor((blue * brightnessFactor)) / 255;
		set_rgb_color(r, g, b); // Set the LED color based on the mic input
		lastBrightnessUpdate = clock();
	}
	else
	{
		set_rgb_color(0, 0, 0); // Turn off the LEDs if no sound is detected
	}
	}
	return 0;
}


