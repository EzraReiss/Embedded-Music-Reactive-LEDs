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

#define BUFFER_SIZE 256

typedef struct {
	uint8_t index;
	uint32_t mic_vals[BUFFER_SIZE];
	uint64_t buffer_power;
}mic_buffer;

mic_buffer mic_data;



/*
 * The macro definitions are for the setup function are in in MKL46Z4.h
 * To use, cut-n-paste this code into your project  and #include <MKL46Z4.h>
 * if it is not already included
 */
void SetupADC(){


	int cal_v;

	//Enable clock gate for ADC0
	SIM->SCGC6 |= (1 << 27);

	// Setup ADC
	ADC0->CFG1 = 0;  // Default everything.
	ADC0->CFG1 |= ADC_CFG1_ADICLK(0b00); // Use bus clock.
	ADC0->CFG1 |= ADC_CFG1_MODE(0b11); //we are using 12bit
										 // 00 for 8-bit
	                                     // 01 for 12-bit
	                                     // 10 for 10-bit
	                                     // 11 for 16-bit

	//Calibrate
	ADC0->SC3 = 0;
	ADC0->SC3 |= ADC_SC3_AVGS(0b11);  // SelectMaximum Hardware Averaging (32) see 28.3.7 for details
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

void push_to_buffer() {
	uint32_t old_val = mic_data.mic_vals[mic_data.index];
	mic_data.buffer_power -= old_val;
	uint16_t new_val = ADC0->R[0];
	uint32_t new_power = new_val*new_val;
	new_power = new_power >> 8;
	mic_data.buffer_power += new_power;
	mic_data.mic_vals[mic_data.index] = new_power;
	mic_data.index++;
}

int main(void) {
	mic_data.index = 0;
	mic_data.buffer_power = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		mic_data.mic_vals[i] = 0;
	}


    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    SetupADC();

    while(1) {
        ADC0->SC1[0] = ADC_SC1_ADCH(8);      // ADC0_SE8 = PTB0
        while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));  // wait for complete
        push_to_buffer();
        if (mic_data.buffer_power > 700043789) {
        	PRINTF("%lld \n ", mic_data.buffer_power);
        	PRINTF("SUPER LOUD NOISE \n");
        }
//        PRINTF("%lld %ld \n", mic_data.buffer_power, mic_data.mic_vals[mic_data.index - 1]);
//        PRINTF("%lld \n ", mic_data.buffer_power);
    }
    return 0;

}
