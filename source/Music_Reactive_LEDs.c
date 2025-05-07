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






/*
 * The macro definitions are for the setup function are in in MKL46Z4.h
 * To use, cut-n-paste this code into your project  and #include <MKL46Z4.h>
 * if it is not already included
 */
void SetupADC(){


	int cal_v;

	//Enable clock gate for ADC0
	SIM->SCGC6 |= (1 << 27);
	ADC0->CFG1 = ADC_CFG1_ADICLK(0b00) | ADC_CFG1_MODE(0b01) | ADC_CFG1_ADIV(0b00);
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


int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
	SetupADC();
	mic_buffer mic_data;
	init_mic_buffer(&mic_data);
	unsigned int sample_counter = 0;
	clock_t start_time = clock();
	while(1) {
		ADC0->SC1[0] = ADC_SC1_ADCH(8);      // ADC0_SE8 = PTB0

		while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));  // wait for conversion complete
		push_to_buffer(&mic_data, ADC0->R[0]);           // push the ADC value to the buffer

		sample_counter++;
		// Every 1000 samples, compute and print the elapsed time.
		if (sample_counter >= 256) {
			clock_t end_time = clock();
			uint32_t elapsed_milliseconds = (1000 * (end_time - start_time)) / CLOCKS_PER_SEC;
			PRINTF("Elapsed time for 1000 samples: %lld milliseconds\n", get_average_power(&mic_data));
			sample_counter = 0;          // reset sample counter
			start_time = clock();        // restart timing for next 1000 samples
		}

		// Optionally, you can keep the loud noise check if needed.
//		if (mic_data.buffer_power > 10000ULL) {
//			PRINTF("%llu \n", mic_data.buffer_power);
//			PRINTF("SUPER LOUD NOISE \n");
//		}
//        PRINTF("%lld %ld \n", mic_data.buffer_power, mic_data.mic_vals[mic_data.index - 1]);
//        PRINTF("%lld \n ", mic_data.buffer_power);
    }
    return 0;

}
