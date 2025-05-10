/* FRDM‑KL46Z: PWM on PTB1/PTB2/PTB3
 * Duty: PTB1 = 64  (25 %)
 *       PTB2 = 128 (50 %)
 *       PTB3 = 192 (75 %)
 * Carrier ≈ 48 MHz / 256 ≈ 187 kHz
 */
#include "MKL46Z4.h"

#define PWM_PERIOD 255U     /* 8‑bit full‑scale */

static void init_pwm_pins(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;       /* Enable Port‑B clock */

    /* ALT3 routes the TPM signals to the pins (see pin‑mux table) */
    PORTB->PCR[1] = PORT_PCR_MUX(3);          /* PTB1  → TPM1_CH1 */
    PORTB->PCR[2] = PORT_PCR_MUX(3);          /* PTB2  → TPM2_CH0 */
    PORTB->PCR[3] = PORT_PCR_MUX(3);          /* PTB3  → TPM2_CH1 */
}

static void init_tpm1_pwm(void)
{
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;        /* Clock TPM1   */
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);        /* 48 MHz FLL    */

    TPM1->SC = 0;                             /* Disable while configuring */
    TPM1->MOD = PWM_PERIOD;                   /* 256‑count period         */

    /* Channel 1 : edge‑aligned, high‑true */
    TPM1->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM1->CONTROLS[1].CnV  = 64;              /* 25 % duty               */

    TPM1->SC = TPM_SC_CMOD(1) | TPM_SC_PS(0); /* Start, prescale ÷1      */
}

static void init_tpm2_pwm(void)
{
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;        /* Clock TPM2   */
    /* TPMSRC already set above */

    TPM2->SC = 0;
    TPM2->MOD = PWM_PERIOD;

    /* CH0 (PTB2) and CH1 (PTB3) */
    TPM2->CONTROLS[0].CnSC =
    TPM2->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

    TPM2->CONTROLS[0].CnV = 128;              /* 50 % */
    TPM2->CONTROLS[1].CnV = 192;              /* 75 % */

    TPM2->SC = TPM_SC_CMOD(1) | TPM_SC_PS(0);
}

/* One‑byte API for run‑time updates */
void set_duty_ptb1(uint8_t v) { TPM1->CONTROLS[1].CnV = v; }
void set_duty_ptb2(uint8_t v) { TPM2->CONTROLS[0].CnV = v; }
void set_duty_ptb3(uint8_t v) { TPM2->CONTROLS[1].CnV = v; }

int main(void)
{
    init_pwm_pins();
    init_tpm1_pwm();
    init_tpm2_pwm();
    while (1) { __WFI(); }    /* CPU can sleep – PWM is hardware‑driven */
}
