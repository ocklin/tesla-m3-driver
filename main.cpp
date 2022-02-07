
extern "C" {
    #include "device.h"

    #include "epwm.h"
    #include "gpio.h"

    #include <math.h>
}

#include "digio.h"
#include "gatedriver.h"
#include "pmic.h"
#include "scheduler.h"
#include "spidriver.h"

//
// Defines
//
#define EN_DRV_GATE_GPIO   124


float32_t targetSpeed  = 200.0f;          // target speed
float32_t theta        = 0.0f;          // >=0, < 2*pi, rotor angle used for transformations

float32_t iMeasureU, iMeasureV, iMeasureW;         // currents measured
float32_t iMeasureDCV;


// currents from park/clark
float32_t id, iq;

// target Iq, Id send out and controlling PWM
float32_t  targetIq = 0.2f, targetId = 0.7f;

float32_t maxModIndex = 0.0f;
float32_t carrierMid  = 0.0f;

#define PI 3.14159265358979

#define OnebySqrt3  1/sqrt(3)
#define Sqrt3  sqrt(3)


//
// Define the system frequency (MHz)
//
#define SYSTEM_FREQUENCY    (DEVICE_SYSCLK_FREQ / 1000000U)

#define PWM_FREQUENCY          10      // in KHz

#define INV_PWM_TICKS          (((SYSTEM_FREQUENCY / 2.0) / PWM_FREQUENCY)*1000)
#define INV_PWM_DB             (200.0)

#define INV_PWM_TBPRD          (INV_PWM_TICKS / 2)
#define INV_PWM_HALF_TBPRD     (INV_PWM_TBPRD / 2)

#define TPWM_CARRIER           (1000.0 / PWM_FREQUENCY)    //in uSec

// Analog scaling with ADC Boostxl
//
#define ADC_PU_SCALE_FACTOR          0.000244140625     // 1/2^12
#define ADC_PU_PPB_SCALE_FACTOR      0.000488281250     // 1/2^11

// Voltage Scale DC Voltage Boostxl
//
#define MAXIMUM_SCALE_VOLTAGE        74.1
#define VOLTAGE_SF                   (MAXIMUM_SCALE_VOLTAGE * ADC_PU_SCALE_FACTOR)
#define VOLTAGE_INV_SF               (4096.0 / MAXIMUM_SCALE_VOLTAGE)

//
// Can defines
//
#define CAN_MSG_DATA_LENGTH    8
#define CAN_TX_MSG_OBJ_ID      1

volatile uint32_t canTxMsgCount = 0;
uint16_t canTxMsgData[CAN_MSG_DATA_LENGTH];




//
// PWM Init
//
// EPWM1 - Phase U
// EPWM2 - Phase V
// EPWM3 - Phase W
uint32_t pwmHandle[3] = {EPWM1_BASE,
                         EPWM2_BASE,
                         EPWM3_BASE
};

// current and voltage sense
uint32_t adcHandle[4] = {ADCA_BASE,
                         ADCB_BASE,
                         ADCC_BASE,
                         ADCD_BASE
};

ADC_Channel adcChannel[4] = {
    ADC_CH_ADCIN2,
    ADC_CH_ADCIN2,
    ADC_CH_ADCIN2,
    ADC_CH_ADCIN14
};

__interrupt void engineCtrlISR(void);

const float32_t deltaTheta   = 2.0*PI/10000.0f;
float32_t thetas[6];
uint8_t theta_pos = 0;

void InitThetas() {
    thetas[0] =  18.0f*2.0f*PI/360.0f;
    thetas[1] =  95.0f*2.0f*PI/360.0f;
    thetas[2] = 150.0f*2.0f*PI/360.0f;
    thetas[3] = 189.0f*2.0f*PI/360.0f;
    thetas[4] = 285.0f*2.0f*PI/360.0f;
    thetas[5] = 347.0f*2.0f*PI/360.0f;
}

void InitParams() {

    // this is copied from TI motor control FCL
    float32_t FCL_COMPUTATION_TIME = 1;
    maxModIndex = ((float32_t)TPWM_CARRIER - (2 * (float32_t)FCL_COMPUTATION_TIME)) / (float32_t)TPWM_CARRIER;

    // not exactly sure if max PWM counter value (+1) is used here, mask? precision?
    carrierMid = maxModIndex * (float32_t)INV_PWM_HALF_TBPRD; // * 0x10000L;
}

void InitADC() {
    // U   ADC C2
    // V   ADC A2
    // W   ADC B2
    // VDC ADC D2

    uint16_t base;
    for(base = 0; base < 4; base++)
    {
        // Set 12-bit single ended conversion mode
        ADC_setMode(adcHandle[base], ADC_RESOLUTION_12BIT,
                    ADC_MODE_SINGLE_ENDED);

        // Set main clock scaling factor (100MHz max clock for the ADC module)
        ADC_setPrescaler(adcHandle[base], ADC_CLK_DIV_4_0);

        // set the ADC interrupt pulse generation to end of conversion
        ADC_setInterruptPulseMode(adcHandle[base], ADC_PULSE_END_OF_CONV);

        // enable the ADC
        ADC_enableConverter(adcHandle[base]);

        // set priority of SOCs
        ADC_setSOCPriority(adcHandle[base], ADC_PRI_ALL_HIPRI);
    }

    // delay to allow ADCs to power up
    DEVICE_DELAY_US(1500U);

    for(base = 0; base < 4; base++)
    {
        // Shunt Motor Currents measured on channel 2

        // SOC0 will convert pin A2/B2/C2,
        // sample window in SYSCLK cycles
        // trigger on ePWM1 SOCA/C
        ADC_setupSOC(adcHandle[base], ADC_SOC_NUMBER0,
                     ADC_TRIGGER_EPWM1_SOCA, adcChannel[base], 14);

        // Configure PPB to eliminate subtraction related calculation
        // PPB is associated with SOC0
        ADC_setupPPB(adcHandle[base], ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);

        // Write zero to this for now till offset ISR is run
        ADC_setPPBCalibrationOffset(adcHandle[base], ADC_PPB_NUMBER1, 0);
    }
}

void InitMotorCtrlInterrupt(void) {

    // engine control function in sync with PWM
    // EPWM_INT_TBCTR_ZERO
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);

    // This needs to be 1 for the INTFRC to work
    EPWM_setInterruptEventCount(EPWM1_BASE, 1);

    // Enable Interrupt Generation from the PWM module
    EPWM_enableInterrupt(EPWM1_BASE);

    // Clear ePWM Interrupt flag
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
}

void InitEnginePWM(void)
{
    uint16_t base;

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // *****************************************
    // Inverter PWM configuration - PWM 1, 2, 3
    // *****************************************
    for(base = 0; base < 3; base++)
    {
        // Time Base SubModule Registers
        // set Immediate load
        EPWM_setPeriodLoadMode(pwmHandle[base], EPWM_PERIOD_DIRECT_LOAD);
        EPWM_setTimeBasePeriod(pwmHandle[base], INV_PWM_TICKS);
        EPWM_setPhaseShift(pwmHandle[base], 0);
        EPWM_setTimeBaseCounter(pwmHandle[base], 0);
        EPWM_setTimeBaseCounterMode(pwmHandle[base], EPWM_COUNTER_MODE_UP_DOWN);
        EPWM_setClockPrescaler(pwmHandle[base], EPWM_CLOCK_DIVIDER_1,
                               EPWM_HSCLOCK_DIVIDER_1);

        EPWM_disablePhaseShiftLoad(pwmHandle[base]);

        // sync "down-stream"
        EPWM_setSyncOutPulseMode(pwmHandle[base],
                                      EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

        // Counter Compare Submodule Registers
        // set duty 0% initially
        EPWM_setCounterCompareValue(pwmHandle[base], EPWM_COUNTER_COMPARE_A, INV_PWM_TICKS / 2 / 4);
        EPWM_setCounterCompareShadowLoadMode(pwmHandle[base],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        // Action Qualifier SubModule Registers
        EPWM_setActionQualifierActionComplete(pwmHandle[base], EPWM_AQ_OUTPUT_A,
                (EPWM_ActionQualifierEventAction)(EPWM_AQ_OUTPUT_LOW_UP_CMPA |
                                               EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA));

        // Active high complementary PWMs - Set up the deadband
        EPWM_setRisingEdgeDeadBandDelayInput(pwmHandle[base],
                                             EPWM_DB_INPUT_EPWMA);
        EPWM_setFallingEdgeDeadBandDelayInput(pwmHandle[base],
                                              EPWM_DB_INPUT_EPWMA);

        EPWM_setDeadBandDelayMode(pwmHandle[base], EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(pwmHandle[base], EPWM_DB_FED, true);
        EPWM_setDeadBandDelayPolarity(pwmHandle[base], EPWM_DB_RED,
                                      EPWM_DB_POLARITY_ACTIVE_HIGH);
        EPWM_setDeadBandDelayPolarity(pwmHandle[base],
                                      EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
        EPWM_setRisingEdgeDelayCount(pwmHandle[base], 200);
        EPWM_setFallingEdgeDelayCount(pwmHandle[base], 200);
    }

    // configure 2 and 3 as dependents
    EPWM_setSyncOutPulseMode(EPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setPhaseShift(EPWM2_BASE, 2);
    EPWM_setCountModeAfterSync(EPWM2_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);

    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_enablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setPhaseShift(EPWM3_BASE, 2);
    EPWM_setCountModeAfterSync(EPWM3_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

void InitCan() {

    CAN_initModule(CANB_BASE);

    // Set up the CAN bus bit rate to 125kHz
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 125000, 20);
}

void PinMux_init()
{
    //EPWM1 -> myEPWM1 Pinmux
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    //EPWM2 -> myEPWM2 Pinmux
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    //EPWM3 -> myEPWM3 Pinmux
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setPinConfig(GPIO_5_EPWM3B);

    // Launchpad has CAN B equipped with a transceiver
    // GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);
    // GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);

}


#define OnebySqrt3  1/sqrt(3)

/*
    Clark alpha/beta:

    alpha = a
    beta = a*(1/sqrt(3)) + 2*b*(1/sqrt(3)) = (a + 2*b)*(1/sqrt(3))

    Park:

    d = alpha*cos + beta*sin
    q = -alpha*sin + beta*cos

    Inv Park:

    alpha = d*cos - q*sin
    beta = d*sin + q*cos

    Inv Clark:

    a = ua;
    b = (-ua + sqrt(3) * ub) / 2;
    c = (-ua - sqrt(3) * ub) / 2 = b - sqrt(3) * ub;
*/
uint16_t a = 0, b = 0, c = 0;
uint8_t cnt = 0;
float32_t minPwm = INV_PWM_HALF_TBPRD / 10.0f;

void runControl() {


    register float32_t  clarkeAlpha, clarkeBeta;
    register float32_t  parkSin, parkCos;

    float32_t Ualpha, Ubeta;
    float32_t Ta, Tb, Tc, minMaxOffset;

    // Clark
    clarkeAlpha = iMeasureU;
    clarkeBeta  = ((clarkeAlpha + 2.0f * iMeasureV) * (float32_t)OnebySqrt3);
    //printf("clark alpha %f, beta %f\n", clarkeAlpha, clarkeBeta);

    parkSin = sinf(theta);  // __sinpuf32(theta);
    parkCos = cosf(theta);  // __cospuf32(theta);

    //printf("park sin %f, cos %f\n", parkSin, parkCos);

    // Park
    iq = ((clarkeBeta * parkCos) - (clarkeAlpha * parkSin));
    id = ((clarkeBeta * parkCos) + (clarkeAlpha * parkSin));

    // PID Control step


    //printf("target %f, %f\n", targetId, targetIq);

    // Inv Park
    Ualpha = targetId * parkCos - targetIq * parkSin;
    Ubeta  = targetId * parkSin + targetIq * parkCos;

    // Inv Clark
    Ubeta *=Sqrt3; // *carrierMid;
    //Ualpha *=carrierMid;

    Ta = Ualpha;
    Tb = (Ubeta - Ualpha) / 2.0f;
    Tc = Tb - Ubeta;

    // printf("%.4f, %f, %f, %f, %f\n", theta, Ta, Tb, Tc, Ta + Tb + Tc);

    // using min-max for 3rd harmonic
    minMaxOffset  = __fmax(__fmax(Ta, Tc), Tb);
    minMaxOffset += __fmin(__fmin(Ta, Tc), Tb);
    minMaxOffset /= 2.0f;

    //printf("min + max offset %f\n", minMaxOffset);

    Ta -= minMaxOffset;
    Tc -= minMaxOffset;
    Tb -= minMaxOffset;

    Ta = Ta*carrierMid + carrierMid;
    Tb = Tb*carrierMid + carrierMid;
    Tc = Tc*carrierMid + carrierMid;

    a = (uint16_t)(uint32_t)Ta;
    b = (uint16_t)(uint32_t)Tb;
    c = (uint16_t)(uint32_t)Tc;

    //log_a[logCnt] = a;
    //log_b[logCnt] = b;
    //log_c[logCnt] = c;

    //logCnt++;

    // printf("%.4f, %f, %f, %f, %f\n", theta, Ta, Tb, Tc, Ta + Tb + Tc);
    //printf("\n");

    EPWM_setCounterCompareValue(pwmHandle[0], EPWM_COUNTER_COMPARE_A, a);
    EPWM_setCounterCompareValue(pwmHandle[1], EPWM_COUNTER_COMPARE_A, b);
    EPWM_setCounterCompareValue(pwmHandle[2], EPWM_COUNTER_COMPARE_A, c);
}


/*
 * get the current as measured at engine
 */
void getCurrentMeasurement() {

    // wait on ADC EOC
    while(ADC_getInterruptStatus(adcHandle[0], ADC_INT_NUMBER1) == 0);
    NOP;    //1 cycle delay for ADC PPB result

    iMeasureU = ADC_readPPBResult(ADCARESULT_BASE, ADC_PPB_NUMBER1) * ADC_PU_SCALE_FACTOR;
    iMeasureV = ADC_readPPBResult(ADCBRESULT_BASE, ADC_PPB_NUMBER1) * ADC_PU_SCALE_FACTOR;
    iMeasureW = ADC_readPPBResult(ADCCRESULT_BASE, ADC_PPB_NUMBER1) * ADC_PU_SCALE_FACTOR;
}

/*
 * get high DC voltage supply
 */
void getHDCV() {
    int32_t vcd = ADC_readPPBResult(ADCDRESULT_BASE, ADC_PPB_NUMBER1);
    iMeasureDCV =  vcd * VOLTAGE_SF;
}

/*
 * calc the theta angle and calc engine speed in Hertz
 */
void getTheta() {

    /* for testing we use 10kHz pwm (in kHz) triggering this via engineCtrl() func
       and deduct next angle from targetSpeed (Hz) */

    //float32_t ticksPerRotation = PWM_FREQUENCY * 1000.0f / targetSpeed;
    //theta += (2.0f * PI / ticksPerRotation);


    theta += deltaTheta;
    if(theta >= 2.0f * PI) theta -= 2.0f * PI;

    // theta = thetas[theta_pos];

    //theta_pos++;
    //if(theta_pos > 5) theta_pos = 0;
}


/*
 * main engine control loop triggered by timer irq
 */
__interrupt void engineCtrlISR(void) {

    getCurrentMeasurement();

    getHDCV();

    getTheta();

    // actual park/clark and inverse calculations
    runControl();

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // Acknowledge interrupt group
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

// task added to scheduler to strobe the powerwatchdog
static void taskStrobePowerWatchdoc() {
    TeslaM3PowerWatchdog<SpiDriver>::Strobe();
}

void main()
{
    // Initialize device clock and peripherals
    Device_init();

    // Disable pin locks and enable internal pull-ups.
    Device_initGPIO();

    // configure parameters used/needed
    InitParams();

    InitThetas();

    // Configure ePWMs pins
    PinMux_init();

    // Clear all interrupts and initialize PIE vector table:
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    Interrupt_initVectorTable();

    // connect the PWM 1 interrupts to motor control function
    InitEnginePWM();

    // init the ADC inputs for V measuring
    InitADC();

    // init the CAN
    InitCan();



    // Select SOC from counter
    // TODO - this requires more investigation,
    //        xxx_PERIOD causes EOC wait loop in engine irq to spend far too much time
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    // Generate pulse on 1st event
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    // Enable SOC on A group
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    // initialize all "classical" GPIO such as led, gate driver enablers, ...
    DIG_IO_CONFIGURE(DIG_IO_LIST);

    // tie the PWM1 interrupt to the motor control loop
    InitMotorCtrlInterrupt();

    // specify the ISR handler function
    // for whatever reasons this needs to be done in main() or here
    Interrupt_register(INT_EPWM1, &engineCtrlISR);

    // Enable AdcA-ADCINT1- to help verify EoC before result data read
    ADC_setInterruptSource(adcHandle[0], ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableContinuousMode(adcHandle[0], ADC_INT_NUMBER1);
    ADC_enableInterrupt(adcHandle[0], ADC_INT_NUMBER1);

    Scheduler::Init();

    // add a task to strobe the power watchdog every 100ms
    Scheduler::AddTask(taskStrobePowerWatchdoc, 100);

    // Enable PWM1INT in PIE group 3
    Interrupt_enable(INT_EPWM1);

    Interrupt_enableInCPU(INTERRUPT_CPU_INT3);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;          // Enable Global interrupt INTM
    ERTM;          // Enable Global realtime interrupt DBGM

    // enable gate drivers on lauchpad (!)
    DigIo::launchpad_gate.Clear();

    //
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      CAN Module: B
    //      Message Object ID Number: 1
    //      Message Identifier: 0x95555555
    //      Message Frame: Extended
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 4 Bytes
    //      No interrupts are enabled for the Transmit side
    //
    CAN_setupMessageObject(CANB_BASE, CAN_TX_MSG_OBJ_ID, 0x95555555,
                           CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, CAN_MSG_DATA_LENGTH);

    //
    // Initialize the transmit message object data buffer to be sent
    //
    canTxMsgData[0] = 0x12;
    canTxMsgData[1] = 0x34;
    canTxMsgData[2] = 0x56;
    canTxMsgData[3] = 0x78;
    canTxMsgData[4] = 0x78;
    canTxMsgData[5] = 0x78;
    canTxMsgData[6] = 0x78;
    canTxMsgData[7] = 0x78;

    CAN_startModule(CANB_BASE);

    TeslaM3GateDriver::Init();

    TeslaM3PowerWatchdog<SpiDriver>::Init();

    for(;;)
    {
        DigIo::led_out.Clear();

        DEVICE_DELAY_US(300000);

        DigIo::led_out.Set();

        DEVICE_DELAY_US(300000);

        CAN_sendMessage(CANB_BASE, CAN_TX_MSG_OBJ_ID, CAN_MSG_DATA_LENGTH, canTxMsgData);
    }
}


//
// End of file
//
