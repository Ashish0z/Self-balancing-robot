#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "pid.h"

#define PID_KP  0.2f
#define PID_KI  0.0005f
#define PID_KD  0.0025f

#define PID_TAU 0.02f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

#define PWM_FREQUENCY 7000

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint8_t ui8Adjust;
volatile float adjust;

PIDController pid = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
                          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };
//
// A boolean that is set when a MPU6050 command has completed.
//
volatile bool g_bMPU6050Done;

//
// I2C master instance
//
tI2CMInstance g_sI2CMSimpleInst;

//
//Device frequency
//
int clockFreq;

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());

}

/*void ConfigureUART(void) { // Função retirada do exemplo hello.c
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}*/

void delayMS(int ms) {
    //ROM_SysCtlDelay( (ROM_SysCtlClockGet()/(3*1000))*ms ) ;  // more accurate
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

//
// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
//
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // See if an error occurred.
    //
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        //
        // An error occurred, so handle it here if required.
        //
    }
    //
    // Indicate that the MPU6050 transaction has completed.
    //
    g_bMPU6050Done = true;
}

//
// The interrupt handler for the I2C module.
//
void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler.
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

//
// The MPU6050 example.
//

float x = 0, y = 0, z = 0;
void MPU6050Example(void)
{
    float fAccel[3], fGyro[3];
    tMPU6050 sMPU6050;

    //
    // Initialize the MPU6050. This code assumes that the I2C master instance
    // has already been initialized.
    //
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Configure the MPU6050 for +/- 4 g accelerometer range.
    //
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
        MPU6050_ACCEL_CONFIG_AFS_SEL_4G, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }


    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Loop forever reading data from the MPU6050. Typically, this process
    // would be done in the background, but for the purposes of this example,
    // it is shown in an infinite loop.
    //

    while (1)
    {
        //
        // Request another reading from the MPU6050.
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        //
        // Get the new accelerometer and gyroscope readings.
        //
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1],
            &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
        //
        // Do something with the new accelerometer and gyroscope readings.
        //

        //x = fGyro[0];
        //y = fGyro[1];
        z = fGyro[2];

        x = (atan2(fAccel[0], sqrt (fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2]))*180.0)/3.14;

        y = (atan2(fAccel[1], sqrt (fAccel[0] * fAccel[0] + fAccel[2] * fAccel[2]))*180.0)/3.14;

        PIDController_Update(&pid, 0.0f, x);
        adjust = pid.out * 10;
        float adj = fmod(fabs(adjust),100);
        if (adjust <= 0) {
            PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, adj * ui32Load / 100); // Pulse width which gives duty cycle
            PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);

            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, adj * ui32Load / 100); // Pulse width which gives duty cycle
            PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
        }
        else {
            PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
            PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, adj * ui32Load / 100); // Pulse width which gives duty cycle
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);

            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, adj * ui32Load / 100); // Pulse width which gives duty cycle
            PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
        }
        //UARTprintf("Ang. X: %d | Ang. Y: %d | Ang. Z: %d\n", (int)x, (int)y, (int)z);

        delayMS(10);
    }
}

int main()
{
    //clockFreq = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 16000000);
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    PIDController_Init(&pid);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

        SysCtlPWMClockSet(SYSCTL_PWMDIV_64); //PWM Clock
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_3);
        GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);

        GPIOPinConfigure(GPIO_PF2_M1PWM6);
        GPIOPinConfigure(GPIO_PF3_M1PWM7);
        GPIOPinConfigure(GPIO_PC4_M0PWM6);
        GPIOPinConfigure(GPIO_PC5_M0PWM7);

        ui32PWMClock = SysCtlClockGet() / 64;
            ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

            PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
            PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load); // Define PWM Period
            PWMGenEnable(PWM1_BASE, PWM_GEN_3);

            PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
            PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load); // Define PWM Period
            PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    InitI2C0();
    //ConfigureUART();
    MPU6050Example();
    return(0);
}
