/*
    Sebastian Cox
    Keegan McGilvray
    12/06/2024
    Lab #10
    This code allows the robot so self navigate a path of a black line
 */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>

#define PERIOD 100  // PWM
#define DUTY 25 // percent ON
#define CLOCKDIVIDER 48  // Clock Divider Value
#define LEFTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_4
#define RIGHTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_3

typedef enum {ALL_LEFT, MORE_LEFT, MIDDLE, MORE_RIGHT, ALL_RIGHT, OFF, STANDBY} Condition;
typedef enum {
  LED2OFF,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  MAGENTA,
  WHITE,
LED2MAINTAIN}
led2;
typedef enum {motorOFF, motorFoward, motorReverse} motorState;
typedef enum {buttonON, buttonOFF} buttonStates;
typedef enum{lastLeft, lastRight} whiteturning;
typedef enum{Middle, glastLeft,gwlastLeft, glastRight, gwlastRight} greenturning;
typedef enum{wMiddle ,wLeft, wRight} allturning;
typedef enum{reset, stable} speed;

Timer_A_UpModeConfig timerConfig;
Timer_A_PWMConfig  timerPWMConfig;

void config432IO(void);
void configRobotIO(void);
void readBumperSwitches(void);
void powerUp(void);
void standBy(void);
void tableII(void);
void rotate(Condition);
void bumperSwitchesHandler(void);
void readSensor(void);
void Input_Output(void);
void Tracking(void);
void Lost(void);
void sensorReader(void);
void configPWMTimer(uint16_t clockPeriod, uint16_t clockDivider, uint16_t duty,uint16_t channel);
void RGBDriver(led2 LEDColor);
buttonStates b0, bn;
motorState leftMotorState, rightMotorState;
whiteturning wturn = lastLeft;
greenturning gturn = Middle;
allturning aturn = wMiddle;
Condition Con = OFF;
led2 LED2State = LED2OFF;
speed izer = stable;
speed stabl = reset;
speed stabr = reset;
uint8_t LEFT = 0, RIGHT = 0, NumSensor = 0;



int main(void) {

    b0 = buttonOFF;

    MAP_WDT_A_holdTimer();


    config432IO(); // Config MSP432 LEDs
    configRobotIO(); // Config Robot
    powerUp();

   configPWMTimer(PERIOD,CLOCKDIVIDER,DUTY,LEFTCHANNEL); // Start L wheel timer
   configPWMTimer(PERIOD,CLOCKDIVIDER,DUTY,RIGHTCHANNEL); // Start R wheel timer

   MAP_Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);

   leftMotorState = motorOFF;
   rightMotorState = motorOFF;

   __enable_interrupt();

   MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); // CNTL EVEN LOW
   MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2); // CNTL ODD LOW

   bumperSwitchesHandler();

   __delay_cycles(1500000);
    while (1) {

        Input_Output();

        if (b0 == buttonON){

            readSensor();
            sensorReader();
            MAP_Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
            if(izer==stable){
            __delay_cycles(15000);
            }
            else{
                __delay_cycles(3050);
            }
        }

        else{

            standBy();
        }

        } // End of while
} // End of main


// Function configures the PWM timer
void configPWMTimer(uint16_t clockPeriod, uint16_t clockDivider, uint16_t duty, uint16_t channel) {
const uint32_t TIMER=TIMER_A0_BASE;
uint16_t dutyCycle = duty*clockPeriod/100;
timerPWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
timerPWMConfig.clockSourceDivider = clockDivider;
timerPWMConfig.timerPeriod = clockPeriod;
timerPWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;
timerPWMConfig.compareRegister = channel;
timerPWMConfig.dutyCycle = dutyCycle;
MAP_Timer_A_generatePWM(TIMER, &timerPWMConfig);
MAP_Timer_A_stopTimer(TIMER);

}


// Function sets LEDs as outputs and have them begin off
void config432IO(void){


       MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); // Red LED1
       MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); // Red 2
       MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1); // Green
       MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); // Blue

       // Turn OFF
       MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); // Red LED1
       MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); // Red 2
       MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // Green
       MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); // Blue


}

// Function sets all the bumpers as inputs
void configRobotIO(void){

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0); // Front left yellow LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN5); // Front Right yellow LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN7); // Back right red LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN6); // Back left Red LED

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); // Set them all to Low L yellow
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5); // R yellow
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7); // R red
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6); // L Red


    // Set bumpers as inputs
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN0);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN2);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN3);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN5);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN6);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN7);

    // Enable Interrupts
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN2);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN3);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN5);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN6);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN7);


    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN0,GPIO_HIGH_TO_LOW_TRANSITION); // Bumper 0
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN2,GPIO_HIGH_TO_LOW_TRANSITION); // Bumper 1
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN3,GPIO_HIGH_TO_LOW_TRANSITION); // Bumper 2
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN5,GPIO_HIGH_TO_LOW_TRANSITION); // Bumper 3
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN6,GPIO_HIGH_TO_LOW_TRANSITION); // Bumper 4
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN7,GPIO_HIGH_TO_LOW_TRANSITION); // Bumper 5

    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN0); // B#0
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN2); // B#1
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN3); // B#2
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN5); // B#3
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN6); // B#4
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN7); // B#5



     // Right
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0); // Enable Right
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5); // Direction Right wheel
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6); // PWM Right
     MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

     // Left
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2); // Enable Left wheel
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4); // Direction Right wheel
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7); // PWM Left wheel
     MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

     // Add Callback fuction
     MAP_GPIO_registerInterrupt(GPIO_PORT_P4, bumperSwitchesHandler);

     // Right
     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Enable Right wheel
     MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6); // PWM Right wheel


     // Left
     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // Enable Left wheel
     MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel
     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // PWM Left wheel


     // Config Sensor
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3); // CNTL EVEN
     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);


     MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2); // CNTL ODD
     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);





}




void powerUp(void){

    //TOGGLE LED 1 ONCE
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);   // LED1 ON-RED

    // RSLK LED TOGGLE ONCE
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0); // Set them all to Low L yellow
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5); // R yellow
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7); // R red
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6); // L Red

    // TOGGLE LED2 WHITE
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);    // LED2 Red
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);   // LED2 GREEN
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);   // LED2 BLUE
    __delay_cycles(3000000);

    //TOGGLE LED 1 ONCE
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);   // LED1 ON-RED

    // RSLK LED TOGGLE ONCE
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); // Set them all to Low L yellow
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5); // R yellow
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7); // R red
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6); // L Red

    // TOGGLE LED2 WHITE
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // LED2 Red
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);   // LED2 GREEN
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);   // LED2 BLUE
    __delay_cycles(3000000);



    // WHEELS ARE OFF
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel

}

void standBy(void) {


    // TOGGLE 1 HZ
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);   // LED1 ON-RED
    __delay_cycles(3000000);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);   // LED1 ON-RED
    __delay_cycles(3000000);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // LED2 Red
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);   // LED2 BLUE
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);   // GREEN ON

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel
}


void tableII(void){

    // TABLE 2 LED 2 COLORS

    if  ((LEFT == 0) && (RIGHT == 0)){// OFF

            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);    // Blue ON
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);    // Green ON
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);    // Red OFF

        }

    else if ((LEFT > 0) && (RIGHT == 0)){ // Red
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); // Red ON

        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);    // Blue OFF
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // Green OFF


    }

    else if ((RIGHT > 0) && (LEFT == 0)){ // Blue

            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);    // Blue ON

            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);    // Green OFF
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // Red OFF

        }

    else if (LEFT > RIGHT){ // Yellow

        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); // Red ON
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); // Green ON

        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);    // Blue OFF
    }

    else if (LEFT == RIGHT){ // Green

        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);    // Green ON

        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);    // Blue OFF
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // Red OFF

    }

    else { // Cyan

        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);    // Blue ON
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);    // Green ON

        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // Red OFF

    }








}




void rotate(Condition Cond) {


        if (Cond == ALL_LEFT){
           if(stabl == reset){
           configPWMTimer(PERIOD,CLOCKDIVIDER,35,LEFTCHANNEL);
           configPWMTimer(PERIOD,CLOCKDIVIDER,50,RIGHTCHANNEL);
           //izer = stable;
           }
           else{
               configPWMTimer(PERIOD,CLOCKDIVIDER,35,LEFTCHANNEL);
               configPWMTimer(PERIOD,CLOCKDIVIDER,26,RIGHTCHANNEL);

           }

            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); // Direction Left wheel
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); // Direction Right wheel


            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel

            stabr = reset;
            izer = stable;
            wturn = lastLeft;

        }

        else if (Cond == MORE_LEFT){ // FRONT ON REAR OFF

          configPWMTimer(100,CLOCKDIVIDER,5,LEFTCHANNEL); // Start L wheel timer
           configPWMTimer(100,CLOCKDIVIDER,5,RIGHTCHANNEL);

            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4); // Direction Left wheel
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); // Direction Right wheel

            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel

            izer = reset;
            stabr = reset;
            wturn = lastLeft;
            gturn = glastLeft;
        }
        else if (Cond == MIDDLE){ // FRONT ON REAR OFF

//            if(gturn == glastLeft){
//                configPWMTimer(PERIOD,CLOCKDIVIDER,25,LEFTCHANNEL); // Start L wheel timer
//                configPWMTimer(PERIOD,CLOCKDIVIDER,22,RIGHTCHANNEL);
//            }
//            else if(gturn == glastRight){
//                configPWMTimer(PERIOD,CLOCKDIVIDER,22,LEFTCHANNEL); // Start L wheel timer
//                configPWMTimer(PERIOD,CLOCKDIVIDER,25,RIGHTCHANNEL);
//            }
//            else if(gturn == gwlastLeft){
//                configPWMTimer(PERIOD,CLOCKDIVIDER,25,LEFTCHANNEL);
//                configPWMTimer(PERIOD,CLOCKDIVIDER,22,RIGHTCHANNEL);
//            }
//            else if(gturn == gwlastRight){
//                configPWMTimer(PERIOD,CLOCKDIVIDER,22,LEFTCHANNEL); // Start L wheel timer
//                configPWMTimer(PERIOD,CLOCKDIVIDER,25,RIGHTCHANNEL);
//            }
//            else{
              configPWMTimer(PERIOD,CLOCKDIVIDER,50,LEFTCHANNEL);
              configPWMTimer(PERIOD,CLOCKDIVIDER,50,RIGHTCHANNEL);
 //           }

                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); // Direction Left wheel
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); // Direction Right wheel

                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel


                    izer = stable;
                    aturn = wMiddle;
                    stabl = reset;
                    stabr = reset;
         }
        else if (Cond == MORE_RIGHT){ // FRONT ON REAR OFF
                  configPWMTimer(100,CLOCKDIVIDER,5,LEFTCHANNEL); // Start L wheel timer
                   configPWMTimer(100,CLOCKDIVIDER,5,RIGHTCHANNEL);

                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); // Direction Left wheel
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5); // Direction Right wheel

                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel


                    izer = reset;
                    stabl = reset;
                    wturn = lastRight;
                    gturn = glastRight;
                }
        else if (Cond == ALL_RIGHT){ // FRONT ON REAR OFF

            if(stabr == reset){
              configPWMTimer(PERIOD,CLOCKDIVIDER,50,LEFTCHANNEL); // Start L wheel timer
              configPWMTimer(PERIOD,CLOCKDIVIDER,35,RIGHTCHANNEL);
              //izer = stable;
            }
            else{
               configPWMTimer(PERIOD,CLOCKDIVIDER,26,LEFTCHANNEL); // Start L wheel timer
                configPWMTimer(PERIOD,CLOCKDIVIDER,35,RIGHTCHANNEL);

            }

               MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); // Direction Left wheel
               MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); // Direction Right wheel

             MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel


//
              wturn = lastRight;
              izer = stable;
              stabl = reset;
//
               }

        else { // FRONT OFF REAR ON

           configPWMTimer(100,CLOCKDIVIDER,5,LEFTCHANNEL); // Start L wheel timer
           configPWMTimer(100,CLOCKDIVIDER,5,RIGHTCHANNEL);



            if(wturn == lastLeft){
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); // Direction Right wheel
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4); // Direction Left wheel
            aturn = wLeft;
            }
            else{
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5); // Direction Right wheel
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); // Direction Left wheel
            aturn = wRight;
            }

            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel

            izer = reset;
            stabl = stable;
            stabr = stable;


        }
}


  void bumperSwitchesHandler(){

        uint16_t status;
        __delay_cycles(300000);

        status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);

        switch(status){

        case GPIO_PIN0: // B0
            if(b0 == buttonON){
                b0 = buttonOFF;
            }
            else {
                b0 = buttonON;
            }
            break;


        }

    }







    void Input_Output(void){

        NumSensor = 0;

        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); // CNTL EVEN HIGH
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2); // CNTL ODD HIGH


        MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0); // QTR0
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1); // QTR1
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2); // QTR2
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3); // QTR3
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4); // QTR4
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5); // QTR5
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6); // QTR6
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN7); // QTR7

        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN1);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN2);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN3);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN5);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN6);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN7);

        __delay_cycles(30); // Delay 10 us

        MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0); // QTR0
        MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN1); // QTR1
        MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN2); // QTR2
        MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN3); // QTR3
        MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN4); // QTR4
        MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN5); // QTR5
        MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN6); // QTR6
        MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN7); // QTR7
        __delay_cycles(3000); // Delay 10 ms


        __delay_cycles(30);

        uint8_t pin;

        for (pin = 0; pin < 8; pin++) {

          if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, (1 << pin))) {

            NumSensor |= (1 << pin); // Set the corresponding bit in sensorVals

          }

        }



        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); // CNTL EVEN LOW
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2); // CNTL ODD LOW



    }


    void readSensor(void){

        LEFT = 0;
        RIGHT = 0;


            if (NumSensor & BIT0){ // Right
                RIGHT++;
            }

            if (NumSensor & BIT1){
                RIGHT++;
            }

            if (NumSensor & BIT2){
                RIGHT++;
            }

            if (NumSensor & BIT3){
                RIGHT++;
            }



            if (NumSensor & BIT4){ // Left
                LEFT++;
            }

            if (NumSensor & BIT5){
                LEFT++;
            }

            if (NumSensor & BIT6){
                LEFT++;
            }

            if (NumSensor & BIT7){
                LEFT++;
            }

            if  ((LEFT == 0) && (RIGHT == 0)){// OFF
                Con = OFF;
            }

            else if ((LEFT > 0) && (RIGHT == 0)){ // Red
                Con = ALL_LEFT;
            }

            else if ((RIGHT > 0) && (LEFT == 0)){ // Blue
                Con = ALL_RIGHT;
            }

            else if (LEFT > (1+RIGHT)){ // Yellow
                Con = MORE_LEFT;
            }

            else if (LEFT == RIGHT){ // Green
                Con = MIDDLE;
            }

            else if(RIGHT > (1+LEFT)) { // Cyan
                Con = MORE_RIGHT;
            }

    }


    void Tracking(void){

        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); // RED LED1
        tableII();

    }


    void Lost(void){

        // LED1
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);

        // LED2
        // TOGGLE LED2 WHITE
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);    // LED2 Red
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);   // LED2 GREEN
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);   // LED2 BLUE

        // RSLK
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); // Set them all to Low L yellow
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5); // R yellow
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7); // R red
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6); // L Red

        // Wheels
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // Sleep Right wheel
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // Sleep Left wheel




    }


void RGBDriver(led2 LEDColor)

{

  switch (LEDColor)

  {

  case LED2OFF:

    // RED AND GREEN AND BLUE OFF

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

    break;

  case RED:

    // RED ON

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);

    // GREEN AND BLUE OFF

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

    break;

  case GREEN:

    // GREEN OFF

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);

    // RED AND BLUE OFF

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

    break;

  case BLUE:

    // BLUE ON

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);

    // RED AND GREEN OFF

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

    break;

  case CYAN:

    // BLUE AND GREEN ON

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);

    // RED OFF

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    break;

  case YELLOW:

    // RED AND GREEN ON

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);

    // BLUE OFF

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

    break;

  case MAGENTA:

    // RED AND BLUE ON

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);

    // GREEN OFF

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

    break;

  case WHITE:

    // RED AND GREEN AND BLUE ON

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);

    break;

  case LED2MAINTAIN:

    break;

  }

}

void sensorReader(void){

    switch (Con)

  {

  case ALL_LEFT:
    RGBDriver(RED);
    rotate(ALL_LEFT);
    break;

  case MORE_LEFT:
    RGBDriver(YELLOW);
    rotate(MORE_LEFT);
    break;

  case MIDDLE:
    RGBDriver(GREEN);
    rotate(MIDDLE);
    break;

  case MORE_RIGHT:
    RGBDriver(CYAN);
    rotate(MORE_RIGHT);

    break;

  case ALL_RIGHT:
    RGBDriver(BLUE);
    rotate(ALL_RIGHT);
    break;

  case OFF:
    RGBDriver(WHITE);
    rotate(OFF);
    break;

  }

  if (Con != OFF)

  {

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

  } else

  {

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

  }
}
