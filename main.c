

/**
 * main.c
 *
 * Proyecto realizado por: Javier Alejandro Pérez Marín
 */

/*
 * BIBLIOTECAS EMPLEADAS
 */
#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_memmap.h" //memory map of the Tiva C
#include "driverlib/interrupt.h"
#include "inc/hw_types.h" //common types and macros
#include"inc/tm4c123gh6pm.h" //CLK
#include "driverlib/pin_map.h"
#include"driverlib/sysctl.h"
#include"driverlib/gpio.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

/*
 * VARIABLES Y ARREGLOS UTILIZADOS
 */
int FSM, i = 17, flag_bt1 = 0, flag_bt2 = 0, cont_led = 0, op = 0;;
uint8_t consola, STLED0 = 0, STLED1 = 0, STLED2 = 0, STLED3 = 0, BT_COUNT = 0, TMR0_COUNT = 0, TMR0_AA = 0, POT = 0;
char ST7SEG, LED1, LED2, LED3, LED4;
unsigned int ADRES[4];
uint8_t DISP [18] = {0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,0b01101101,0b01111101,
                     0b00000111,0b01111111,0b01101111,0b1110111,0b1111100,0b0111001,0b1011110,
                     0b01111001,0b01110001,0b10000000,0b00000000};
uint8_t snake[9] = {0b00000100,0b01000100,0b01100000,0b00100001,0b00000011,0b01000010,0b01010000,0b00011000,0b10000000};
char imp[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

/*
 * Definiciones
 */

//Valores de entrada ADC
#define IN_MIN 0
#define IN_MAX 4095

//Valores de mapeo
#define OUT_MIN 0
#define OUT_MAX 99
/*
 * PROTOTIPOS DE FUNCIONES EMPLEADAS
 */
void CONFIG(void); //Configuraciones OSC y GPIOs
void SET_UART(void); //Configuración módulo UART
void SET_TIMER0(void); //Configuración módulo TIMER0
void SET_ADC(void); //Configuración módulo ADC

void AA_LEDs(void); //Animación LEDS
unsigned short map(uint32_t val, uint32_t in_min, uint32_t in_max,
                   unsigned short out_min, unsigned short out_max); //Mapeo

void PB(void); //Handler INT. botones
void UARTint(void);
void TMR0int(void);
void ADCint(void);


int main(void){
    CONFIG(); //Se pasa a configurar la TIVA
    SET_UART(); //Se configura e inicializa módulo UART
    SET_TIMER0(); //Se configura e inicializa TIMER0
    SET_ADC(); //Se configura e inicializa módulo ADC
    IntMasterEnable(); //Se habilita respuesta del procesador a interrupciones

    //Loop principal
    while(1){
        FSM = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3); //Se lee en cada repetición estado de DIP SW
        switch(FSM) {
            case 0: //Estado 1 (Control por UART de LEDs y 7SEG)
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, STLED0); //Se modifica estado de led1
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, STLED1); //Se modifica estado de led2
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, STLED2); ////Se modifica estado de led3
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, STLED3); //Se modifica estado de led4
                GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, DISP [i]); //Se despliega valor de la tabla
                break;
            case 4: //Estado 2 (Contador independiente en 7SEG y contador de botones en LEDs)
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, BT_COUNT); //Se manda valor de contador LEDs
                GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, DISP [TMR0_COUNT]); //Se despliega valor de la tabla
                break;
            case 8: //Estado 3 (Animación en LEDs y 7SEG)
                AA_LEDs(); //Función para animación LEDs
                GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, snake [TMR0_AA]); //Se despliega valor de la tabla animada
                break;
            case 12: //Estado 4 (Lectura de POT y mapeado de 0  a 99)
                ADCProcessorTrigger(ADC0_BASE, 1);
                while(!ADCIntStatus(ADC0_BASE, 1, false)); //Mientras la conversión esté activa
                ADCIntClear(ADC0_BASE, 1); //Clear ADC flag ISR
                ADCSequenceDataGet(ADC0_BASE, 1, ADRES); //Se lee y almacena el valor leido

                POT = map(ADRES[0], IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); //Mapeo de ADC de 0 a 99
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, POT/10); //Se manda valor de decenas a contador LEDs
                GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, DISP [POT%10]); //Se despliega valor de unidades con la tabla
                break;
            default:
                break;
        }
    }
}

/*
 * FUNCIONES
 */
void CONFIG(void){
    //Reloj interno (Equivalente a set de OSC del PIC)
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //FREQ/4 (50 MHz), POS IN EDGE, 16 MHz, OSC PRINCIPAL

    //Enable CLK para Periféricos GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Entradas y Salidas (Equivalente a TRISbits del PIC)

    //Entradas (DIP1 en PORTA2, DIP2 en PORTA3, PB1 en PORTA4, PB2 en PORTA5)
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

    //Salida (7SEG en PORTB (ABCDEFG & DP), 4 LEDs (PD0 - PD3)
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    //Configuración Adicional de PORTA (Pull-Up e interrupciones)
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD); //PUSH-PULL pin -> PORTA2 y PORTA3 con corriente Low Level (2mA)
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Weak Pull Up PORTA4 y PORTA5 con corriente Low Level (2mA)
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_4|GPIO_INT_PIN_5); //Se habilita interrupción para PORTA4 y PORTA5 (Botones)
    IntPrioritySet(INT_GPIOA, 0); // Se configura como alta prioridad a las interrupciones del PORTA
    IntRegister(INT_GPIOA, PB); // Se asigna la función "PB" para ser el handler de las interrupciones de PORTA
    IntEnable(INT_GPIOA); // ISR PORTA -> Enable
    return;
}

void SET_UART(){
    //Se habilita módulo UART0 (PC Access)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTStdioConfig(0, 9600, 16000000); // UART0 con baudaje de 115200 con freq de 16 Mhz

    //RX & TX IO UART0
    GPIOPinConfigure(GPIO_PA0_U0RX); //Recepción
    GPIOPinConfigure(GPIO_PA1_U0TX); //Envío

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); //Se indica que el tipo de pin para PA0 y PA1 es de UART

    //Config UART con parámetros: UARTbase, OSC, BAUD,  QTY. BITS, QTY. STOP BITS, PARIDAD
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntEnable(INT_UART0); // ISR UART0 -> Enable

    UARTFIFODisable(UART0_BASE); //FIFOs Disabled
    UARTIntEnable(UART0_BASE, UART_INT_RX); //Se habilitan interrupciones de Recepción
    return;
}

void SET_TIMER0(){
    //Se habilita módulo TIMER0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //Se configura TIMER0 como temporizador periódico de ancho completo

    TimerLoadSet(TIMER0_BASE, TIMER_A, ((SysCtlClockGet())/4) - 1); //Se reinicia timer para iniciar cuenta en 0 (250 ms)

    IntEnable(INT_TIMER0A); //ISR TIMER0A -> Enable
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Se habilita interrupción Timer A uDMA complete

    TimerEnable(TIMER0_BASE, TIMER_A); //TIMER0 -> Enable
    return;
}
void SET_ADC(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Se habilita PORTE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //Se habilita ADC0 en PE3 y PE2
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); //Mientras no se habilite el periférico PORTE
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)); //Mientras no se habilite el periférico ADC0
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2); //Se define PE2 y PE3 que estarán trabajando con el ADC0
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); //Se indica que ADC0 tiene la prioridad máxima y funciona con pulsos generados por el procesador
    //Se configuran ambos canales de ADC0
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 );
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE,1); //Se configura para tomar la primer muestra con ADC
    ADCIntClear(ADC0_BASE, 1); //Se limpia interrupción de muestra 1
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1); //Config. de reloj ADC a 16 Mhz con repetición en cada muestra.
    return;
}

void AA_LEDs(void){

    if(cont_led == 8){
        cont_led = 0;
        op++;
        if(op == 1)
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, 0b1001);
        else if(op == 2){
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, 0b0110);
            op = 0;
        }

    }
    return;
}

unsigned short map(uint32_t x, uint32_t x0, uint32_t x1, //Función para mapeo
                   unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

/*
 * FUNCIONES HANDLER
 */
void PB(void){
    LED1 = '0';
    LED2 = '0';
    LED3 = '0';
    LED4 = '0';
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_4|GPIO_INT_PIN_5); //Se limpian la bandera de interrupción PA4 o PA5
    GPIOIntStatus(GPIO_PORTA_BASE, true); //Se obtiene interrupt status PA con máscara (indica qué boton causó la INT)


       if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_INT_PIN_4) == 0){
           while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_INT_PIN_4) == 0); //Anti-rebote
           flag_bt1 = 1; //Bandera para control de acciones
       }
       else if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_INT_PIN_5) == 0){
           while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_INT_PIN_5) == 0); //Anti-rebote
           flag_bt2 = 1; //Bandera para control de acciones
       }

       if(FSM == 0){
        if(STLED0 == 255)
            LED1 = '1';
        if(STLED1 == 255)
            LED2 = '1';
        if(STLED2 == 255)
            LED3 = '1';
        if(STLED3 == 255)
            LED4 = '1';

        // Se imprime en consola LED1   LED2    LED3    LED4    7SEG
        UARTCharPut(UART0_BASE, LED1);
        UARTCharPut(UART0_BASE, 32); //SPACE
        UARTCharPut(UART0_BASE, LED2);
        UARTCharPut(UART0_BASE, 32); //SPACE
        UARTCharPut(UART0_BASE, LED3);
        UARTCharPut(UART0_BASE, 32); //SPACE
        UARTCharPut(UART0_BASE, LED4);
        UARTCharPut(UART0_BASE, 32); //SPACE
        UARTCharPut(UART0_BASE, ST7SEG);
        UARTCharPut(UART0_BASE, 10); //ENTER
    }
    else if(FSM == 4){
        if(flag_bt1){
            flag_bt1 = 0;
            BT_COUNT++;
            if(BT_COUNT == 16)
                BT_COUNT = 0;
        }
        else if(flag_bt2){
            flag_bt2 = 0;
            BT_COUNT--;
            if(BT_COUNT == 255)
                BT_COUNT = 15;
        }
    }

    return;
}

void UARTint (void){
    UARTIntClear(UART0_BASE, UART_INT_RX); //Clear de bandera de interrupción
    consola = UARTCharGet(UART0_BASE); // Se guarda lo que envía la consola
    UARTCharPut(UART0_BASE, consola); //Se imprime como feedback valor recibido
    UARTCharPut(UART0_BASE, 10); //Enter

    if(FSM == 0){
        if(consola == 'j'){
            STLED0 = ~STLED0;
        }
        else if(consola == 'o'){
            STLED1 = ~STLED1;
        }
        else if(consola == 's'){
            STLED2 = ~STLED2;
        }
        else if(consola == 'e'){
            STLED3 = ~STLED3;
        }
        else if(consola == '0'){
            ST7SEG = '0';
            i = 0;
        }
        else if(consola == '1'){
            ST7SEG = '1';
            i = 1;
        }
        else if(consola == '2'){
            ST7SEG = '2';
            i = 2;
        }
        else if(consola == '3'){
            ST7SEG = '3';
            i = 3;
        }
        else if(consola == '4'){
            ST7SEG = '4';
            i = 4;
        }
        else if(consola == '5'){
            ST7SEG = '5';
            i = 5;
        }
        else if(consola == '6'){
            ST7SEG = '6';
            i = 6;
        }
        else if(consola == '7'){
            ST7SEG = '7';
            i = 7;
        }
        else if(consola == '8'){
            ST7SEG = '8';
            i = 8;
        }
        else if(consola == '9'){
            ST7SEG = '9';
            i = 9;
        }
        else if(consola == 'A'){
            ST7SEG = 'A';
            i = 10;
        }
        else if(consola == 'B'){
            ST7SEG = 'B';
            i = 11;
        }
        else if(consola == 'C'){
            ST7SEG = 'C';
            i = 12;
        }
        else if(consola == 'D'){
            ST7SEG = 'D';
            i = 13;
        }
        else if(consola == 'E'){
            ST7SEG = 'E';
            i = 14;
        }
        else if(consola == 'F'){
            ST7SEG = 'F';
            i = 15;
        }
        else if(consola == '.'){
            ST7SEG = '.';
            i = 16;
        }
    }
    return;
}

void TMR0int(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Reinicio de TMR0
    if(FSM == 4){
        if(TMR0_COUNT != 15) //Si no hay overflow continuar aumento
            TMR0_COUNT++;
        else
            TMR0_COUNT = 0; //Si hubo overflow reiniciar

        //Impresión contadores
        UARTCharPut(UART0_BASE, imp[TMR0_COUNT]);
        UARTCharPut(UART0_BASE, 32); //SPACE
        UARTCharPut(UART0_BASE, 26); //SPACE
        UARTCharPut(UART0_BASE, 32); //SPACE
        UARTCharPut(UART0_BASE, imp[BT_COUNT]);
        UARTCharPut(UART0_BASE, 10); //ENTER

    }
    else if(FSM == 8){
        TMR0_AA++; //Index que permite cambiar array de animación (SNAKE)
        cont_led++; //Index que permite cambiar animación de leds
        if(TMR0_AA == 9) //Al pasarse de lenght se reinicia
            TMR0_AA = 0;
    }
    return;
}

void ADCint(void){
    ADCIntClear(ADC0_BASE, 3); //Clear bandera interrupción
    ADCSequenceDataGet(ADC0_BASE, 3, & ADRES);   // Se obtienen los  datos del módulo y se guarda en ADRES

    return;
}
