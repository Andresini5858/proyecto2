/*
 * File:   main.c
 * Author: Andrés Lemus 21634
 * Proyecto #2
 * Created on November 08, 2022, 7:02 AM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#define _XTAL_FREQ 1000000
#define tmr0_val 236 //valor del timer0 para un período de 20ms

char selector;
char bandera;
int pot;

void setup(void);
void setupPWM(void);
void setupADC(void);
void setupUART(void);
void setupEEPROM(void);
void delay(unsigned int micro); //función para obtener delay variable
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax){ //función para mapear valores
    return ((value - inputmin)*(outmax-outmin)) / (inputmax-inputmin)+outmin;} 


//VECTOR DE INTERRUPCIONES
void __interrupt() isr(void){
    if (INTCONbits.RBIF == 1){
        if (PORTBbits.RB7 == 0){
            bandera = 1;}
        if (PORTBbits.RB7 == 0 && bandera == 1){
            selector++;}
            if (selector == 4){
                selector = 1;}
            INTCONbits.RBIF = 0;   
    }

    if (PIR1bits.ADIF == 1){ //verificar bandera del conversor ADC
        if (ADCON0bits.CHS == 0b0000){ 
            CCPR1L = map(ADRESH, 1, 255, 3, 20); //mapear valores para el servomotor 1
            ADCON0bits.CHS = 0b0001;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0001){
            CCPR2L = map(ADRESH, 1, 255, 3, 20); //mapear valores para el servomotor 2
            ADCON0bits.CHS = 0b0010;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0010){
            pot = map(ADRESH, 0, 255, 0, 80); //mapear valores para intensidad del led
            ADCON0bits.CHS = 0b0011;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0011){
            pot = map(ADRESH, 0, 255, 0, 80); //mapear valores para intensidad del led
            ADCON0bits.CHS = 0b0000;} //cambio de canal
            PIR1bits.ADIF = 0;} //limpiar bandera}
    
    if (INTCONbits.T0IF == 1){ //chequear interrupción del Timer0
        INTCONbits.T0IF = 0; // limpiar bandera
        TMR0 = tmr0_val; //asignar valor al timer0
        PORTAbits.RA0 = 1; //encender led
        delay(pot); // delay (tiempo en alto del pulso)
        PORTAbits.RA0 = 0; //apagar
    }
   
}

void main(void) {
    setup();
    setupADC();
    setupPWM();
    TMR0 = tmr0_val; //asignar valor al timer 0
    selector = 1;
    while(1){
        if (selector == 1){
            PORTDbits.RD0 = 1;
            if (ADCON0bits.GO == 0){ //Chequear si la conversión ya termino
            ADCON0bits.GO = 1;} // Iniciar Conversión
        }
        if (selector == 2){
            
        }
    }
}

void setup(void){
    ANSEL = 0b00000111; // puertos digitales
    ANSELH = 0;
    TRISBbits.TRISB2 = 1; //puerto B7 como entrada
    TRISBbits.TRISB1 = 1; //puerto B6 como entrada
    TRISBbits.TRISB0 = 1; //puerto B0 como entrada
    TRISAbits.TRISA0 = 0; //puero A0 como salida
    TRISD = 0;
    PORTD = 0;
    PORTB = 0; // limpiar puerto B
    PORTA = 0; // Limpiar Puerto A
    
    INTCONbits.GIE = 1; //Activar interrupciones globales
    INTCONbits.PEIE = 1; //Activar interrupciones periféricas
    INTCONbits.T0IE = 0; //Activar interrupciones del timer0
    INTCONbits.T0IF = 0; //Limpiar bandera de interrupcion del Timer0
    PIE1bits.ADIE = 1; // Habiliar interrupcion del conversor ADC
    PIR1bits.ADIF = 0; // Limpiar bandera de interrupción del ADC

    OSCCONbits.IRCF2 = 1; //Oscilador a 1MHz
    OSCCONbits.IRCF1 = 0;
    OSCCONbits.IRCF0 = 0;
    OSCCONbits.SCS = 1; //Oscialdor interno
    
    OPTION_REGbits.T0CS = 0; //Usar Timer0 con Fosc/4
    OPTION_REGbits.PSA = 0; //Prescaler con el Timer0
    OPTION_REGbits.PS2 = 1; //Prescaler de 256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1; 
}

void setupADC(void){
    ADCON0bits.ADCS1 = 0; // Fosc/2        
    ADCON0bits.ADCS0 = 0; // =======      
    
    ADCON1bits.VCFG1 = 0; // Referencia VSS (0 Volts)
    ADCON1bits.VCFG0 = 0; // Referencia VDD (5 Volts)
    
    ADCON1bits.ADFM = 0;  // Justificado hacia izquierda
    
    ADCON0bits.CHS3 = 0; // Canal AN12
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;        
    
    ADCON0bits.ADON = 1; // Habilitamos el ADC
    __delay_us(100); //delay de 100 us
}

void setupPWM(void){
    //CCP1
    TRISCbits.TRISC2 = 1; //se pone CCP1 com entrada
    PR2 = 155; //Período de 20ms
    CCP1CON = 0b00001100; // P1A como PWM 
    CCP1CONbits.DC1B = 0b11; //bis menos significativos para el tiempo en alto
    CCPR1L = 11;  //valor asignado para oscilar para empezar en 90
    PIR1bits.TMR2IF = 0; //limpiar bandera de overflow del Tiemr2
    T2CONbits.T2CKPS1 = 1; //Prescaler de 16 bits
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1; //Habiliar Timer2
            
   //CP2
   //TRISCbits.TRISC1 = 1; //se pone CCP2 com entrada  
   //CCP2CON = 0b00001100; //Modo PWM
   //CCP2CONbits.DC2B0 = 1; //bits menos significativos para el tiempo en alto
   //CCP2CONbits.DC2B1 = 1;
   //CCPR2L = 11; //valor asignado para oscilar para empezar en 0
   
   while (PIR1bits.TMR2IF == 0); //No hacer nada hasta que haya interrupcion
    PIR1bits.TMR2IF = 0; //limpiar bandera de overflow del Tiemr2
    TRISCbits.TRISC2 = 0; //Poner C2 como salida PWM
    //TRISCbits.TRISC1 = 0; //Poncer C1 como salida PWM
}

void setupUART(void){
    // Paso 1: configurar velocidad baud rate
    
    SPBRG = 2; //valor para 9600 de baud rate
    
    // Paso 2:
    
    TXSTAbits.SYNC = 0;         // Modo Asíncrono
    RCSTAbits.SPEN = 1;         // Habilitar UART
    
    // Paso 3:
    // Usar 8 bits
    
    // Paso 4:
    TXSTAbits.TXEN = 1;         // Habilitamos la transmision
    PIR1bits.TXIF = 0;
    RCSTAbits.CREN = 1;         // Habilitamos la recepcion
}

void setupEEPROM(void){
    
}

//FUNCION DE DELAY VARIABLES
void delay(unsigned int micro){
    while (micro > 0){
        __delay_us(980); //delay de 0.25ms
        micro--; //decrementar variable
    }
}

//Funcion para mostrar texto
void cadena(char *cursor){
    while (*cursor != '\0'){//mientras el cursor sea diferente a nulo
        while (PIR1bits.TXIF == 0); //mientras que se este enviando no hacer nada
            TXREG = *cursor; //asignar el valor del cursor para enviar
            *cursor++;//aumentar posicion del cursor
    }
}

