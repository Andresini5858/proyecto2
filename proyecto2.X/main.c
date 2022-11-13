/*
 * File:   main.c
 * Author: Andrés Lemus
 * Laboratorio #5 PWM
 * Created on October 17, 2022, 5:26 PM
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
#include <stdint.h>

#define _XTAL_FREQ 500000 //frecuencia de 500 kHZ
#define tmr0_val 246 //valor del timer0 para un período de 20ms

unsigned char x=0;
unsigned char y=0;
unsigned int selector = 0;
unsigned int bandera = 0;
unsigned int loop = 0;
unsigned int pot; //valor para tiempo en alto de PWM para intensidad del led
unsigned int pot1; //valor para tiempo en alto de PWM para intensidad del led
unsigned char dato;
unsigned char servo[9] = {7,8,9,10,11,12,13,14,15};

void setup(void); //función de configuración
void setupADC(void); //función de configuración del ADC
void setupPWM(void); //función de configuración del PWM
void setupUART(void); //función de UART
unsigned char readEEPROM(void);
void writeEEPROM(unsigned char data);
void interrup(void);
void cadena(char *cursor);
void delay(unsigned int micro); //función para obtener delay variable
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax){ //función para mapear valores
    return ((value - inputmin)*(outmax-outmin)) / (inputmax-inputmin)+outmin;} 

//VECTOR DE INTERRUPCIONES
void __interrupt() isr(void){
    if (INTCONbits.RBIF == 1){
        INTCONbits.RBIF = 0;
        if (PORTBbits.RB7 == 0){
            bandera = 1;}
        if (PORTBbits.RB7 == 1 && bandera == 1){
            selector++;
            loop = 0;
            bandera = 0;
            if (selector == 3){
                selector = 0;}
        }               
    }
    
    if (PIR1bits.ADIF == 1){ //verificar bandera del conversor ADC
        if (ADCON0bits.CHS == 0b0000){ 
            CCPR1L = map(ADRESH, 0, 255, 7, 15); //mapear valores para el servomotor 1
            ADCON0bits.CHS = 0b0001;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0001){
            CCPR2L = map(ADRESH, 0, 255, 7, 15); //mapear valores para el servomotor 2
            ADCON0bits.CHS = 0b0010;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0010){
            pot = map(ADRESH, 0, 255, 5, 14); //mapear valores para intensidad del led
            ADCON0bits.CHS = 0b0011;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0011){
            pot1 = map(ADRESH, 0, 255, 5, 14); //mapear valores para intensidad del led
            ADCON0bits.CHS = 0b0000;} //cambio de canal
            PIR1bits.ADIF = 0;} //limpiar bandera}
    
    if (INTCONbits.T0IF == 1){ //chequear interrupción del Timer0
        INTCONbits.T0IF = 0; // limpiar bandera
        TMR0 = tmr0_val; //asignar valor al timer0
        PORTCbits.RC0 = 1; //encender led
        delay(pot); // delay (tiempo en alto del pulso)
        PORTCbits.RC0 = 0; //apagar
        PORTCbits.RC3 = 1; //encender led
        delay(pot1); // delay (tiempo en alto del pulso)
        PORTCbits.RC3 = 0; //apagar
        
    }
    
    if (PIR1bits.RCIF == 1){
        if (RCREG == 'd'){
            if (x == 9){
                x = 8;}
            CCPR1L = servo[x];
            x++;
            PIR1bits.RCIF = 0;
        }
        if (RCREG == 'a'){
            if (x == 255){
                x = 0;}
            CCPR1L = servo[x];
            x--;
            PIR1bits.RCIF = 0;
        }
        if (RCREG == 'w'){
            if (y == 9){
                y = 8;}
            CCPR2L = servo[y];
            y++;
            PIR1bits.RCIF = 0;
        }
        if (RCREG == 's'){
            if (y == 255){
                y = 0;}
            CCPR2L = servo[y];
            y--;
            PIR1bits.RCIF = 0;
        }
    }
}

//LOOP PRINCIPAL
void main(void) { 
    setup(); //Llamar al setup
    setupADC(); //Llamar a la configuración del ADC
    setupPWM(); //Llamar a la configuración del PWM
    setupUART(); //Llamar función de UART
    TMR0 = tmr0_val; //asignar valor al timer 0
    cadena("\n\r---------------------------------PARA CONTROLAR CON LA COMPUTADORA ELEGIR EL MODO 3---------------------------------\n\r");
    while (1){
        if (selector == 0){
            loop = 1;
            while (loop == 1){
                if (ADCON0bits.GO == 0){ //Chequear si la conversión ya termino
                    ADCON0bits.GO = 1;}
                PORTDbits.RD5 = 1;    
                PORTDbits.RD6 = 0;
                PORTDbits.RD7 = 0;
                if (PORTBbits.RB6 == 0){
                    bandera = 2;}
                if (PORTBbits.RB6 == 1 && bandera == 2){
                    EEADR = 0b00000000;
                    writeEEPROM(CCPR1L);
                    __delay_us(40);
                
                    EEADR = 0b00000001;
                    writeEEPROM(CCPR2L);
                    __delay_us(40);
                   
                    EEADR = 0b00000010;
                    writeEEPROM(pot);
                    __delay_us(40);
                   
                    EEADR = 0b00000011;
                    writeEEPROM(pot1);
                    __delay_us(40);
                    
                    bandera = 0;}
                
                if (PORTBbits.RB5 == 0){
                    bandera = 3;}
                if (PORTBbits.RB5 == 1 && bandera == 3){
                    EEADR = 0b00000100;
                    writeEEPROM(CCPR1L);
                    __delay_us(40);
                
                    EEADR = 0b00000101;
                    writeEEPROM(CCPR2L);
                    __delay_us(40);
                   
                    EEADR = 0b00000110;
                    writeEEPROM(pot);
                    __delay_us(40);
                   
                    EEADR = 0b00000111;
                    writeEEPROM(pot1);
                    __delay_us(40);
                    
                    bandera = 0;}
            
            }}
        
        if (selector == 1){
            interrup();
            loop = 1;
            while (loop == 1){
                PORTDbits.RD5 = 0;
                PORTDbits.RD6 = 1;
                PORTDbits.RD7 = 0;
            if (PORTBbits.RB6 == 0){
                bandera = 2;}
            if (PORTBbits.RB6 == 1 && bandera == 2){
                EEADR = 0b00000000;
                readEEPROM();
                CCPR1L = dato; 
                __delay_us(40);
                
                EEADR = 0b00000001;
                readEEPROM();
                CCPR2L = dato; 
                __delay_us(40);
                
                EEADR = 0b00000010;
                readEEPROM();
                pot = dato; 
                __delay_us(40);
                
                EEADR = 0b00000011;
                readEEPROM();
                pot1 = dato; 
                __delay_us(40);
                
                bandera = 0;}
                
            if (PORTBbits.RB5 == 0){
            bandera = 3;}
            if (PORTBbits.RB5 == 1 && bandera == 3){
                EEADR = 0b00000100;
                readEEPROM();
                CCPR1L = dato; 
                __delay_us(40);
                
                EEADR = 0b00000101;
                readEEPROM();
                CCPR2L = dato; 
                __delay_us(40);
                
                EEADR = 0b00000110;
                readEEPROM();
                pot = dato; 
                __delay_us(40);
                
                EEADR = 0b00000111;
                readEEPROM();
                pot1 = dato; 
                __delay_us(40);
                
                bandera = 0;}
            
            }}
        
        if (selector == 2){
            loop = 1;
            while (loop == 1){
            PIE1bits.RCIE =  1;    
            PORTDbits.RD5 = 0;    
            PORTDbits.RD6 = 0;
            PORTDbits.RD7 = 1;
            }} 
    }   
}

void setup(void){
    ANSELH = 0; // Puertos digitales
    ANSELbits.ANS0 = 1; //puerto RA0 como analógico
    ANSELbits.ANS1 = 1; //puerto RA1 como analógico
    ANSELbits.ANS2 = 1; //puerto RA2 como analógico
    ANSELbits.ANS3 = 1; //puerto RA3 como analógico
    TRISBbits.TRISB7 = 1; //puerto B7 como entrada
    TRISBbits.TRISB6 = 1; //puerto B6 como entrada
    TRISBbits.TRISB5 = 1; //puerto B5 como entrada
    TRISBbits.TRISB4 = 1; //puerto B4 como entrada
    TRISBbits.TRISB3 = 1; //puerto B3 como entrada
    TRISCbits.TRISC0 = 0; //puerto C0 como salida
    TRISCbits.TRISC3 = 0; //puerto C3 como salida
    TRISE = 0;
    TRISD = 0; // Puerto D como salida
    PORTA = 0; // limpiar puerto A
    PORTB = 0; // Limpiar puerto B
    PORTD = 0; // Limpiar puerto D
    PORTE = 0; 
    
    INTCONbits.GIE = 1; //Activar interrupciones globales
    INTCONbits.PEIE = 1; //Activar interrupciones periféricas
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    INTCONbits.T0IE = 1; //Activar interrupciones del timer0
    INTCONbits.T0IF = 0; //Limpiar bandera de interrupcion del Timer0
    PIE1bits.ADIE = 1; // Habiliar interrupcion del conversor ADC
    PIR1bits.ADIF = 0; // Limpiar bandera de interrupción del ADC
    PIE1bits.RCIE =  0;

    OSCCONbits.IRCF2 = 0; //Oscilador a 500kHz
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1; //Oscialdor interno
    
    OPTION_REGbits.nRBPU = 0;
    OPTION_REGbits.T0CS = 0; //Usar Timer0 con Fosc/4
    OPTION_REGbits.PSA = 0; //Prescaler con el Timer0
    OPTION_REGbits.PS2 = 1; //Prescaler de 256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    
    WPUBbits.WPUB7 = 1;
    WPUBbits.WPUB6 = 1;
    WPUBbits.WPUB5 = 1;
    WPUBbits.WPUB4 = 1;
    WPUBbits.WPUB3 = 1;
    
    IOCBbits.IOCB7 = 1;
    IOCBbits.IOCB6 = 0;
    IOCBbits.IOCB5 = 0;
    IOCBbits.IOCB4 = 0;
    IOCBbits.IOCB3 = 0;  
    
    EEADRH = 0;
}

//ADC
void setupADC(void){
    ADCON0bits.ADCS1 = 0; // Fosc/2        
    ADCON0bits.ADCS0 = 0; // =======      
    
    ADCON1bits.VCFG1 = 0; // Referencia VSS (0 Volts)
    ADCON1bits.VCFG0 = 0; // Referencia VDD (3.3 Volts)
    
    ADCON1bits.ADFM = 0;  // Justificado hacia izquierda
    
    ADCON0bits.CHS3 = 0; // Canal AN12
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;        
    
    ADCON0bits.ADON = 1; // Habilitamos el ADC
    __delay_us(100); //delay de 100 us
}

//PWM
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
   TRISCbits.TRISC1 = 1; //se pone CCP2 com entrada  
   CCP2CON = 0b00001100; //Modo PWM
   CCP2CONbits.DC2B0 = 1; //bis menos significativos para el tiempo en alto
   CCP2CONbits.DC2B1 = 1;
   CCPR2L = 11; //valor asignado para oscilar para empezar en 0
   
   while (PIR1bits.TMR2IF == 0); //No hacer nada hasta que haya interrupcion
    PIR1bits.TMR2IF = 0; //limpiar bandera de overflow del Tiemr2
    TRISCbits.TRISC2 = 0; //Poner C2 como salida PWM
    TRISCbits.TRISC1 = 0; //Poncer C1 como salida PWM
}

void setupUART(void){
    // Paso 1: configurar velocidad baud rate
    BAUDCTLbits.BRG16 = 1;
    TXSTAbits.BRGH = 1;
    SPBRGH = 0;
    SPBRG = 12; //valor para 9600 de baud rate
    
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

unsigned char readEEPROM(void){
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    dato = EEDATA;
    return dato;
}

void writeEEPROM(unsigned char data){
    EEDATA = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
                
    INTCONbits.GIE = 0;
    while (INTCONbits.GIE == 1);
    __delay_us(50);
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    interrup();
    __delay_us(50);
    
    while (EECON1bits.WR == 1);
    EECON1bits.WREN = 0;
}

void interrup(void){
    INTCONbits.GIE = 1; //Activar interrupciones globales
    INTCONbits.PEIE = 1; //Activar interrupciones periféricas
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    INTCONbits.T0IE = 1; //Activar interrupciones del timer0
    INTCONbits.T0IF = 0; //Limpiar bandera de interrupcion del Timer0
    PIE1bits.ADIE = 1; // Habiliar interrupcion del conversor ADC
    PIR1bits.ADIF = 0; // Limpiar bandera de interrupción del ADC
}

//FUNCION DE DELAY VARIABLES
void delay(unsigned int micro){
    while (micro > 0){
        __delay_us(50); //delay de 0.25ms
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