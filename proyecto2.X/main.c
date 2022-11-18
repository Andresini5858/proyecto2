/*
 * File:   main.c
 * Author: Andrés Lemus
 * Laboratorio #5 PWM
 * Created on November 9, 2022, 7:02 AM
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _XTAL_FREQ 500000 //frecuencia de 500 kHZ
#define tmr0_val 246 //valor del timer0 para un período de 20ms

unsigned char x=0; //variable para posicion del servo1
unsigned char y=0; //variable para posicion del servo2
unsigned char x1=0; //variable para posicion del servo3
unsigned char y1=0; //variable para posicion del servo4
unsigned char z=0; //variable para selector del servo
unsigned int selector = 0; //variable para selector de modo
unsigned int bandera = 0; //variable para bandera de pushbuttons
unsigned int loop = 0; //variable para mantener loop
unsigned int pot; //valor para tiempo en alto de PWM para intensidad del led
unsigned int pot1; //valor para tiempo en alto de PWM para intensidad del led
unsigned char dato; //variable para dato a leer de la EEPROM
unsigned char servo[18] = {3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}; //arreglo para posicionar servos del CCP
unsigned char servo2[17] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17}; //arreglo para posicionar servos del PWM

void setup(void); //función de configuración
void setupADC(void); //función de configuración del ADC
void setupPWM(void); //función de configuración del PWM
void setupUART(void); //función de UART
unsigned char readEEPROM(void); //funcio de leer de la EEPROM
void writeEEPROM(unsigned char data); //funcion para escribir de la EEPROM
void cadena(char *cursor); //funcion para escribir texto a la terminal
void delay(unsigned int micro); //función para obtener delay variable
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax){ //función para mapear valores
    return ((value - inputmin)*(outmax-outmin)) / (inputmax-inputmin)+outmin;} 

//VECTOR DE INTERRUPCIONES
void __interrupt() isr(void){ 
    if (INTCONbits.RBIF == 1){ // Chequear interrupcion del Receptor 
        INTCONbits.RBIF = 0; //Limpiar bandera
        if (PORTBbits.RB7 == 0){ //Verificar si se presiono el boton RB7 
            bandera = 1;} //poner bandera en 1
        if (PORTBbits.RB7 == 1 && bandera == 1){ //chequear si se dejo de presionar el boton
            selector++; //aumentar selector
            loop = 0; //salir del loop para reiniciar
            bandera = 0; //limpiar bandera
            if (selector == 3){ // si el selector es 3, regresarlo a 0
                selector = 0;}
        }               
    }
    
    if (PIR1bits.ADIF == 1){ //verificar bandera del conversor ADC
        if (ADCON0bits.CHS == 0b0000){ 
            CCPR1L = map(ADRESH, 0, 255, 3, 20); //mapear valores para el servomotor 1
            ADCON0bits.CHS = 0b0001;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0001){
            CCPR2L = map(ADRESH, 0, 255, 3, 20); //mapear valores para el servomotor 2
            ADCON0bits.CHS = 0b0010;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0010){
            pot = map(ADRESH, 0, 255, 1, 17); //mapear valores para servomotor 3
            ADCON0bits.CHS = 0b0011;} //cambio de canal
        
        else if (ADCON0bits.CHS == 0b0011){
            pot1 = map(ADRESH, 0, 255, 1, 17); //mapear valores para servomotor 4
            ADCON0bits.CHS = 0b0000;} //cambio de canal
            PIR1bits.ADIF = 0;} //limpiar bandera}
    
    if (INTCONbits.T0IF == 1){ //chequear interrupción del Timer0
        INTCONbits.T0IF = 0; // limpiar bandera
        TMR0 = tmr0_val; //asignar valor al timer0
        PORTCbits.RC0 = 1; //encender puerto
        delay(pot); // delay (tiempo en alto del pulso)
        PORTCbits.RC0 = 0; //apagar
        PORTCbits.RC3 = 1; //encender puerto
        delay(pot1); // delay (tiempo en alto del pulso)
        PORTCbits.RC3 = 0; //apagar    
    }
    
    if (PIR1bits.RCIF == 1){
        if (RCREG == '3'){ //revisar si se recibio un 3
            CCPR1L = 3; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == '4'){ //revisar si se recibio un 4
            CCPR1L = 4; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == '5'){ //revisar si se recibio un 5
            CCPR1L = 5; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == '6'){ //revisar si se recibio un 6
            CCPR1L = 6; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == '7'){ //revisar si se recibio un 7
            CCPR1L = 7; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == '8'){ //revisar si se recibio un 8
            CCPR1L = 8; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == '9'){ //revisar si se recibio un 9
            CCPR1L = 9; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'A'){ //revisar si se recibio un A
            CCPR1L = 10; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'B'){ //revisar si se recibio un B
            CCPR1L = 11; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'C'){ //revisar si se recibio un C
            CCPR1L = 12; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'D'){ //revisar si se recibio un D
            CCPR1L = 13; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'E'){ //revisar si se recibio un E
            CCPR1L = 14; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        } 
        if (RCREG == 'F'){ //revisar si se recibio un F
            CCPR1L = 15; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'G'){ //revisar si se recibio un G
            CCPR1L = 16; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'H'){ //revisar si se recibio un H
            CCPR1L = 17; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'J'){ //revisar si se recibio un J
            CCPR1L = 18; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'K'){ //revisar si se recibio un K
            CCPR1L = 19; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'L'){ //revisar si se recibio un L
            CCPR1L = 20; //mandar valor al motor deseado
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        
        if (RCREG == 'z'){ //revisar si se recibio una z
            selector = 0; //elegir modo 0
            loop = 0; //sacar del loop
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'x'){ //revisar si se recibio un x
            selector = 1; //elegir modo 1
            loop = 0; //sacar del loop
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'c'){ ////revisar si se recibio un c
            selector = 2; //elegir modo 2
            loop = 0; //sacar del loop
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'd'){ //revisar si se recibio un d
            if (x == 255){ //si la posicion del servo es 255 
                x = 0;} //igual a cero
            if (x == 17){ //si llego al máximo regresar uno
                x = 16;} //dejar en el máximo
            CCPR1L = servo[x]; //mandar posicion
            x++; //aumentar la posicion
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'a'){ //revisar si se recibio un a
            if (x == 255){ //si la posicion del servo es 255 
                x = 0;} //igual a cero
            CCPR1L = servo[x]; //mandar posicion
            x--; //decrementar la posicion
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'w'){ //revisar si se recibio un w
            if (y == 255){ //si la posicion del servo es 255 
                y = 0;} //igual a cero
            if (y == 17){ //si llego al máximo regresar uno
                y = 16;} //dejar en el máximo
            CCPR2L = servo[y]; //mandar posicion al servo
            y++; //aumentar la posicion
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 's'){ //revisar si se recibio un s
            if (y == 255){ //si la posicion del servo es 255 
                y = 0;} //igual a cero
            CCPR2L = servo[y]; //mandar posicion al servo
            y--; //decrementar la posicion
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        
        if (RCREG == 'i'){ //revisar si se recibio un i
            if (x1 == 255){  //si la posicion del servo es 255 
                x1 = 0;} //igual a cero
            if (x1 == 16){ //si llego al máximo regresar uno
                x1 = 15;} //dejar en el máximo
            pot = servo2[x1]; //mandar posicion al servo
            x1++; //aumentar la posicion
            PIR1bits.RCIF = 0; //limpiar
        }
        if (RCREG == 'k'){ //revisar si se recibio un k
            if (x1 == 255){ //si la posicion del servo es 255 
                x1 = 0;} //igual a cero
            pot = servo2[x1]; //mandar posicion al servo
            x1--; //decrementar la posicion
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        
        if (RCREG == 'l'){ //revisar si se recibio un l
            if (y1 == 255){ //si la posicion del servo es 255
                y1 = 0;} //igual a cero
            if (y1 == 16){ //si llego al máximo regresar uno
                y1 = 15;} //dejar en el máximo
            pot1 = servo2[y1];  //mandar posicion al servo
            y1++; //aumentar la posicion
            PIR1bits.RCIF = 0; //limpiar bandera
        }
        if (RCREG == 'j'){ //revisar si se recibio un j
            if (y1 == 255){ //si la posicion del servo es 255
                y1 = 0;} //igual a cero
            pot1 = servo2[y1]; //mandar posicion al servo
            y1--; //decrementar la posicion
            PIR1bits.RCIF = 0; //limpiar bandera
        } 
        
        
    }}

//LOOP PRINCIPAL
void main(void) { 
    setup(); //Llamar al setup
    setupADC(); //Llamar a la configuración del ADC
    setupPWM(); //Llamar a la configuración del PWM
    setupUART(); //Llamar función de UART
    TMR0 = tmr0_val; //asignar valor al timer 0
    cadena("\n\r---------------------------------PARA CONTROLAR CON LA COMPUTADORA ELEGIR EL MODO 3---------------------------------\n\r");
    while (1){ //LOOP PRINCIPAL
        if (selector == 0){ //Iniciar en modo 1
            loop = 1; //dejar en loop modo 1
            PORTDbits.RD5 = 1; //encender led indicador
            PORTDbits.RD6 = 0; //apagar led
            PORTDbits.RD7 = 0; //apagar led
            while (loop == 1){ 
                if (ADCON0bits.GO == 0){ //Chequear si la conversión ya termino
                    ADCON0bits.GO = 1;} //iniciar conversion
                if (PORTBbits.RB6 == 0){ //revisar si se presiono RB6
                    bandera = 2;} //poner bandera 
                if (PORTBbits.RB6 == 1 && bandera == 2){ //revisar si se dejo de presionar 
                    EEADR = 0b00000000; //indicar direccion en data memory
                    writeEEPROM(CCPR1L); // funcion de escribir en la EEPROM
                
                    EEADR = 0b00000001; //indicar direccion en data memory
                    writeEEPROM(CCPR2L); // funcion de escribir en la EEPROM
                   
                    EEADR = 0b00000010; //indicar direccion en data memory
                    writeEEPROM(pot); // funcion de escribir en la EEPROM
                   
                    EEADR = 0b00000011; //indicar direccion en data memory
                    writeEEPROM(pot1); // funcion de escribir en la EEPROM
                    
                    bandera = 0;} //limpiar bandera
                
                if (PORTBbits.RB5 == 0){ //revisar si se presiono el boton RB5 
                    bandera = 3;} //encender bandera
                if (PORTBbits.RB5 == 1 && bandera == 3){ //revisar si se dejo de presionar
                    EEADR = 0b00000100; //indicar direccion en data memory
                    writeEEPROM(CCPR1L); // funcion de escribir en la EEPROM
                
                    EEADR = 0b00000101; //indicar direccion en data memory
                    writeEEPROM(CCPR2L); // funcion de escribir en la EEPROM
                   
                    EEADR = 0b00000110; //indicar direccion en data memory
                    writeEEPROM(pot); // funcion de escribir en la EEPROM
                   
                    EEADR = 0b00000111; //indicar direccion en data memory
                    writeEEPROM(pot1); // funcion de escribir en la EEPROM
                    
                    bandera = 0;} //limpiar bandera
                
                if (PORTBbits.RB4 == 0){ //revisar si se presiono boton RB4
                    bandera = 4;} //encender bandera
                if (PORTBbits.RB4 == 1 && bandera == 4){ //revisar si se dejo de presionar el boton
                    EEADR = 0b00001000; //indicar direccion en data memory
                    writeEEPROM(CCPR1L); // funcion de escribir en la EEPROM
                
                    EEADR = 0b00001001; //indicar direccion en data memory
                    writeEEPROM(CCPR2L);// funcion de escribir en la EEPROM
                   
                    EEADR = 0b00001010; //indicar direccion en data memory
                    writeEEPROM(pot); // funcion de escribir en la EEPROM
                   
                    EEADR = 0b00001011; //indicar direccion en data memory
                    writeEEPROM(pot1); // funcion de escribir en la EEPROM
                    
                    bandera = 0;} //limpiar bandera
                
                if (PORTBbits.RB3 == 0){ //revisar bandera
                    bandera = 5;} //encender bandera 
                if (PORTBbits.RB3 == 1 && bandera == 5){ //revisar si se apago la bandera
                    EEADR = 0b00001100; // //indicar direccion en data memory
                    writeEEPROM(CCPR1L); // funcion escribir en la EEPROM
                
                    EEADR = 0b00001101; //indicar direccion en data memory
                    writeEEPROM(CCPR2L); // funcion escribir en la EEPROM
                   
                    EEADR = 0b00001110; //indicar direccion en data memory
                    writeEEPROM(pot); // funcion escribir en la EEPROM
                   
                    EEADR = 0b00001111; //indicar direccion en data memory
                    writeEEPROM(pot1); // funcion escribir en la EEPROM
                    
                    bandera = 0;} //limpiar bandera
            }}
        
        if (selector == 1){
            loop = 1; //manterner modo en loop
            PORTDbits.RD5 = 0; //apagar led
            PORTDbits.RD6 = 1; //encender led
            PORTDbits.RD7 = 0; //apagar led
            while (loop == 1){ 
            if (PORTBbits.RB6 == 0){ //revisar si se presiono el boton
                bandera = 2;} //encender bandera
            if (PORTBbits.RB6 == 1 && bandera == 2){ //revisar si se dejo de presionar
                EEADR = 0b00000000;  //indicar direccion en data memory
                CCPR1L = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00000001; //indicar direccion en data memory
                CCPR2L = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00000010; //indicar direccion en data memory
                pot = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00000011; //indicar direccion en data memory
                pot1 = readEEPROM(); // funcion de leer en la EEPROM
                
                bandera = 0;} //limpiar bandera
                
            if (PORTBbits.RB5 == 0){ //revisar si se presiono el boton
            bandera = 3;} //encender bandera
            if (PORTBbits.RB5 == 1 && bandera == 3){ //revisar si se dejo de presionar
                EEADR = 0b00000100; //indicar direccion en data memory
                CCPR1L = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00000101; //indicar direccion en data memory
                CCPR2L = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00000110; //indicar direccion en data memory
                pot = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00000111; //indicar direccion en data memory
                pot1 = readEEPROM(); // funcion de leer en la EEPROM
                
                bandera = 0;} //limpiar bandera
                
            if (PORTBbits.RB4 == 0){ //revisar si se presiono el boton
            bandera = 4;} //encender bandera
            if (PORTBbits.RB4 == 1 && bandera == 4){ //revisar si se dejo de presionar
                EEADR = 0b00001000; //indicar direccion en data memory
                CCPR1L = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00001001; //indicar direccion en data memory
                CCPR2L = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00001010; //indicar direccion en data memory
                pot = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00001011; //indicar direccion en data memory
                pot1 = readEEPROM();  // funcion de leer en la EEPROM
                
                bandera = 0;} //limpiar bandera
                
            if (PORTBbits.RB3 == 0){ //revisar si se presiono el boton
            bandera = 5;} //encender bandera
            if (PORTBbits.RB3 == 1 && bandera == 5){ //revisar si se dejo de presionar
                EEADR = 0b00001100; //indicar direccion en data memory
                CCPR1L = readEEPROM();  // funcion de leer en la EEPROM
                
                EEADR = 0b00001101; //indicar direccion en data memory
                CCPR2L = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00001110; //indicar direccion en data memory
                pot = readEEPROM(); // funcion de leer en la EEPROM
                
                EEADR = 0b00001111; //indicar direccion en data memory
                pot1 = readEEPROM();  // funcion de leer en la EEPROM
                
                bandera = 0;} //limpiar bandera
            
            }}
        
        if (selector == 2){
            loop = 1;
            PORTDbits.RD5 = 0; //apagar led   
            PORTDbits.RD6 = 0; //apagar led
            PORTDbits.RD7 = 1; //encender led
            while (loop == 1){
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
    TRISD = 0; // Puerto D como salida
    PORTA = 0; // limpiar puerto A
    PORTB = 0; // Limpiar puerto B
    PORTD = 0; // Limpiar puerto D
    
    INTCONbits.GIE = 1; //Activar interrupciones globales
    INTCONbits.PEIE = 1; //Activar interrupciones periféricas
    INTCONbits.RBIE = 1; //Activar interrupciones del puerto B
    INTCONbits.RBIF = 0; //Limpiar bandera de interrupcion del puerto B
    INTCONbits.T0IE = 1; //Activar interrupciones del timer0
    INTCONbits.T0IF = 0; //Limpiar bandera de interrupcion del Timer0
    PIE1bits.ADIE = 1; // Habiliar interrupcion del conversor ADC
    PIR1bits.ADIF = 0; // Limpiar bandera de interrupción del ADC
    PIE1bits.RCIE =  1; //Encender interrupciones del RECEPTOR UART
    PIR1bits.RCIF =  0; //Limpiar bandera del receptor UART

    OSCCONbits.IRCF2 = 0; //Oscilador a 500kHz
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1; //Oscialdor interno
    
    OPTION_REGbits.nRBPU = 0; //Activar Pull-ups
    OPTION_REGbits.T0CS = 0; //Usar Timer0 con Fosc/4
    OPTION_REGbits.PSA = 0; //Prescaler con el Timer0
    OPTION_REGbits.PS2 = 1; //Prescaler de 256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    
    WPUBbits.WPUB7 = 1; //Pull-up para RB7
    WPUBbits.WPUB6 = 1; //Pull-up para RB6
    WPUBbits.WPUB5 = 1; //Pull-up para RB5
    WPUBbits.WPUB4 = 1; //Pull-up para RB4
    WPUBbits.WPUB3 = 1; //Pull-up para RB3
    
    IOCBbits.IOCB7 = 1; //Interrupcion para RB7
    IOCBbits.IOCB6 = 0; //Desactivar interrupcion RB6
    IOCBbits.IOCB5 = 0; //Desactivar interrupcion RB5
    IOCBbits.IOCB4 = 0; //Desactivar interrupcion RB4
    IOCBbits.IOCB3 = 0; //Desactivar interrupcion RB3  
    
    EEADRH = 0;
}

//ADC
void setupADC(void){
    ADCON0bits.ADCS1 = 0; // Fosc/2        
    ADCON0bits.ADCS0 = 0; // =======      
    
    ADCON1bits.VCFG1 = 0; // Referencia VSS (0 Volts)
    ADCON1bits.VCFG0 = 0; // Referencia VDD (3.3 Volts)
    
    ADCON1bits.ADFM = 0;  // Justificado hacia izquierda
    
    ADCON0bits.CHS3 = 0; // Canal AN0
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
    while(WR||RD); //REVISAR banderas
    EECON1bits.EEPGD = 0; //Acceder data memory
    EECON1bits.RD = 1; //iniciar lectura
    dato = EEDATA; //pasar EEDATA a variable
    return dato; //retornar dato
}

void writeEEPROM(unsigned char data){
    
    while (WR); //revisar que no haya escritura
    EEDAT = data; //asignar valor a escribir
    EECON1bits.EEPGD = 0; //acceder a data memory
    EECON1bits.WREN = 1; //habiliatar escritura
                
    INTCONbits.GIE = 0; //Deshabilitar interrupciones
    while (INTCONbits.GIE == 1); //Revisar
    EECON2 = 0x55; //Secuencia de escritura
    EECON2 = 0xAA; 
    EECON1bits.WR = 1; //escribir
    INTCONbits.GIE = 1; //habilitar interrupciones
    
    while (EECON1bits.WR == 1); //mientras se escribe no hacer nada
    EECON1bits.WREN = 0; //deshabilitar escritura
}

//FUNCION DE DELAY VARIABLES
void delay(unsigned int micro){
    while (micro > 0){
        __delay_us(50); //delay de 
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