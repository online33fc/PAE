/******************************
 *
 * Practica_03_PAE Timers i interrupcions
 * UB, 04/2023.
 * Adrià Valdueza, Soufiane Lyazidi
 *
 *****************************/

#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "lib_PAE.h"

#define LED_V_BIT BIT0
#define LED_RGB_R BIT0
#define LED_RGB_G BIT1
#define LED_RGB_B BIT2

#define SW1_POS 1
#define SW2_POS 4
#define SW1_INT 0x04
#define SW2_INT 0x0A
#define SW1_BIT BIT(SW1_POS)
#define SW2_BIT BIT(SW2_POS)
//Definimos variables para las posiciones necesarias para configurar el Joystick
#define JS_POS_U 4
#define JS_POS_D 5
#define JS_POS_R 5
#define JS_POS_L 7
#define JS_INT_U 0x0A
#define JS_INT_D 0x0C
#define JS_INT_R 0x0C
#define JS_INT_L 0x10
#define JS_BIT_U BIT(JS_POS_U)
#define JS_BIT_D BIT(JS_POS_D)
#define JS_BIT_R BIT(JS_POS_R)
#define JS_BIT_L BIT(JS_POS_L)

volatile uint8_t estado_joystick = 0;
volatile uint8_t step = 0;

typedef struct
{
    bool r, g, b;
    uint8_t time;
} color_t;

color_t color_sequence[] = { { .r = true, .g = false, .b = false, .time = 1 }, {
        .r = true, .g = true, .b = false, .time = 2 },
                             { .r = false, .g = true, .b = false, .time = 3 }, {
                                     .r = false, .g = false, .b = true, .time =
                                             2 },
                             { .r = true, .g = true, .b = true, .time = 1 }, };

#define COLOR_SIZE  5 //Mida de l'array dels colors
//L'estat inicial per defecte serà 0
volatile uint8_t current_stage = 0;
volatile uint8_t cnt_max_T1 = 1;

/**************************************************************************
 * INICIALIZACION DEL CONTROLADOR DE INTERRUPCIONES (NVIC).
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_interrupciones()
{
    // Configuracion al estilo MSP430 "clasico":
    // --> Enable Port 4 interrupt on the NVIC.
    // Segun el Datasheet (Tabla "6-39. NVIC Interrupts", apartado "6.7.2 Device-Level User Interrupts"),
    // la interrupcion del puerto 1 es la User ISR numero 35.
    // Segun el Technical Reference Manual, apartado "2.4.3 NVIC Registers",
    // hay 2 registros de habilitacion ISER0 y ISER1, cada uno para 32 interrupciones (0..31, y 32..63, resp.),
    // accesibles mediante la estructura NVIC->ISER[x], con x = 0 o x = 1.
    // Asimismo, hay 2 registros para deshabilitarlas: ICERx, y dos registros para limpiarlas: ICPRx.

    //Int. port 1 = 35 corresponde al bit 3 del segundo registro ISER1:
    NVIC->ICPR[1] |= 1 << (PORT1_IRQn & 31); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= 1 << (PORT1_IRQn & 31); //y habilito las interrupciones del puerto

    //Int. port 4 = 38 corresponde al bit 5 del segundo registro ISER1:
    NVIC->ICPR[1] |= 1 << (PORT4_IRQn & 31); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= 1 << (PORT4_IRQn & 31); //y habilito las interrupciones del puerto

    //Int. port 5 = 39 corresponde al bit 3 del segundo registro ISER1:
    NVIC->ICPR[1] |= 1 << (PORT5_IRQn & 31); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= 1 << (PORT5_IRQn & 31); //y habilito las interrupciones del puerto

    //Int. TA0 de CCTL0, corresponde al bit 8 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << TA0_0_IRQn; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= 1 << TA0_0_IRQn; //y habilito las interrupciones del puerto

    //Int. TA0 de CCTL0, corresponde al bit 10 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << TA1_0_IRQn; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= 1 << TA1_0_IRQn; //y habilito las interrupciones del puerto

}

/**************************************************************************
 * INICIALIZACION DE LOS BOTONES & LEDS DEL BOOSTERPACK MK II.
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_botons(void)
{
    //Configuramos botones i LED vermell
    //***************************
    P1SEL0 &= ~(BIT0 + BIT1 + BIT4 );    //Els polsadors son GPIOs
    P1SEL1 &= ~(BIT0 + BIT1 + BIT4 );    //Els polsadors son GPIOs

    //LED vermell = P1.0
    P1DIR |= LED_V_BIT;      //El LED es una sortida
    P1OUT &= ~LED_V_BIT;     //El estat inicial del LED es apagat

    //Boto S1 = P1.1 i S2 = P1.4
    P1DIR &= ~(SW1_BIT + SW2_BIT);    //Un polsador es una entrada
    P1REN |= (SW1_BIT + SW2_BIT);     //Pull-up/pull-down pel pulsador
    P1OUT |= (SW1_BIT + SW2_BIT); //Donat que l'altra costat es GND, volem una pull-up
    P1IE |= (SW1_BIT + SW2_BIT);      //Interrupcions activades
    P1IES &= ~(SW1_BIT + SW2_BIT);    // amb transicio L->H
    P1IFG = 0;                  // Netegem les interrupcions anteriors
}

/*****************************************************************************
 * CONFIGURACION DE LOS LEDs DEL PUERTO 1. A REALIZAR POR EL ALUMNO
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 ****************************************************************************/
void config_RGB_LEDS(void)
{

    //LEDs RGB = P2.0, P2.1, P2.2
    P2SEL0 &= ~(BIT0 + BIT1 + BIT2 );    //P2.0, P2.1, P2.2 son GPIOs
    P2SEL1 &= ~(BIT0 + BIT1 + BIT2 );    //P2.0, P2.1, P2.2 son GPIOs
    P2DIR |= (BIT0 + BIT1 + BIT2 );      //Els LEDs son sortides
    P2OUT &= ~(BIT0 + BIT1 + BIT2 );     //El seu estat inicial sera apagat



}

void config_RGB_LEDS_joytsick(void)
{

    //LEDs RGB = P2.6 (R), P2.4 (G), P5.6 (B)
    P2SEL0 &= ~(BIT4 + BIT6 );    //P2.6, P2.4 son GPIOs
    P2SEL1 &= ~(BIT4 + BIT6 );    //P2.6, P2.4 son GPIOs
    P5SEL0 &= ~(BIT6 );    //P5.6 es GPIO
    P5SEL1 &= ~(BIT6 );    //P5.6 es GPIO

    P2DIR |= ( BIT4 + BIT6 );      //Els LEDs son sortides (configuració per LEDs R i G)
    P2OUT &= ~(BIT4 + BIT6 );     //Els posem a 0 per configurar els seu estat inicial amb una porta OR  (configuració per LEDs R i G)
    P5DIR |= ( BIT6 );      //El LED es sortida (configuració per LEDs B)
    P5OUT &= ~( BIT6 );     //E posem a 0 per configurar els seu estat inicial amb una porta OR (configuració per LEDs B)

    //L'estat inicial serà el 0 del color_sequence
    //true ens retorna un 1 desplaçem l'estat per cada color els bit necesaris (el green seri el bit 1 desplaçem un bit)
    P2OUT |= ((color_sequence[current_stage].r << 6)
            | (color_sequence[current_stage].g << 4));
    P5OUT |= (color_sequence[current_stage].b << 6);




}
/**************************************************************************
 * INICIALIZACION DELS TIMERS
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_timers(void)
{
    //TODO
    //Timer A0, used for red LED PWM
    //Divider = 1; CLK source is SMCLK; clear the counter; MODE is up
    TIMER_A0->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__SMCLK
            | TIMER_A_CTL_CLR | TIMER_A_CTL_MC__UP; //
    TIMER_A0->CCR[0] = 2400 - 1;     // 10k Hz
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0

    //Timer A1, used for RGB LEDs color transitions
    //Divider = 1; CLK source is ACLK; clear the counter; MODE is up
    TIMER_A1->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_CLR
            | TIMER_A_CTL_MC__UP;
    TIMER_A1->CCR[0] = (1 << 15) - 1;     // 1 Hz
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0

}

/**************************************************************************
 * INICIALIZACION DEL JOYSTICK
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_joystick(void)
{
    //Configuramos el joystick (dreta, esquerra amunt i avall)

    //Joysticks dreta i esquerra D = P4.5 i E = P4.7
    P4SEL0 &= ~(BIT5 + BIT7 );    //Els joystick dreta i esquerra son GPIOs
    P4SEL1 &= ~(BIT5 + BIT7 );    //Els joystick dreta i esquerra son GPIOs

    //Joysticks dreta i esquerra amunt = P5.4 i Avall = P5.5
    P5SEL0 &= ~(BIT4 + BIT5 );    //Els joystick amunt i avall son GPIOs
    P5SEL1 &= ~(BIT4 + BIT5 );    //Els joystick amunt i avall son GPIOs

    //Joysticks dreta i esquerra D = P4.5 i E = P4.7
    P4DIR &= ~(JS_BIT_R + JS_BIT_L);    //Un joystick es una entrada
    P4REN |= (JS_BIT_R + JS_BIT_L);     //Pull-up/pull-down pel joystick
    P4OUT |= (JS_BIT_R + JS_BIT_L); //Donat que l'altra costat es GND, volem una pull-up
    P4IE |= (JS_BIT_R + JS_BIT_L);      //Interrupcions activades
    P4IES &= ~(JS_BIT_R + JS_BIT_L);    // amb transicio L->H
    P4IFG = 0;                  // Netegem les interrupcions anteriors

    //Joysticks dreta i esquerra amunt = P5.4 i Avall = P5.5
    P5DIR &= ~(JS_BIT_U + JS_BIT_D);    //Un joystick es una entrada
    P5REN |= (JS_BIT_U + JS_BIT_D);     //Pull-up/pull-down pel joystick
    P5OUT |= (JS_BIT_U + JS_BIT_D); //Donat que l'altre costat es GND, volem una pull-up
    P5IE |= (JS_BIT_U + JS_BIT_D);      //Interrupcions activades
    P5IES &= ~(JS_BIT_U + JS_BIT_D);    // amb transicio L->H
    P5IFG = 0;                  // Netegem les interrupcions anteriors
}

//inicializacion de variables relacionadas con el PWM duty

#define CNT_MAX 100 //El màxim a comptar amb el T0

volatile int8_t pwm_duty = 50; //valor on s'encen el T0

#define PWM_DUTY_CHANGE 10  //valor inicial del step del duty

volatile int8_t pwm_step = PWM_DUTY_CHANGE; //valor del step del duty

#define PWM_STEP_UPDATE 5 //valor d'actualització del step


/**************************************************************************
 * ACTUALITZACION DEL PWM
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void update_PWM(void)
{

    if (estado_joystick != 0) //si el estado no es 0, hay que actualizar los valores PWM dependiendo del estado
    {
        /*
         * Joystick desplazado hacia arriba: estado_joystick=1
         * Joystick desplazado hacia abajo: estado_joystick=2
         * Joystick desplazado hacia la derecha: estado_joystick=3
         * Joystick desplazado hacia la izquierda: estado_joystick=4
         * */
        //en funcion del estado del joystick variamos los valores pwm
        switch (estado_joystick)
        {
        case 1:
            if (pwm_duty + pwm_step > CNT_MAX) //Si se sobrepassa el maximo a contar (CNT_MAX) se corta en el maximo
                pwm_duty = CNT_MAX;
            else if (pwm_duty + pwm_step < 0) //Si se llega a un valor negativo se corta en el 0
                pwm_duty = 0;
            else
                //En el caso general se resta el step al duty
                pwm_duty += pwm_step;
            break;
        case 2:
            if (pwm_duty - pwm_step > CNT_MAX) //Si se sobrepassa el maximo a contar (CNT_MAX) se corta en el maximo
                pwm_duty = CNT_MAX;
            else if (pwm_duty - pwm_step < 0) //Si se llega a un valor negativo se corta en el 0
                pwm_duty = 0;
            else
                //En el caso general se resta el step al duty
                pwm_duty -= pwm_step;
            break;
        case 3:
            pwm_step += PWM_STEP_UPDATE; //Se actualiza el valor del salto (step) sumandole 5
            break;
        case 4:
            pwm_step -= PWM_STEP_UPDATE; //Se actualiza el valor del salto (step) restandole 5
            break;
        default:
            break;
        }
        estado_joystick = 0; //El estado a 0 para que entre en un nuevo movimento del joystick
    }

}

/**************************************************************************
 * INICIALIZACION DEL RGB del boosterpack
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void update_RGB(void)
{
    static uint8_t anterior_stage = 0;
    //en caso que haya un cambio de estado del circuito de los RGB se actualizan los RGB
    if (current_stage != anterior_stage)
    {
        anterior_stage = current_stage; //se actualiza el estado anterior al actual

        //Establecemos a 0 el estado de los bits RGB amb un màscara AND
        P2OUT &= ~(BIT4 + BIT6 );
        P5OUT &= ~( BIT6 );
        //Establecemos al estado que les corresponde a los RGB segun color_sequence amb una màscara OR
        //Desplaçant l'estat de cada LED dels RGB a la posició corresponent
        P2OUT |= ((color_sequence[current_stage].r << 6)
                    | (color_sequence[current_stage].g << 4));
        P5OUT |= (color_sequence[current_stage].b << 6);
    }
}

void main(void)
{

    WDTCTL = WDTPW + WDTHOLD;       // Stop watchdog timer

    //Inicializaciones:
    init_ucs_24MHz();
    init_botons();         //Configuramos botones y leds (encara que en aquest codi sigui innecesari pel que respecte als botons)

    init_joystick(); //configurem el joystick
    config_RGB_LEDS(); //configurem els RGB (encara que en aquest codi sigui innecesari)
    config_RGB_LEDS_joytsick(); //configurem els RGB del boosterpack
    init_timers(); //configurem els Timers
    init_interrupciones(); //Configurar y activar las interrupciones de los botones


    __enable_interrupts(); //Activem les interrupcions a nivell global

    //Bucle principal (infinito):
    while (true)
    {

        update_PWM(); //cridem a la funció que si s'escau actualitza els valors d'on s'encen el LED1
        update_RGB(); //cridem la funció que si s'escau actualitza l'estat dels RGB

    }

}

void TA0_0_IRQHandler(void)
{
    static uint8_t cnt = 0;

    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG; //Clear interrupt flag

    //Actualitzem en 1 el comptador
    cnt++;

    //Si hem arribat al valor màxim resetejem el comptador a 0 y encenem el LED amb una màscara OR
    if (cnt == CNT_MAX)
    {
        cnt = 0;
        P1OUT |= LED_V_BIT;

    }

    //En cas que estiguem sobre el duty hem d'apagar el LED i es fa amb una màscara AND
    if (cnt == pwm_duty)
    {
        P1OUT &= ~LED_V_BIT;
    }

}


void TA1_0_IRQHandler(void)
{
    //TODO

    static uint8_t cnt = 0;

    TA1CCTL0 &= ~TIMER_A_CCTLN_CCIFG; //Clear interrupt flag

    cnt++;  //Actualitzem en 1 el comptador

    if (cnt == cnt_max_T1)  //en cas hem arribat al màxim a comptar
    {
        cnt = 0; //resetejem el comptador
        current_stage++; //actualitzem sumant-li 1 a l'estat actual (posició al color_sequence)
        current_stage = current_stage % COLOR_SIZE; //ho posem modul del size del color_sequence
        cnt_max_T1 = color_sequence[current_stage].time; //actualitzem el pròxim temps fins al cual comptarem
    }

}

/**************************************************************************
 * RUTINAS DE GESTION DE LOS BOTONES:
 * Mediante estas rutinas, se detectar que boton se ha pulsado
 *
 * Sin Datos de entrada
 *
 * Sin datos de salida
 *
 * Actualizar el valor de la variable global estado
 *
 **************************************************************************/

//ISR para las interrupciones del puerto 1:
void PORT1_IRQHandler(void)
{
    uint8_t flag = P1IV; //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P1IE &= ~(SW1_BIT + SW2_BIT); //interrupciones del boton S2 en port 3 desactivadas

    switch (flag)
    {
    case SW1_INT:
        //codi temporal per fer comprovacions
        //estado_joystick = 1;
        break;
    case SW2_INT:
        //codi temporal per fer comprovacions
        //estado_joystick = 2;
        break;
    default:
        break;
    }

    P1IE |= (SW1_BIT + SW2_BIT);   //interrupciones S2 en port 3 reactivadas
}

/**************************************************************************
 * RUTINAS DE GESTION DE LOS BOTONES:
 * Mediante estas rutinas, se detectar  que boton se ha pulsado
 *
 * Sin Datos de entrada
 *
 * Sin datos de salida
 *
 * Actualizar el valor de la variable global estado_joytsick
 *
 **************************************************************************/

//ISR para las interrupciones del puerto 1:
void PORT4_IRQHandler(void)
{
    uint8_t flag = P4IV; //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P4IE &= ~(JS_BIT_R + JS_BIT_L); //interrupciones de los joystick de derecha e izquierda en port 4 desactivadas

    switch (flag)
    {
    case JS_INT_R: //CAS joystick mogut a la dreta
        estado_joystick = 3;  //estado_joystick 3 que indica al codi que el joystick ha estat mogut cap a la dreta
        break;
    case JS_INT_L:   //CAS joystick mogut cap a l'esquerra
        estado_joystick = 4;  //estado_joystick 4 que indica al codi que el joystick ha estat mogut cap a l'esquerra
        break;
    default:
        break;
    }

    P4IE |= (JS_BIT_R + JS_BIT_L);   //interrupciones de los joystick de derecha e izquierda en port 4 reactivadas
}

/**************************************************************************
 * RUTINAS DE GESTION DE LOS BOTONES:
 * Mediante estas rutinas, se detectar donde se ha movido el joystick
 *
 * Sin Datos de entrada
 *
 * Sin datos de salida
 *
 * Actualizar el valor de la variable global estado_joytsick
 *
 **************************************************************************/

//ISR para las interrupciones del puerto 1:
void PORT5_IRQHandler(void)
{
    uint8_t flag = P5IV; //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P5IE &= ~(JS_BIT_U + JS_BIT_D); //interrupciones de los joystick de arriba y abajo en port 5 desactivadas

    switch (flag)
    {
    case JS_INT_U: //CAS joystick mogut adalt
        estado_joystick = 1; //estado_joystick 1 que indica al codi que el joystick ha estat mogut cap adalt
        break;
    case JS_INT_D: //CAS joystick mogut avall
        estado_joystick = 2; //estado_joystick 1 que indica al codi que el joystick ha estat mogut cap avall
        break;
    default:
        break;
    }

    P5IE |= (JS_BIT_U + JS_BIT_D);   //interrupciones de los joystick de arriba y abajo en port 5 reactivadas
}
