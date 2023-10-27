/*
 * (APAGADO-ENCENDIDO) - PWM.c
 *
 * Created: 26/10/2023 09:11:43
 * Author : user
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Declaracion de las funciones
void conf_interrupciones(void);
void conf_motor_LED(void);
void conf_PWM (void);
void Velocidad_girar_derecha(void);
void Velocidad_girar_izquierda(void);
void Velocidad_2();
void Velocidad_3(void);
void Velocidad_4(void);
void Reversa(void);

// Frecuencia para el PWM
#define PWM_Frecuencia 1000

// Declaramos los estados
enum ESTADOS {APAGADO, ENCENDIDO, CANT_ESTADOS};
int EstadoActual;

// Estado: APAGADO
void apagado (void) {
	if ((PIND & (1 << PIND3)) != 0) {
		// Pulsador encedido
		EstadoActual = ENCENDIDO;
	}
}

// ESTADO: ENCENDIDO
void encendido(void) {

	// INICIO DE LA SECUENCIA
	Velocidad_2();
	_delay_ms(3000);
	Velocidad_girar_derecha();
	_delay_ms(1000);
	Velocidad_3();
	_delay_ms(4000);
	Velocidad_girar_derecha();
	_delay_ms(1000);
	Velocidad_4();
	_delay_ms(4000);
	Velocidad_girar_derecha();
	_delay_ms(1000);
	
	if ((PIND & (1 << PIND3)) == 0) {
		// Pulsador apagado
		EstadoActual = APAGADO;
	}
}

/*------------------------------------------------------------------------------------------------------------------------------------------- */
int main(void) {
	// PUERTO DE ENTRADA PARA EL PULSADOR 
	DDRD &= ~(1 << DDD3);
	
	conf_motor_LED();
	conf_interrupciones();
	conf_PWM();
	
	OCR1A = OCR1B = 0;
	
	void (*vector_estados[CANT_ESTADOS])();
	vector_estados[APAGADO] = apagado;
	vector_estados[ENCENDIDO] = encendido;
	
	EstadoActual = APAGADO;
	
	/* Replace with your application code */
	while (1) {
		vector_estados[EstadoActual]();
	}
	
}
/*------------------------------------------------------------------------------------------------------------------------------------------- */

void conf_motor_LED(void){
	// CONFIGURAMOS EL PUERTO D PARA LOS LEDS (PIN 4 Y 7)
	DDRD |= (1 << DDD4); // ROJO
	DDRD |= (1 << DDD7); // VERDE
	
	// CONFIGURACION DEL PUERTO B PARA LOS MOTORES (PINES 1, 2, 3 Y 4)
	// MOTOR A
	DDRB |= (1 << DDB1);
	DDRB |= (1 << DDB2);
	// MOTOR B
	DDRB |= (1 << DDB3);
	DDRB |= (1 << DDB4);
	
}

void conf_interrupciones(void){
	// PIND 2 COMO ENTRADA PARA EL BOTON DE LOS CHOQUES
	DDRD &= ~(1 << DDD2);
	
	EICRA = (1 << ISC00);
	EIMSK = (1 << INT0);
}

void conf_PWM (void){
	
	// CONFIGURAMOS EL PUERTO B (PIN 1 Y 2) COMO SALIDA PARA EL PWM
	DDRD |= (1 << DDD5);
	DDRD |= (1 << DDD6);
	
	// Top a 124
	ICR1 = F_CPU / (8 * PWM_Frecuencia) - 1;
	
	//OCR1A = OCR1B = 0;
	
	// no invertido
	TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
	
	// Fast PWM mode usando ICR1 como TOP
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12)|(1 << WGM13);
	
	// Timer con preescaler de 8
	TCCR1B |= (1 << CS11);
}

void Velocidad_girar_derecha(void){
	// CICLO DE TRABAJO DEL 25% PARA EL PWM
	OCR1A = 31;
	OCR1B = 31;
	
	// ACTIVAMOS LA LED DE COLOR VERDE
	PORTD |= (1 << PIND7);
	
	// ACTIVAMOS LAS SALIDAS PARA LOS MOTORES
	// MOTOR A (IZQUIERDA)
	PORTB |= (1 << PINB1); // SALIDA 0 --> 1
	PORTB &= ~(1 << PINB2);
	// MOTOR B (DERECHA)
	PORTB &= ~(1 << PINC3);
	PORTB &= ~(1 << PINC4);
}

void Velocidad_girar_izquierda(void){
	// CICLO DE TRABAJO DEL 25% PARA EL PWM
	OCR1A = 31;
	OCR1B = 31;

	// ACTIVAMOS LA LED DE COLOR VERDE
	PORTD |= (1 << PIND7);
	
	// ACTIVAMOS LAS SALIDAS PARA LOS MOTORES
	// MOTOR A (IZQUIERDA)
	PORTB &= ~(1 << PINB1); 
	PORTB &= ~(1 << PINB2);
	// MOTOR B (DERECHA)
	PORTB |= (1 << PINB3);// SALIDA 0 --> 1
	PORTB &= ~(1 << PINB4);
}

void Velocidad_2(void){
	// CICLO DE TRABAJO DEL 50% PARA EL PWM
	OCR1A = 62;
	OCR1B = 62;

	// ACTIVAMOS LA LED DE COLOR VERDE
	PORTD |= (1 << PIND7);
	
	// ACTIVAMOS LAS SALIDAS PARA LOS MOTORES
	// MOTOR A (IZQUIERDA)
	PORTB |= (1 << PINB1); // SALIDA 0 --> 1
	PORTB &= ~(1 << PINB2);
	// MOTOR B (DERECHA)
	PORTB |= (1 << PINB3); // SALIDA 2 --> 1
	PORTB &= ~(1 << PINB4);
}

void Velocidad_3(void){
	
	// CICLO DE TRABAJO DEL 75% PARA EL PM
	OCR1A = 93;
	OCR1B = 93;
	
	// ACTIVAMOS LA LED DE COLOR VERDE
	PORTD |= (1 << PIND7);
	
	// ACTIVAMOS LAS SALIDAS PARA LOS MOTORES
	// MOTOR A (IZQUIERDA)
	PORTB |= (1 << PINB1); // SALIDA 0 --> 1
	PORTB &= ~(1 << PINB2);
	// MOTOR B (DERECHA)
	PORTB |= (1 << PINB3); // SALIDA 2 --> 1
	PORTB &= ~(1 << PINB4);
}

void Velocidad_4(void){
	// CICLO DE TRABAJO DEL 100% PARA EL PWM
	OCR1A = 124;
	OCR1B = 124;
	
	// ACTIVAMOS LA LED DE COLOR VERDE
	PORTD |= (1 << PIND7);
	
	// ACTIVAMOS LAS SALIDAS PARA LOS MOTORES
	// MOTOR A (IZQUIERDA)
	PORTB |= (1 << PINB1); // SALIDA 0 --> 1
	PORTB &= ~(1 << PINB2);
	// MOTOR B (DERECHA)
	PORTB |= (1 << PINB23); // SALIDA 2 --> 1
	PORTB &= ~(1 << PINB4);
}

void Reversa(void){
	// CICLO DE TRABAJO PARA EL PWM 50%
	OCR1A = 62;
	OCR1B = 62;
	
	// ACTIVAMOS LAS SALIDAS PARA LOS MOTORES
	// MOTOR A (IZQUIERDA)
	PORTC &= ~(1 << PINC0);
	PORTC |= (1 << PINC1); // SALIDA 1 --> 1
	// MOTOR B (DERECHA)
	PORTC &= ~(1 << PINC2);
	PORTC |= (1 << PINC3); // SALIDA 3 --> 1
}

ISR(INT0_vect){
	
	// ACTIVAMOS LA LED DE COLOR VERDE
	PORTD &= ~(1 << PIND7);
	PORTD |= (1 << PIND4);
	
	//AL DETERCTAR EL CHOQUE:
	// 1. HAGA REVERSA
	// 2. HAGA UN GIRO A LA IZQUIERDA (EN REVERSA)
	Reversa();
	_delay_ms(2000);
	Velocidad_girar_izquierda();
	_delay_ms(500);
	
}
