#include <Servo.h>
#include <stdint.h>              // standard library for integers (used in the next libraries)
#include "driverlib\systick.h"   // standard library for the SysTick (header)
#include "driverlib\systick.c"   // standard library for the SysTick (functions)

#include "wiring_private.h"      // library to access the PWM with configurable frequency

#define TickerPeriod 240   // Definicion del TickerPeriod: 120e6/240=500e3 Hz
#define ELECCION_SERVO 0	// MAQUINA ESTADOS: Eleccion Servo - Reposo - Momivimiento
#define REPOSO 1
#define MOVIMIENTO 2
#define PWM_LOW 10000    //20ms a 240ticks
#define T_MOV 1000000   //A 120MHz, si la interrupcion salta cada 240ticks, tenemos una interrupcion de timer cada 2us.
                         

#include "Arduino.h"
void setup();
void loop();
void Ticker();
int muestreo=0;
int estado=ELECCION_SERVO;
char dato_serie[4];
char servo_serie[1];
int destino=0;
int servo=5;
int num_char=0;
int fin_pwm=0;
int tiempo_movimiento=0;
float c1=0;
float c2=0;
float c3=0;
float c4=0;
float c1_norm=0;
float c2_norm=0;
float c3_norm=0;
float c4_norm=0;
float x=0;
float y=0;

void setup()
{
	SysTickDisable();                      // Disables SysTick during the configuration
	SysTickPeriodSet(TickerPeriod);        // Define the period of the counter. When it reaches 0, it activates the interrupt
	SysTickIntRegister(&Ticker);   // The interrupt is associated to the SysTick ISR
	SysTickIntEnable();                    // The SysTick interrupt is enabled
	SysTickEnable();                       // The SysTick is enabled, after having been configured
	IntMasterEnable();                     // All interrupts are enabled


	// put your setup code here, to run once:
	//Inicializamos el Serial y los puertos de salida para los servos y los de entrada para el detector de cuadrante
	Serial.begin(9600);
	pinMode(PC_4,OUTPUT);
	pinMode(PC_5,OUTPUT);
	pinMode(PE_0,INPUT);
	pinMode(PE_1,INPUT);
	pinMode(PE_2,INPUT);
	pinMode(PE_3,INPUT);
}

void loop()
{
	//ESTADO: ELECCION DE SERVO
	if (estado == ELECCION_SERVO){
	
		Serial.begin(9600);
		Serial.println(" ");
		Serial.println("Las medidas actuales son las siguientes: ");
		//Lectura y normalizacion del detector de cuadrante
		c1=(analogRead(PE_1));
		c2=(analogRead(PE_0));   
		c3=(analogRead(PE_3));  
		c4=(analogRead(PE_2));
		Serial.print("Cuadrante 1: ");
		Serial.println(c1);
		Serial.print("Cuadrante 2: ");
		Serial.println(c2);
		Serial.print("Cuadrante 3: ");
		Serial.println(c3);
		Serial.print("Cuadrante 4: ");
		Serial.println(c4);
		Serial.println(" ");
		c1_norm=c1;
		c2_norm=c2*1.169;
		c3_norm=c3*1.306;
		c4_norm=c4*1.359;
		// Composicion de coordenadas
		x=((c1_norm+c4_norm)-(c2_norm+c3_norm))/(c1_norm+c2_norm+c3_norm+c4_norm);
		y=((c1_norm+c2_norm)-(c3_norm+c4_norm))/(c1_norm+c2_norm+c3_norm+c4_norm);
		Serial.print("X: ");
		Serial.println(x);
		Serial.print("Y: ");
		Serial.println(y);
		Serial.println(" ");
		//Eleccion de servo por puerto serie
		Serial.println("Introduzca el servo que desea mover (1-abajo, 2-arriba): ");
   
    	while (Serial.available()<=0){
        	delay(1000);
      	}
      	if (Serial.available()>0) {
			Serial.readBytes(servo_serie,1); 
			Serial.print("Introducida posicion: ");
			servo=atoi(servo_serie);
			Serial.println(servo,DEC);
			Serial.end();
    
			if(servo<3 && servo >0){
				//Tras elegir el servo pasamos a elegir la posicion destino en el estado REPOSO
				estado=REPOSO;
			}
		}


    
	}
	//Estado REPOSO: Se elige la direccion destino y se modifican las variables que definen la nueva "PWM"
	if (estado==REPOSO){
		Serial.begin(9600);
	  	Serial.println("Introduzca posicion destino: (Rango: 375 (0 grados) - 1125 (180 grados)): ");
		while (Serial.available()<=0){
        	delay(1000);
      	}
      	if (Serial.available()>0) {
	 		Serial.readBytes(dato_serie,4); 
			Serial.print("Introducida posicion: ");
			destino=atoi(dato_serie);
			Serial.println(destino,DEC);
			Serial.end();
			fin_pwm=PWM_LOW+destino;
    
			if(destino<1126 && destino >374){
				tiempo_movimiento=0;
				estado=MOVIMIENTO;
      		}
  		}
   	}
}


void Ticker()                          
{
	//Monitorizacion y generacion de flancos en la señal "PWM" mediante escrituras digitales HIGH y LOW. 
	if (estado==MOVIMIENTO && tiempo_movimiento<T_MOV){
    	tiempo_movimiento++;
    	if(muestreo==0){
      		if (servo==1){
          		digitalWrite(PC_4, HIGH);
          	}else{
         		digitalWrite(PC_5, HIGH);
          	}
      	muestreo++;
    	}else if (muestreo==destino){
			if (servo==1){        
            	digitalWrite(PC_4, LOW);
            }else{
              	digitalWrite(PC_5, LOW);
            }
			muestreo++;
		}else if (muestreo==fin_pwm){
      		muestreo=0;
    	}else{
      		muestreo++;
    	}
	}else{
    	estado=ELECCION_SERVO;
  	}
}
