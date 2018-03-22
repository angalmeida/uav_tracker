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
int muestreoX=0;
int muestreoY=0;

int estado=ELECCION_SERVO;
char dato_serie[4];
char servo_serie[1];
int destinoX=625;
int destinoY=648;
int servo=5;
int num_char=0;
int fin_pwmX=0;
int fin_pwmY=0;
int tiempo_movimiento=0;
float c1=0;
float c2=0;
float c3=0;
float c4=0;
float c1_norm=0;
float c2_norm=0;
float c3_norm=0;
float c4_norm=0;
float c1_normY=0;
float c2_normY=0;
float c3_normY=0;
float c4_normY=0;
float x=0;
float y=0;
float x_outz1=0.5;
float y_outz1=0.5;
float x_outz2=0.5;
float y_outz2=0.5;
int flag_out=0;
int initialization=0;
void setup()
{
	SysTickDisable();                      // Disables SysTick during the configuration
  	SysTickPeriodSet(TickerPeriod);        // Define the period of the counter. When it reaches 0, it activates the interrupt
	SysTickIntRegister(&Ticker);   // The interrupt is associated to the SysTick ISR
	SysTickIntEnable();                    // The SysTick interrupt is enabled
	SysTickEnable();                       // The SysTick is enabled, after having been configured
	IntMasterEnable();                     // All interrupts are enabled
	fin_pwmX=PWM_LOW+destinoX;
	fin_pwmY=PWM_LOW+destinoY;

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
 
	//Lectura y normalizacion por eje del detector de cuadrante
	c1=(analogRead(PE_1));
	c2=(analogRead(PE_0));   
	c3=(analogRead(PE_3));  
	c4=(analogRead(PE_2));
       
	c1_normY=c1*1.254;
	c2_normY=c2*1.808;
	c3_normY=c3;
	c4_normY=c4*1.084;    

	c1_norm=c1;
	c2_norm=c2*1.169;
	c3_norm=c3*1.306;
	c4_norm=c4*1.359;
	
	//Comprobamos si hemos perdido el tracking, en cuyo caso arrancamos el controlador PI
	if((c1<75)&&(c2<75)&&(c3<75)&&(c4<75) && (flag_out==0)){
	 	flag_out=1;               
	}else{
		flag_out=0;

		x_outz2=x_outz1;
		y_outz2=y_outz1;

		x_outz1=x;
		y_outz1=y;

	}
	// Composicion de coordenadas
	x=((c1_norm+c4_norm)-(c2_norm+c3_norm))/(c1_norm+c2_norm+c3_norm+c4_norm);
	y=((c1_normY+c2_normY)-(c3_normY+c4_normY))/(c1_normY+c2_normY+c3_normY+c4_normY);
       
}


void Ticker()                          
{
	//Monitorizacion y generacion de flancos en la señal "PWM" mediante escrituras digitales HIGH y LOW. 
	if(muestreoX==0){
		digitalWrite(PC_4, HIGH);

		//Si X es mayor que 0.5 --> servo 1 mover izquierda
		//Si X es menor que -0.5 --> servo 1 mover derecha
		//Si Y es mayor que 0.5 --> servo 2 mover abajo
		//Si Y es menor que -0.5 --> servo 2 mover arriba

     	muestreoX++;
	}else if (muestreoX==destinoX){  
		digitalWrite(PC_4, LOW);
	  	muestreoX++;
	}else if (muestreoX==fin_pwmX){   
		if(flag_out==0){
			if(x>0.5){
		  		muestreoX=0;
	  			destinoX++;
              	fin_pwmX=PWM_LOW+destinoX;
            }else if (x<(-0.5)){
			 	muestreoX=0;
		  		destinoX--;
		  		fin_pwmX=PWM_LOW+destinoX;
            }else{
              	muestreoX=0;
              	fin_pwmX=PWM_LOW+destinoX;
            }
            
		}else if (flag_out==1){
		 	//En caso de perder el tracking, mediante el controlador integral se estima la trayectoria del detector de cuadrante para volver a encontrarlo. Si no es encontrado en un ciclo de "PWM", entra a funcionar el mecanismo de pointing.
			if(x_outz2>(x_outz1+0.05)){
				muestreoX=0;
             	destinoX--;
             	fin_pwmX=PWM_LOW+destinoX;
             	flag_out=2;
			}else if(x_outz2<(x_outz1-0.05)){
				muestreoX=0;
             	destinoX++;
             	fin_pwmX=PWM_LOW+destinoX;
              	flag_out=2;
           }else{
            	muestreoX=0;
            	fin_pwmX=PWM_LOW+destinoX;
             	flag_out=2;
	  		}
	 	}
	}else{
  		muestreoX++;
	}
    
	if(muestreoY==0){
		digitalWrite(PC_5, HIGH);

		//Si X es mayor que 0.5 --> servo 1 mover izquierda
       	//Si X es menor que -0.5 --> servo 1 mover derecha
		//Si Y es mayor que 0.5 --> servo 2 mover abajo
       	//Si Y es menor que -0.5 --> servo 2 mover arriba

		muestreoY++;
	}else if (muestreoY==destinoY){  
      	digitalWrite(PC_5, LOW);
      	muestreoY++;
    }else if (muestreoY==fin_pwmY){
	  	if(flag_out==0){
			if(y>0.5){
		  		muestreoY=0;
              	destinoY++;
				fin_pwmY=PWM_LOW+destinoY;
            }else if (y<(-0.5)){
		  		muestreoY=0;
              	destinoY--;
              	fin_pwmY=PWM_LOW+destinoY;
            }else{
              	muestreoY=0;
              	fin_pwmY=PWM_LOW+destinoY;
            }
	 	}else{
			//En caso de perder el tracking, mediante el controlador integral se estima la trayectoria del detector de cuadrante para volver a encontrarlo. Si no es encontrado en un ciclo de "PWM", entra a funcionar el mecanismo de pointing.
			if(y_outz2>(y_outz1+0.05)){
		  		muestreoY=0;
             	destinoY--;
             	fin_pwmY=PWM_LOW+destinoY;
           	}else if(y_outz2<(y_outz1-0.05)){
				muestreoY=0;
				destinoY++;
				fin_pwmY=PWM_LOW+destinoY;
           	}else{
				muestreoY=0;
				fin_pwmY=PWM_LOW+destinoY;
	   		}
		}
	}else{
  		muestreoY++;
	}
}


