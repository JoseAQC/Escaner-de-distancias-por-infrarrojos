
//EL OBJETIVO DE ESTA PRÁCTICA ES CONSTRUIR UN ESCÁNER DE DISTANCIAS BASADO EN EL MICROCONTROLADOR 
//LPC1768 UTILIZANDO EL SENSOR DE INFRARROJOS SHARP GP2Y0A02 PARA LA DETECCIÓN DE OBSTÁCULOS QUE 
//REALIZA BARRIDOS DE FORMA AUTOMÁTICA O MANUAL A PARTIR DE LOS PULSADORES KEY1 Y KEY2, AMBOS MODOS 
//DE FUNCIONAMIENTO PUEDEN DETECTAR OBSTÁCULOS QUE SE ENCUENTREN ENTRE 30 Y 150 CM. AL DETECTAR AL
//OBJETO SE REPRODUCE UNA SEÑAL DE ALARMA. ADICIONALMENTE, EL LPC SE PUEDE CONECTAR A UN ORDENADOR 
//USANDO UN PROGRAMA DE TERMINAL O CON UN SMARTPHONE MEDIANTE UNA CONEXIÓN BLUETOOTH.
// AUTORES:
// José Quinga Calvachi  y  Alfonso Fernández Álvarez
//

#include <LPC17xx.H>
#include "uart.h"
#include "uart_2.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "lcddriver.h"
#include <Math.h>
#define F_cpu 100e6			// Defecto Keil (xtal=12Mhz)
#define F_pclk F_cpu/4 	// Defecto despues del reset
#define Ftick 750 // Frecuencia de interrupción del SYSTICK (1kHz-preiodo: 1ms)

uint8_t i=0,j,mode=0, sentido=1,delay=0;
uint16_t key1=0,key2=0,a=0,iter=0;
uint32_t contador,contador1=0,pulsacion,comfirmo=0, aux=0,contador2=0,resoluciongrad=10,resolucion=113,dis=0,disadc,voltaje;
int angulo=0;
double debs=0, debc=0;	
double x1,y1;
char dm[200],dm1[200],ang[6], tiempo_string[10000];

uint8_t secuencia1[8]={1,3,2,6,4,12,8,9};

////Variables Cronómetro Key 1
uint32_t t_key1=0;
uint16_t c_timer2=0,c_key1=0;

/////variables  configuracion ISP ////
uint8_t c_key0=0;
uint8_t end_ISP=0;

////UART
float periodomov=2000000;

//Alarma
int contperiodo=0;
#define pi 3.14159
#define N_muestras 32	// 32 muestras/ciclo
#define V_refp 3.3
uint16_t muestras[N_muestras];
#define Umbral_dist 50    //En centimetros
int F_out = 500-(Umbral_dist);		  // En Hercios

//UART
char buffer[30]; // Buffer de recepción de 30 caracteres
char *ptr_rx; // puntero de recepción
char rx_completa; // Flag de recepción de cadena que se activa a "1" al recibir la tecla return CR(ASCII=13)
char *ptr_tx; // puntero de transmisión
char tx_completa; // Flag de transmisión de cadena que se activa al transmitir el caracter null (fin de cadena)
char fin=0;
char distantmessage[10];	
char distantmessage2[10];	
//Bluetooth UART
char b_buffer[30]; // Buffer de recepción de 30 caracteres
char *b_ptr_rx; // puntero de recepción
char b_rx_completa; // Flag de recepción de cadena que se activa a "1" al recibir la tecla return CR(ASCII=13)
char *b_ptr_tx; // puntero de transmisión
char b_tx_completa; // Flag de transmisión de cadena que se activa al transmitir el caracter null (fin de cadena)


void genera_muestras(uint16_t muestras_ciclo)
{
	uint16_t i;
	//señal senoidal
	for(i=0;i<muestras_ciclo;i++)
	muestras[i]=(uint32_t)(511.5+511.5*sin((2*pi*i)/N_muestras)); // Ojo! el DAC es de 10bits
}

void LCD()
{
    lcdInitDisplay();
    fillScreen(BLACK);
		if(mode){
	
		drawString(30,4,"Estas en modo automatico", CYAN, BLACK, SMALL); 
		drawString(20,80,"ISP: iniciar/detener", WHITE, BLACK, MEDIUM);
		drawString(20,100," medidas de distancia", WHITE, BLACK, MEDIUM);
			
					tx_cadena_UART0("Estas en modo automatico\n\r"
										"1. xxg (introduce la resolucion en grados xx, seguido de g y terminado en Enter: 10g, 15g, 20g)\n\r"
										"2. xxxms (introduce el periodo en milisegundos xxx, seguido de ms y terminado en Enter: 200ms, 400ms, 600ms)\n\r"
										"3. h (Para volver a mostrar este mensaje de ayuda, pulsar h terminado en Enter)\n\r");	
		}
else  {
    setRotation(3);
    drawString(40,40,"Estas en modo manual", CYAN, BLACK, MEDIUM);
	  drawString(20,80,"Key 1: ", WHITE, BLACK, MEDIUM);
	  drawString(20,100,"Girar 10g a la izquierda", WHITE, BLACK, MEDIUM);
	  drawString(20,140,"Key 2:", WHITE, BLACK, MEDIUM);
		drawString(20,160,"Girar 10g a la derecha", WHITE, BLACK, MEDIUM);
	  drawString(20,190,"ISP: iniciar/detener", WHITE, BLACK, MEDIUM);
		drawString(20,210," medidas de distancia", WHITE, BLACK, MEDIUM);
		drawString(30,270,"Mantener pulsado Key1 ", CYAN, BLACK, SMALL); 
		drawString(30,290,"para modo automatico", CYAN, BLACK, SMALL); 
		}
}
void ADCconfig()
{
    LPC_SC->PCONP|= (1<<12);             // Power ON
    LPC_PINCON->PINSEL1 |= (1<<14);      // ADC input= P0.23 (AD0.0)
    LPC_PINCON->PINMODE1|= (2<<14);     // Deshabilita pullup/pulldown        
    LPC_ADC->ADCR|= (1<<0)|(24<<8)|(1<<16)|(1<<21);          
}

void config_IRQ(){

LPC_PINCON->PINSEL4|=(1<<22)|(1<<24)|(1<<20);  //P2.10 EINT0,p2.11 eint1, p2.12 eint2 
LPC_SC->EXTMODE |= 0x7;                        //interruciones activas por flanco
LPC_SC->EXTPOLAR &= (0<<0);                    //flanco de bajada
NVIC->ISER[0]|=(7<<18);                            //Habilita las interrupciones
LPC_SC->EXTINT=7;

NVIC_SetPriority (34, 7);
NVIC_SetPriority (35, 10);  //prioridad 1 exclusiva
NVIC_SetPriority (36, 10);
NVIC_SetPriority (15, 20);   //prioridad 2 exclusiva
}
void init_DAC(void)
{
	LPC_PINCON->PINSEL1|= (2<<20); 	 	// DAC output = P0.26 (AOUT)
	LPC_PINCON->PINMODE1|= (2<<20); 	// Deshabilita pullup/pulldown
	LPC_DAC->DACCTRL=0;								//  
}


/*  Timer 1 en modo Output Compare (reset T0TC on Match 0)
	  Counter clk: 25 MHz 	MAT1.0 : On match, salida de una muestra hacia el DAC */
void init_TIMER1(void)
{
	  LPC_SC->PCONP|=(1<<2);						// 	Power ON
    LPC_TIM1->PR = 0x00;     	 				//  Prescaler =1
    LPC_TIM1->MCR = 0x03;							//  Reset TC on Match, e interrumpe!  
    LPC_TIM1->MR0 = (F_pclk/F_out/N_muestras)-1;  // Cuentas hasta el Match 
		//Si yo quisiera que la frecuencia variara con la distancia debería variar el valor del Mach0 que es solo posible con un PWM
    LPC_TIM1->EMR = 0x00;   					//  No actúa sobre el HW
    LPC_TIM1->TCR = 0x00;							// 
    NVIC_EnableIRQ(TIMER1_IRQn);			//  Habilita NVIC
	  NVIC_SetPriority(TIMER1_IRQn,9);   
}
void init_TIMER0(void)
{
	  LPC_SC->PCONP|=(1<<1);						// 	Power ON
    LPC_TIM0->PR = 25-1;     	 				//  Prescaler =1
    LPC_TIM0->MCR |=(1<<1)|(1<<0) ;		//  Reset TC on Match, e interrumpe!  
    LPC_TIM0->MR0 = 2000000-1; 					 // 
    LPC_TIM0->EMR = 0x00;   					//  No actúa sobre el HW
    LPC_TIM0->TCR = 0x00;							//  
    NVIC_EnableIRQ(TIMER0_IRQn);			//  Habilita NVIC
	  NVIC_SetPriority(TIMER0_IRQn,10);   
}

void MedidaIR(){

	voltaje=(((LPC_ADC->ADGDR >>4)&0xFFF)*3300/4095);
	dis = (58125/(voltaje-63));
	
	
	
	sprintf(distantmessage2, " angulo:%d Distancia: %i cm\n\r",angulo,dis);   //Print a string
	
	tx_cadena_UART0(distantmessage2);
	tx_cadena_UART2(distantmessage2);
}

void EINT0_IRQHandler(void){
	
	LPC_SC->EXTINT|=2;  //borrar flag
	
	if(mode==0){
	if(c_key0==0){
		
		NVIC_DisableIRQ(EINT1_IRQn);	//Deshailitar Key 1
		NVIC_DisableIRQ(EINT2_IRQn);	//Deshabilitar Key 2
		LPC_TIM0->TCR =1;		//  Start Timer 0
		c_key0=1;
	}
	else {
		LCD();
		c_key0=0;
		key1=0;
		key2=0;
		LPC_TIM0->TCR =0;
		NVIC_EnableIRQ(EINT1_IRQn);	//Habilitar Key 1
		NVIC_EnableIRQ(EINT2_IRQn);	//Habilitar Key 2
	}
 }
	else {
		if(c_key0==0){
			
		SysTick->CTRL=0x4;	//Stop Systick
			LPC_TIM0->TCR =0;
			c_key0=1;
		}
		else {
			c_key0=0;
			SysTick->CTRL=0x7; // Start
			LPC_TIM0->TCR =1;
		}
	}
}
void EINT1_IRQHandler(void){
	
  LPC_SC->EXTINT|=2;  //borrar flag

	key1=1;					// PULSAR / SOLTAR
 
}
void EINT2_IRQHandler(void){
  LPC_SC->EXTINT|=4; //borrar flag
	
  key2=1;

  if((key2==1)&&(mode==0)){
   
    //drawString(30,230,"- 10 grados", RED, BLACK, LARGE);  
    drawString(30,270,"Mantener pulsado Key1 ", CYAN, BLACK, SMALL); 
    drawString(30,290,"para modo automatico", CYAN, BLACK, SMALL); 
  }
}

void TIMER1_IRQHandler(void)
{
	static uint16_t indice_muestra;
	LPC_TIM1->IR|= (1<<0); 			// borrar flag
	if(dis<Umbral_dist){
	LPC_DAC->DACR= muestras[indice_muestra++] << 6; // bit6..bit15 
	if(indice_muestra==N_muestras-1) contperiodo++;
	indice_muestra&= N_muestras-1;		// contador circular (Si N_muestras potencia de 2) 
	}
		
}	
void TIMER0_IRQHandler(void)
{
	
		LPC_TIM0->IR|= (1<<0); 	// borrar flag
  	LPC_TIM0->MR0 = periodomov-1;
		contador2=0;
	
		MedidaIR();
	sprintf(dm1," %d ",angulo);
	sprintf(dm," :%i cm",dis);
	
			if(dis<=Umbral_dist && dis>=30) LPC_TIM1->TCR=1; //Suena
			else{ 
				LPC_TIM1->TCR=0; //No suena
				contperiodo=0;
			}
			
			//// Dibujar Sonar////
			if(mode==1){
			drawString(20,120,"Distancia:", WHITE, BLACK, MEDIUM);
			drawString(100, 120,dm, RED, BLACK, MEDIUM);    //primeros dígitos (48=0, 49=1, 50=2....)
			drawString(20,132,"Angulo:", WHITE, BLACK, MEDIUM);
			drawString(100, 132,dm1, RED, BLACK, MEDIUM);    //primeros dígitos (48=0, 49=1, 50=2....)
			drawRect( 5, 160, 230, 140, RED);
			iter++;
			if(iter==((360/resoluciongrad)-1)) {
				iter=0;
				angulo=0;
			}

		if(sentido)	angulo=360-resoluciongrad*iter;
			else angulo=resoluciongrad*iter;

			y1=dis*sin(angulo*3.1415/180);
			x1=dis*cos(angulo*3.1415/180);
			drawLine(120, 230, 120+x1, 230-y1, GREEN);
		}
	else{
	
			drawString(20,230,"Distancia:", WHITE, BLACK, MEDIUM);
			drawString(100, 230,dm, RED, BLACK, MEDIUM);    //primeros dígitos (48=0, 49=1, 50=2....)			
			
	   }
			////////////////
	
}	


int main(void)
{

//Pines motor: 4 5 6 7
	
	genera_muestras(N_muestras);
	
	if((LPC_GPIO2->FIOPIN<<20)>>31) mode=0; // Lee puerto Key1
	else mode=1;
	LPC_TIM1->TCR=0;
	LCD();
	init_DAC();
	init_TIMER1();
	init_TIMER0();
  config_IRQ();
  LPC_GPIO0->FIODIR |= (1<<4) | (1<<5) | (1<<6) | (1<<7); //P0.4..0.7: 4 salidas hacia el driver del motor
  NVIC_SetPriorityGrouping(2); //32 niveles de prioridad preemptive (sin subprioridad)
  SysTick_Config(SystemCoreClock/Ftick); //Valor de RELOAD (nº de cuentas del SYSTICK hasta llegar a cero)
	ADCconfig();
 
	ptr_rx=buffer;                    // inicializa el puntero de recepción al comienzo del buffer
  uart0_init(9600);                                // configura la UART0 a 9600 baudios, 8 bits, 1 bit stop
	
	b_ptr_rx=b_buffer;                    // inicializa el puntero de recepción al comienzo del buffer
  uart2_init(9600);      	// configura la UART2 a 9600 baudios, 8 bits, 1 bit stop
	if(mode==1){
			tx_cadena_UART2("Estas en modo automatico\n\r"
										"1. xxg (introduce la resolucion en grados xx, seguido de g y terminado en Enter: 10g, 15g, 20g)\n\r"
										"2. xxxms (introduce el periodo en milisegundos xxx, seguido de ms y terminado en Enter: 200ms, 400ms, 600ms)\n\r"
										"3. h (Para volver a mostrar este mensaje de ayuda, pulsar h terminado en Enter)\n\r");	
						}
while(1){
	
	
	
	/// Bluetooth UART
if(b_rx_completa==0){
	 b_rx_completa=0;
		//b_message=atoi(&b_buffer[0]);
		do{
			if(b_rx_completa){ // Comprabamos la llegada de una cadena por RXD
			b_rx_completa=0; // Borrar flag para otra recepción
			if (strcmp (b_buffer, "10g\r") == 0) {
			tx_cadena_UART2("Has seleccionado 10g de resolucion\n\r");
			resoluciongrad=10;
			}
			else if (strcmp (b_buffer, "15g\r") == 0){
			tx_cadena_UART2("Has seleccionado 15g de resolucion\n\r");
			resoluciongrad=15;
			}
			else if (strcmp (b_buffer, "20g\r") == 0){
			tx_cadena_UART2("Has seleccionado 20g de resolucion\n\r");
			resoluciongrad=20;
			}
					else if (strcmp (b_buffer, "200ms\r") == 0){
			tx_cadena_UART2("Has seleccionado 200ms de resolucion\n\r");
			 periodomov=(F_cpu*0.2/25); // lo paso a Ncuentas
			
			}
			 else if (strcmp (b_buffer, "400ms\r") == 0){
			tx_cadena_UART2("Has seleccionado 400ms de resolucion\n\r");
			 periodomov=(F_cpu*0.4/25); // lo paso a Ncuentas
			
			}
			else if (strcmp (b_buffer, "600ms\r") == 0){
			tx_cadena_UART2("Has seleccionado 600ms de resolucion\n\r");
			 periodomov=(F_cpu*0.6/25); // lo paso a Ncuentas
			

			}
			else if (strcmp (b_buffer, "h\r") == 0){
			tx_cadena_UART2("Estas en modo automatico\n\r"
				"Introduce la resolucion en grados xx, seguido g y teminado Enter: 10g, 15g ,20g\n\r"
				 "Periodo en milisegundos xxx, seguido de ms y terminado en Enter: 200ms, 400ms 600ms\n\r"
				 "Para volver a mostrar este mensaje de ayuda pulsar h seguido de Enter:\n\r");
			}
			else tx_cadena_UART2("Comando erroneo\n\r");
			}
			}while(mode==1);
		}
	/// PC
if(rx_completa==0){	// Espera introducir una cadena de caracteres terminada con CR (0x0D)
	rx_completa=0; // Borrar flag
			do{
			if(rx_completa){ // Comprabamos la llegada de una cadena por RXD
			rx_completa=0; // Borrar flag para otra recepción
			if (strcmp (buffer, "10g\r") == 0) {
			tx_cadena_UART0("Has seleccionado 10g de resolucion\n\r");
			resoluciongrad=10;
			}
			else if (strcmp (buffer, "15g\r") == 0){
			tx_cadena_UART0("Has seleccionado 15g de resolucion\n\r");
			resoluciongrad=15;
			}
			else if (strcmp (buffer, "20g\r") == 0){
			tx_cadena_UART0("Has seleccionado 20g de resolucion\n\r");
			resoluciongrad=20;
			}
					else if (strcmp (buffer, "200ms\r") == 0){
			tx_cadena_UART0("Has seleccionado 200ms de resolucion\n\r");
			 periodomov=(F_cpu*0.2/25); // lo paso a Ncuentas
			
			}
			 else if (strcmp (buffer, "400ms\r") == 0){
			tx_cadena_UART0("Has seleccionado 400ms de resolucion\n\r");
			 periodomov=(F_cpu*0.4/25); // lo paso a Ncuentas
			
			}
			else if (strcmp (buffer, "800ms\r") == 0){
			tx_cadena_UART0("Has seleccionado 800ms de resolucion\n\r");
			 periodomov=(F_cpu*0.8/25); // lo paso a Ncuentas
			

			}
			else if (strcmp (buffer, "h\r") == 0){
			tx_cadena_UART0("Estas en modo automatico\n\r"
				"Introduce la resolucion en grados xx, seguido g y teminado Enter: 10g, 15g ,20g\n\r"
				 "Periodo en milisegundos xxx, seguido de ms y terminado en Enter: 200ms, 400ms 600ms\n\r"
				 "Para volver a mostrar este mensaje de ayuda pulsar h seguido de Enter:\n\r");
			}
			else tx_cadena_UART0("Comando erroneo\n\r");
			}
			}while(mode==1);
			

		}

		
		}
	
}

void SysTick_Handler(void) //Se ejecuta priodicamente a Ftick (Hz)
{
  //Contador circular de 8 pasos
	i&=7;
	resolucion=(4096)/(360/resoluciongrad); //Convierte de grados a pasos
	
/*=====================================|MODO AUTOMÁTICO|========================================*/
  if(mode==1){		
		
    contador2++;
	
		if(contador2<resolucion){
			 i&=7;
		contador1++;
		if(sentido) LPC_GPIO0->FIOPIN=(secuencia1[i++]<<4); //Giro Horario
		else LPC_GPIO0->FIOPIN=(secuencia1[i--]<<4);        //Giro Antihorario
			
		if(contador1%(4096-1)==0) {
				LPC_TIM0->TCR=1;//Una vuelta entera (4096 pasos)
		sentido ^=1; // Cambiar de sentido
		fillRect( 5, 160, 230, 140, BLACK);
		contador1=0;	
		 }
		}
		else {
			LPC_TIM0->TCR=1;
		}

    
	}

/*==============================================================================================*/
	
 /*=======================================|MODO MANUAL|=========================================*/   
  if((key1==1)&&(mode==0)){
  

    if(contador1<=resolucion){
      
      LPC_GPIO0->FIOPIN=(secuencia1[i++]<<4); //Giro horario
      contador1++;
    }
    else{
			MedidaIR(); //mide
      key1=0;	
      contador1=0;


    }
  }
  if((key2==1)&&(mode==0)){
    
    if(contador1<=resolucion){
      
      LPC_GPIO0->FIOPIN=(secuencia1[i--]<<4); //Giro antihorario
      contador1++;
    }
    else{
			MedidaIR(); //mide
      key2=0;
      contador1=0;

		}
      
    }
	/*============================================================================================*/
	
}