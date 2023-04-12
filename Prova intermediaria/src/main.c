#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "rtt.h"

/* Botao da placa */
#define BUT_PIO			 PIOD
#define BUT_PIO_ID       ID_PIOD
#define BUT_PIO_IDX      28
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_IDX)

#define BUT2_PIO          PIOC
#define BUT2_PIO_ID       ID_PIOC
#define BUT2_PIO_IDX      31
#define BUT2_PIO_IDX_MASK (1 << BUT2_PIO_IDX)

#define BUT3_PIO          PIOA
#define BUT3_PIO_ID       ID_PIOA
#define BUT3_PIO_IDX      19
#define BUT3_PIO_IDX_MASK (1 << BUT3_PIO_IDX)

#define LED1_PIO          PIOA
#define	LED1_PIO_ID       ID_PIOA
#define LED1_PIO_IDX      0
#define LED1_PIO_IDX_MASK (1 <<	LED1_PIO_IDX)


#define CLK_PIO			 PIOA
#define CLK_PIO_ID       ID_PIOA
#define CLK_PIO_IDX      3
#define CLK_PIO_IDX_MASK (1 << CLK_PIO_IDX)

#define DT_PIO			 PIOA
#define DT_PIO_ID       ID_PIOA
#define DT_PIO_IDX      2
#define DT_PIO_IDX_MASK (1 << DT_PIO_IDX)

#define SW_PIO			PIOC
#define SW_PIO_ID       ID_PIOC
#define SW_PIO_IDX      19
#define SW_PIO_IDX_MASK (1 << SW_PIO_IDX)


/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
static void LED_init(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
volatile char passwd_iter;
volatile char but_flag;
volatile char but2_flag;
volatile char but3_flag;

volatile char direita_flag = 0;
volatile char esquerda_flag = 0;
volatile char iter = 0;
volatile char pisca = 0;
volatile char rtt_flag = 0;
volatile char zera_flag = 0;


void but_callback(void) {
	printf("callback 1 \n");
	iter++;
	if (iter > 3)
		iter = 0;
	if (iter < 0)
		iter = 3;
}

void but2_callback(void) {
	printf("callback 2 \n");
	but2_flag = 1;
}

void but3_callback(void) {
	printf("callback 3 \n");
	but3_flag = 1;
}

volatile char sw_flag = 0;
void sw_interrupt(void){
	if (!pio_get(SW_PIO, PIO_DEFAULT, SW_PIO_IDX_MASK)){
		RTT_init(1000,5*1000,0);
	} else{
		if(rtt_read_timer_value(RTT) > 5000){
			rtt_flag = 1;
			zera_flag = 1;
		}
		else{
			iter++;
			if (iter > 3)
				iter = 0;
		}
	}
}

volatile char clk_flag = 0;
volatile char dt_flag = 0;

void clk_interrupt(void){
	printf("clk");
	clk_flag = 1;
	if (dt_flag){
		esquerda_flag = 1;
		clk_flag = 0;
		dt_flag = 0;
	}
}

void dt_interrupt(void){
	printf("dt");
	dt_flag = 1;
	if (clk_flag){
		direita_flag =1;
		clk_flag = 0;
		dt_flag = 0;
	}
}
/**************************************************
ALARME
***************************************************/

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		printf("DEU O TEMPO");
		if(!pio_get(SW_PIO,PIO_INPUT, SW_PIO_IDX_MASK)){
			rtt_flag = 1;
		}
	}
}


static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	BUT_init();
	gfx_mono_ssd1306_init();
    gfx_mono_draw_string("Avaliacao intermediaria", 0, 16, &sysfont);
//   gfx_mono_draw_string("oii", 0, 20, &sysfont);
	char numbers[4] = {0,0,0,0};
	char msg[14];
	sprintf(msg, "0x%x000",0);
// 	char numero = 0;
	for (;;)  {
		if(zera_flag){
			for(int i = 0; i <= 3; i++){
				numbers[i] = 0;
			}
			zera_flag = 0;
		}
		if (direita_flag){
			numbers[iter]++;
			direita_flag = 0;
			if (numbers[iter] >= 16)
				numbers[iter] = 0;
		}
		if (esquerda_flag){
			numbers[iter]--;
			esquerda_flag = 0;
			if (numbers[iter] >= 16)
				numbers[iter] = 15;
		}
		sprintf(msg, "0x %x %x %x %x   ",numbers[0],numbers[1],numbers[2],numbers[3]);
		

		if (pisca){
// 			printf("era pra piscar");
			gfx_mono_draw_string("_ ", 18+11*iter, 20, &sysfont);
		}
		else
			gfx_mono_draw_string(msg, 0, 20, &sysfont);
		vTaskDelay(1000/60/portTICK_PERIOD_MS);
	}
}

static void task_direcao(void *pvParameters) {
	// o nome eh meme, na verdade eh a task do pisca
	printf("task direcao \n");
// 	char pisca = 0;
	for (;;)  {
		if (!pisca){
// 			printf("pisca");
			pisca = 1;
		}
		else{
			pisca = 0;
// 			printf("não pisca");
		}
		vTaskDelay(500/portTICK_PERIOD_MS);
	}
}

static void task_pisca_led(void *pvParameters){
	int i = 1;
// 	pio_set(LED1_PIO,LED1_PIO_IDX_MASK);
	printf("task led");
	LED_init();
	pio_set(LED1_PIO,LED1_PIO_IDX_MASK);

	for(;;){
		if (rtt_flag){
			printf("flag ativada");
			if (i < 21){
				if (i % 2){
  					pio_clear(LED1_PIO,LED1_PIO_IDX_MASK);
					  printf("pisca");
				}
				else{
  					pio_set(LED1_PIO,LED1_PIO_IDX_MASK);
				}
				i++;
			}
			else{
				i = 0;
				rtt_flag = 0;
			}
		}
		vTaskDelay(50/portTICK_PERIOD_MS);
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	/* configura prioridae */

	/* conf botão como entrada */
// 	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
// 	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
// 	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
// 	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_LOW_LEVEL , but_callback);
// 	
// 	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
// 	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
// 	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
// 	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_LOW_LEVEL , but2_callback);
// 
// 	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
// 	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
// 	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
// 	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_LOW_LEVEL , but3_callback);
// 
// 	NVIC_EnableIRQ(BUT_PIO_ID);
// 	NVIC_EnableIRQ(BUT2_PIO_ID);
// 	NVIC_EnableIRQ(BUT3_PIO_ID);
// 
// 	NVIC_SetPriority(BUT_PIO_ID, 4);
// 	NVIC_SetPriority(BUT2_PIO_ID, 4);
// 	NVIC_SetPriority(BUT3_PIO_ID, 4);
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);


	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP);



	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_PIO_PIN_MASK,
	PIO_IT_FALL_EDGE,
	but_callback);
	
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_PIO_IDX_MASK,
	PIO_IT_LOW_LEVEL,
	but2_callback);
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_PIO_IDX_MASK,
	PIO_IT_LOW_LEVEL,
	but3_callback);


	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);


	pio_get_interrupt_status(BUT_PIO);
	pio_get_interrupt_status(BUT2_PIO);
	pio_get_interrupt_status(BUT3_PIO);

	
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);


	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)

	
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_EnableIRQ(BUT3_PIO_ID);


	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
	NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 3
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
	
	////
		pmc_enable_periph_clk(CLK_PIO_ID);
		pmc_enable_periph_clk(DT_PIO_ID);
		pmc_enable_periph_clk(SW_PIO_ID);


		// Configura PIO para lidar com o pino do botão como entrada
		// com pull-up
		pio_configure(CLK_PIO, PIO_INPUT, CLK_PIO_IDX_MASK, PIO_DEBOUNCE | PIO_PULLUP);
		
		pio_configure(DT_PIO, PIO_INPUT, DT_PIO_IDX_MASK, PIO_DEBOUNCE | PIO_PULLUP);
		
		pio_configure(SW_PIO, PIO_INPUT, SW_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);



		// Configura interrupção no pino referente ao botao e associa
		// função de callback caso uma interrupção for gerada
		// a função de callback é a: but_callback()
		pio_handler_set(CLK_PIO,
		CLK_PIO_ID,
		CLK_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		clk_interrupt);
		
		pio_handler_set(DT_PIO,
		DT_PIO_ID,
		DT_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		dt_interrupt);
		
		pio_handler_set(SW_PIO,
		SW_PIO_ID,
		SW_PIO_IDX_MASK,
		PIO_IT_EDGE,
		sw_interrupt);


		// Ativa interrupção e limpa primeira IRQ gerada na ativacao
		pio_enable_interrupt(CLK_PIO, CLK_PIO_IDX_MASK);
		pio_enable_interrupt(DT_PIO, DT_PIO_IDX_MASK);
		pio_enable_interrupt(SW_PIO, SW_PIO_IDX_MASK);


		pio_get_interrupt_status(CLK_PIO);
		pio_get_interrupt_status(DT_PIO);
		pio_get_interrupt_status(SW_PIO);

		
		pio_set_debounce_filter(CLK_PIO, CLK_PIO_IDX_MASK, 60);
		pio_set_debounce_filter(DT_PIO, DT_PIO_IDX_MASK, 60);
		pio_set_debounce_filter(SW_PIO, SW_PIO_IDX_MASK, 60);


		
		// Configura NVIC para receber interrupcoes do PIO do botao
		// com prioridade 4 (quanto mais próximo de 0 maior)

		
		NVIC_EnableIRQ(CLK_PIO_ID);
		NVIC_EnableIRQ(DT_PIO_ID);
		NVIC_EnableIRQ(SW_PIO_ID);


		NVIC_SetPriority(CLK_PIO_ID, 4); // Prioridade 4
		NVIC_SetPriority(DT_PIO_ID, 4); // Prioridade 3
		NVIC_SetPriority(SW_PIO_ID, 4); // Prioridade 4

}

static void LED_init(void){
		//agora ligando o led
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO,LED1_PIO_IDX_MASK,0,0,0);

}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_direcao, "direcao", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create direacao task\r\n");
	}
	
	if (xTaskCreate(task_pisca_led, "piscaled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create piscaled task\r\n");
	}


	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
