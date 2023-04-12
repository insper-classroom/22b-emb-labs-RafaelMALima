#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

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

#define B1 0
#define B2 1
#define B3 2
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

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
volatile char passwd_iter;
volatile char but_flag;
volatile char but2_flag;
volatile char but3_flag;


void but_callback(void) {
	printf("callback 1 \n");
	but_flag = 1;
}

void but2_callback(void) {
	printf("callback 2 \n");
	but2_flag = 1;
}

void but3_callback(void) {
	printf("callback 3 \n");
	but3_flag = 1;
}



/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	BUT_init();
	gfx_mono_ssd1306_init();
    gfx_mono_draw_string("Cofre trancado", 0, 0, &sysfont);
//   gfx_mono_draw_string("oii", 0, 20, &sysfont);
	char passwd[4] = {B1, B1, B2, B3};
	char passwd_buffer[4];
	char isRight;
	printf("cofre trancado \n");
	for (;;)  {
		//printf("esperando input \n");
		if (but_flag){
			passwd_buffer[passwd_iter] = B1;
			passwd_iter++;
		}
		if (but2_flag){
			passwd_buffer[passwd_iter] =B2;
			passwd_iter++;
		}
		if (but3_flag){
			passwd_buffer[passwd_iter] =B3;
			passwd_iter++;
		}
		if (but3_flag || but2_flag || but_flag){
			gfx_mono_draw_string("*", passwd_iter*10, 20, &sysfont);
			but_flag = 0; but2_flag = 0; but3_flag = 0;
		}
		if (passwd_iter == 4){
			isRight = 1;
			for (int i = 0; i < passwd_iter; i++){
				if (passwd[i] != passwd_buffer[i]){
					isRight == 0;
					gfx_mono_draw_string("Deu ruim", 0, 0, &sysfont);
				}
			}
			if (isRight)
				gfx_mono_draw_string("Cofre aberto", 0, 0, &sysfont);
			passwd_iter = 0;
		}
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
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP);
	
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP);



	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_PIO_PIN_MASK,
	PIO_IT_LOW_LEVEL,
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

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
