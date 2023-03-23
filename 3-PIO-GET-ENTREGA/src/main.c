/************************************************************************
 * 5 semestre - Eng. da Computao - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Material:
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 *
 * Objetivo:
 *  - Demonstrar interrupção do PIO
 *
 * Periféricos:
 *  - PIO
 *  - PMC
 *
 * Log:
 *  - 10/2018: Criação
 ************************************************************************/

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"


/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED1_PIO          PIOA
#define	LED1_PIO_ID       ID_PIOA
#define LED1_PIO_IDX      0
#define LED1_PIO_IDX_MASK (1 <<	LED1_PIO_IDX)

#define BUT1_PIO          PIOD
#define BUT1_PIO_ID       ID_PIOD
#define BUT1_PIO_IDX      28
#define BUT1_PIO_IDX_MASK (1 << BUT1_PIO_IDX)

#define LED2_PIO          PIOC
#define	LED2_PIO_ID       ID_PIOC
#define LED2_PIO_IDX      30
#define LED2_PIO_IDX_MASK (1 <<	LED2_PIO_IDX)

#define BUT2_PIO          PIOC
#define BUT2_PIO_ID       ID_PIOC
#define BUT2_PIO_IDX      31
#define BUT2_PIO_IDX_MASK (1 << BUT2_PIO_IDX)

#define LED3_PIO          PIOB
#define	LED3_PIO_ID       ID_PIOB
#define LED3_PIO_IDX      2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)

#define BUT3_PIO          PIOA
#define BUT3_PIO_ID       ID_PIOA
#define BUT3_PIO_IDX      19
#define BUT3_PIO_IDX_MASK (1 << BUT3_PIO_IDX)



/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

volatile char but1_flag = 0;
volatile char but2_flag = 0;
volatile char but3_flag = 0;

/************************************************************************/
/* prototype                                                            */
/************************************************************************/
void io_init(void);
void pisca_led(int n, int t);

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

/*
 * Exemplo de callback para o botao, sempre que acontecer
 * ira piscar o led por 5 vezes
 *
 * !! Isso é um exemplo ruim, nao deve ser feito na pratica, !!
 * !! pois nao se deve usar delays dentro de interrupcoes    !!
 */
void but1_callback(void)
{
  but1_flag = 1;
}

void but2_callback(void){
	but2_flag = !but2_flag;
}

void but3_callback(void){
	but3_flag = 1;
}
/************************************************************************/
/* funções                                                              */
/************************************************************************/

// pisca led N vez no periodo T
void pisca_led(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
    for(int i=40;i<=120;i+=2){
		if (but1_flag || but2_flag || but3_flag){ return; }
	    gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_SET);
		gfx_mono_draw_filled_circle(20,16,(40+i)*12/(120),GFX_PIXEL_SET, GFX_WHOLE);
	    delay_ms(t/(120-40));
	}

    pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
    for(int i=40;i<=120;i+=2){
		if (but1_flag || but2_flag || but3_flag){ return; }
	    gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_CLR);
		gfx_mono_draw_filled_circle(20,16,(i+40)*12/(120),GFX_PIXEL_CLR, GFX_WHOLE);
	    delay_ms(t/(120-40));
    }
  }
}

// Inicializa botao SW0 do kit com interrupcao
void io_init()
{
  // Configura led
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);


  // Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);


  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP);


  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUT1_PIO,
                  BUT1_PIO_ID,
                  BUT1_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  but1_callback);
				  
  pio_handler_set(BUT2_PIO,
				  BUT2_PIO_ID,
				  BUT2_PIO_IDX_MASK,
				  PIO_IT_FALL_EDGE,
				  but2_callback);
				 
				  
  pio_handler_set(BUT3_PIO,
				  BUT3_PIO_ID,
				  BUT3_PIO_IDX_MASK,
				  PIO_IT_FALL_EDGE,
				  but3_callback);


  // Ativa interrupção e limpa primeira IRQ gerada na ativacao
  pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
  pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
  pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);


  pio_get_interrupt_status(BUT1_PIO);
  pio_get_interrupt_status(BUT2_PIO);
  pio_get_interrupt_status(BUT3_PIO);

  
  pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 40);
  pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 40);
  pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 40);


  
  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)

  
  NVIC_EnableIRQ(BUT1_PIO_ID);
  NVIC_EnableIRQ(BUT2_PIO_ID);
  NVIC_EnableIRQ(BUT3_PIO_ID);


  NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
  NVIC_SetPriority(BUT2_PIO_ID, 4);
  NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4

  // vou ficar maluco o botao 2 nao vai
  //pmc_enable_periph_clk(BUT2_PIO_ID);
  
  //pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  //pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
  //pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, but2_callback);
  //pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
  //NVIC_EnableIRQ(BUT2_PIO_ID);
  //NVIC_SetPriority(BUT2_PIO_ID,4);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
		board_init();
		sysclk_init();
		delay_init();

		gfx_mono_ssd1306_init();
		
		
		gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
			
	// Inicializa clock

	// Desativa watchdog
	//WDT->WDT_MR = WDT_MR_WDDIS;

    // configura botao com interrupcao
    io_init();

	// super loop
	// aplicacoes embarcadas no devem sair do while(1).
	
	int delay = 500;
	char freq[20];
	while(1)
  {
	  if (delay < 100){ delay = 100; }
	  sprintf(freq, "%.2f Hz", (float)1000/delay);
	  gfx_mono_draw_string(freq,50,16,&sysfont);
	  pisca_led(10,delay);	  

	  if (but1_flag){
		  delay_ms(500);
		  if(!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)){ delay += 40; }
		  else { delay -= 40; }
		  but1_flag = 0;
	  }
	  //if(!pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)){ while(1){ ;  }
	  while(but2_flag){ ; } 
	  if(but3_flag){
	  	  delay_ms(500);
	  	  if(!pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)){ delay -= 40; }
		  else { delay += 40; }
	  	  but3_flag = 0;
		  }
	}
}
