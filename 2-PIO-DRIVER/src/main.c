/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)






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

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

void _pio_set(Pio *pio, const uint32_t mask);
 
void _pio_clear(Pio *pio, const uint32_t mask);

void _pio_pull_up(Pio *pio, const uint32_t mask, const uint32_t pull_up_enable);

void _pio_set_input(Pio *pio, const uint32_t mask,const uint32_t attribute);

void _pio_set_output(Pio *pio, const uint32_t mask, const uint32_t ul_default_level, const uint32_t ul_multidrive_enable, const uint32_t ul_pull_up_enable);

int _pio_get(Pio *pio, const pio_type_t ul_type, const uint32_t mask);

void _ms_delay(int ms);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

_pio_set(Pio *p_pio, const uint32_t mask){
	p_pio -> PIO_SODR = mask;
}
_pio_clear(Pio *p_pio, const uint32_t mask){
	p_pio -> PIO_CODR = mask;
}

_pio_pull_up(Pio *p_pio,const uint32_t mask, const uint32_t pull_up_enable){
	if(pull_up_enable){
		p_pio->PIO_PUER = mask;
		return;
	}
	p_pio->PIO_PUDR = mask;
}

_pio_set_input(Pio *p_pio, const uint32_t mask,const uint32_t attribute){
	if(attribute & _PIO_PULLUP){ _pio_pull_up(p_pio, mask, _PIO_PULLUP); }
	if(attribute & _PIO_DEBOUNCE){ p_pio->PIO_IFSCER = mask; p_pio->PIO_IFER = mask; }
	p_pio->PIO_IFDR = mask;
}

_pio_set_output(Pio *p_pio, const uint32_t mask, const uint32_t ul_default_level, const uint32_t ul_multidrive_enable, const uint32_t ul_pull_up_enable){
	p_pio->PIO_OER = mask;
	p_pio->PIO_PER = mask;
	if(ul_multidrive_enable){ p_pio->PIO_MDER = mask;}
	else { p_pio-> PIO_MDDR = mask ;}
	if(ul_default_level){
		p_pio->PIO_SODR = mask;
	} else { p_pio->PIO_CODR = mask; }
}

_pio_get(Pio *p_pio, const pio_type_t ul_type, const uint32_t mask){
	uint32_t reg_value;
	if(ul_type == PIO_OUTPUT_0 || ul_type == PIO_OUTPUT_1){ reg_value = p_pio->PIO_ODSR; }
	else{ reg_value = p_pio->PIO_PDSR; }
	if(!(reg_value & mask)){
		return 0;
	} return 1;
}

_delay_ms(int time_ms){
	//clock setado para 30MHz - 1 ciclo da cpu = 1.5*10**-5 ms
	for(int i = 0; i < time_ms*150000; i++){
		asm("NOP");
	}
}

// Função de inicialização do uC
void init(void)
{
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(LED1_PIO_ID);
	_pio_set_output(LED1_PIO,LED1_PIO_IDX_MASK,0,0,0);
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	_pio_set_output(LED2_PIO,LED2_PIO_IDX_MASK,0,0,0);

	pmc_enable_periph_clk(LED3_PIO_ID);
	_pio_set_output(LED3_PIO,LED3_PIO_IDX_MASK,0,0,0);

	pmc_enable_periph_clk(BUT1_PIO_ID);
	_pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_pull_up(BUT1_PIO, BUT1_PIO_IDX_MASK, 1);
	
	pmc_enable_periph_clk(BUT2_PIO_ID);
	_pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_pull_up(BUT2_PIO, BUT2_PIO_IDX_MASK, 1);

	pmc_enable_periph_clk(BUT3_PIO_ID);
	_pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_pull_up(BUT3_PIO, BUT3_PIO_IDX_MASK, 1);

}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	if (! _pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)){
		for(int i = 0; i< 5; i++){
			_pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
			_delay_ms(300);
			_pio_set(LED1_PIO,LED1_PIO_IDX_MASK);
			delay_ms(300);
		}
	}
	if (! _pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)){
		for(int i = 0; i< 5; i++){
			_pio_clear(LED2_PIO,LED2_PIO_IDX_MASK);
			delay_ms(300);
			_pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			delay_ms(300);
		}
	}
	if (! pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)){
		for(int i = 0; i< 5; i++){
			_pio_clear(LED3_PIO,LED3_PIO_IDX_MASK);
			delay_ms(300);
			_pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			delay_ms(300);
		}
	}
  }
  return 0;
}
