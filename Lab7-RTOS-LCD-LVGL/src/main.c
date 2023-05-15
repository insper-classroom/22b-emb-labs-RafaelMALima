/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"
#include "rtt.h"


/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

LV_FONT_DECLARE(dseg70);

LV_FONT_DECLARE(dseg50);

LV_FONT_DECLARE(dseg35);

LV_FONT_DECLARE(Clock);

#define my_clock "\xEF\x80\x97"


#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)


  
/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

static lv_obj_t * label_power_btn;
static lv_obj_t * label_menu_btn;
static lv_obj_t * label_clock_btn;

static lv_obj_t * label_up_btn;
static lv_obj_t * label_dn_btn;

lv_obj_t * labelFloor;
lv_obj_t * labelSetValue;
lv_obj_t * labelClock;

SemaphoreHandle_t xMutexLVGL;

volatile char ref_temp = 23;
volatile char secs = 00;
volatile char mins = 00;
volatile char hrs = 00;

volatile char power = 1;
volatile char clck = 0;


/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }
	
int main(void);
	
	
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

void lv_termostato(void);

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

//handlers
static void up_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
	int temp;
	if(code == LV_EVENT_CLICKED) {
		if (clck == 0){
			c = lv_label_get_text(labelSetValue);
			temp = atoi(c);
			lv_label_set_text_fmt(labelSetValue, "%02d", temp + 1);
		}
		if (clck == 1) mins++;
		if (clck == 2) hrs++;
	}
}

//handlers
static void down_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
	int temp;
	if(code == LV_EVENT_CLICKED) {
		if (clck == 0){
		c = lv_label_get_text(labelSetValue);
		temp = atoi(c);
		lv_label_set_text_fmt(labelSetValue, "%02d", temp - 1);
		}
		if (clck == 1) mins--;
		if (clck == 2) hrs--;
	}
}

static void pwr_handler(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);
	if (code == LV_EVENT_CLICKED){
		lv_obj_clean(lv_scr_act());
		if (power) power = 0;
		else power = 1;
		lv_termostato();
	}
}

static void RTT_init(float freqPrescale,       uint32_t IrqNPulses, uint32_t rttIRQSource) {
// 	printf("RTT INIT");

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

void RTT_Handler( void ){
	printf("entrou no timer");
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		printf("\n ESTOROU O TIMER \n");
		secs++;
		if (secs >= 60){
			mins ++;
			secs = 0;
			if (mins >= 60){
				mins = 0;
				hrs++;
			}
		RTT_init(1,1000,0);
	}
	if (secs % 2 == 0){
		lv_label_set_text_fmt(labelClock, "%02d %02d %02d", hrs,mins,secs);
	}
	else{
		lv_label_set_text_fmt(labelClock, "%02d:%02d:%02d", hrs,mins,secs);

	}
	}
}

static void clock_btn_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) {
		clck++;
		if (clck > 2) clck = 0;
	}

}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void event_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
		printf("clicado");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

void lv_ex_btn_1(void) {
	lv_obj_t * label;

	lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);

	label = lv_label_create(btn1);
	lv_label_set_text(label, "Corsi");
	lv_obj_center(label);

	lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
	lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_set_height(btn2, LV_SIZE_CONTENT);

	label = lv_label_create(btn2);
	lv_label_set_text(label, "Toggle");
	lv_obj_center(label);
}

void lv_termostato(void) {
    lv_obj_t * labelBtn1;

//     lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
//     lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
//     lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
// 
//     labelBtn1 = lv_label_create(btn1);
//     lv_label_set_text(labelBtn1, "Teste");
//     lv_obj_center(labelBtn1);
	
	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_NONE));
	lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_NONE));
	lv_style_set_border_width(&style, 5);
	
// 	lv_obj_add_style(btn1, &style, 0);

	//POWER BUTTON
	lv_obj_t * power_btn = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(power_btn, pwr_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(power_btn, LV_ALIGN_CENTER, -125, +100);

	label_power_btn = lv_label_create(power_btn);
	lv_label_set_text(label_power_btn, "[  " LV_SYMBOL_POWER);
	lv_obj_center(label_power_btn);
	
	lv_obj_add_style(power_btn, &style, 0);
	if (power){
	//MENU BUTTON
	lv_obj_t * menu_btn = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(menu_btn, event_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(menu_btn, LV_ALIGN_CENTER, -65, +100);


	label_menu_btn = lv_label_create(menu_btn);
	lv_label_set_text(label_menu_btn, "|  M");
	lv_obj_center(label_menu_btn);
	lv_obj_add_style(menu_btn, &style, 0);
	
	//CLOCK BUTTON
	lv_obj_t * clock_btn = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(clock_btn, clock_btn_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(clock_btn, LV_ALIGN_CENTER, -5, +100);

	label_clock_btn = lv_label_create(clock_btn);
	lv_obj_set_style_text_font(label_clock_btn, &Clock, LV_STATE_DEFAULT);

	lv_label_set_text(label_clock_btn, "| " my_clock " ]");
	lv_obj_center(label_clock_btn);


	
	lv_obj_add_style(clock_btn, &style, 0);
// 
// 
	//DOWN BTN
 	lv_obj_t * down_btn = lv_btn_create(lv_scr_act());
 	lv_obj_add_event_cb(down_btn, down_handler, LV_EVENT_ALL, NULL);
 	lv_obj_align(down_btn, LV_ALIGN_CENTER, +125, +100);

 	label_dn_btn = lv_label_create(down_btn);
 	lv_label_set_text(label_dn_btn, LV_SYMBOL_DOWN "  ]");
 	lv_obj_center(label_dn_btn);
//
 	lv_obj_add_style(down_btn, &style, 0);


	//UP BUTTON
	lv_obj_t * up_btn = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(up_btn, up_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(up_btn, LV_ALIGN_CENTER, +65, +100);

	label_up_btn = lv_label_create(up_btn);
	lv_label_set_text(label_up_btn, "[  " LV_SYMBOL_UP);
	lv_obj_center(label_power_btn);
	
	lv_obj_add_style(up_btn, &style, 0);
	
	
	//label temp
	    labelFloor = lv_label_create(lv_scr_act());
	    lv_obj_align(labelFloor, LV_ALIGN_LEFT_MID, 35 , -10);
	    lv_obj_set_style_text_font(labelFloor, &dseg70, LV_STATE_DEFAULT);
	    lv_obj_set_style_text_color(labelFloor, lv_color_white(), LV_STATE_DEFAULT);
	    lv_label_set_text_fmt(labelFloor, "%02d", 23);

	//label temp reference
	labelSetValue = lv_label_create(lv_scr_act());
	lv_obj_align(labelSetValue, LV_ALIGN_RIGHT_MID, -25 , -10);
	lv_obj_set_style_text_font(labelSetValue, &dseg50, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelSetValue, lv_color_white(), LV_STATE_DEFAULT);
// 	char ref_temp_str[3];
// 	sprintf(ref_temp_str, "%d", ref_temp);
	lv_label_set_text_fmt(labelSetValue, "%02d", ref_temp);
	
		//label clock
		labelClock = lv_label_create(lv_scr_act());
		lv_obj_align(labelClock, LV_ALIGN_RIGHT_MID, -25 , -80);
		lv_obj_set_style_text_font(labelClock, &dseg35, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelClock, lv_color_white(), LV_STATE_DEFAULT);
		// 	char ref_temp_str[3];
		// 	sprintf(ref_temp_str, "%d", ref_temp);
		lv_label_set_text_fmt(labelClock, "%02d:%02d:%02d", hrs,mins,secs);

	//label temp
	lv_obj_t * a;

	a = lv_label_create(lv_scr_act());
	lv_obj_align_to(a,labelFloor, LV_ALIGN_BOTTOM_RIGHT, 50 , 0);
	lv_obj_set_style_text_font(a, &dseg35, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(a, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(a, ".4");

	}
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
	int px, py;

	lv_termostato();
	RTT_init(1000,1000,0);

	for (;;)  {
		xSemaphoreTake( xMutexLVGL, portMAX_DELAY );
		lv_tick_inc(50);
		lv_task_handler();
		xSemaphoreGive( xMutexLVGL );
		vTaskDelay(50);
		if(rtt_read_timer_value(RTT) > 1000){
			printf("\n ESTOROU O TIMER \n");
			secs++;
			if (secs >= 60){
				mins ++;
				secs = 0;
				if (mins >= 60){
					mins = 0;
					hrs++;
				}
			}
			RTT_init(1000,1000,0);
			if (secs % 2 == 0){
				lv_label_set_text_fmt(labelClock, "%02d %02d %02d", hrs,mins,secs);
			}
			else{
				lv_label_set_text_fmt(labelClock, "%02d:%02d:%02d", hrs,mins,secs);
			}
		}
	}
}

static void task_rtt(void *pvParameters){
	for (;;){
	}
}

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py))
		data->state = LV_INDEV_STATE_PRESSED;
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	
	data->point.x = px;
	data->point.y = py;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();
	
	xMutexLVGL = xSemaphoreCreateMutex();


	/* LCd, touch and lvgl init*/
	configure_lcd();
	configure_touch();
	configure_lvgl();

	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	
	/* Start the scheduler. */
// 	RTT_init(1000,1000,0);
	vTaskStartScheduler();

	while(1){ }
}
