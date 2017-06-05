#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

/**
 * LEDs
 */
#define LED_PIO_ID		  ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		      8
#define LED_PIN_MASK    (1<<LED_PIN)

//pinos da ponte H
#define PIN1_PIO_ID		ID_PIOD
#define PIN1_PIO		PIOD
#define PIN1_PIN		30
#define PIN1_PIN_MASK	(1 << PIN1_PIN)

#define PIN2_PIO_ID		ID_PIOA
#define PIN2_PIO		PIOA
#define PIN2_PIN		6
#define PIN2_PIN_MASK	(1 << PIN2_PIN)

#define PIN3_PIO_ID		ID_PIOC
#define PIN3_PIO		PIOC
#define PIN3_PIN		19
#define PIN3_PIN_MASK	(1 << PIN3_PIN)

#define PIN4_PIO_ID		ID_PIOA
#define PIN4_PIO		PIOA
#define PIN4_PIN		2
#define PIN4_PIN_MASK	(1 << PIN4_PIN)


/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

#define STRING_EOL    "\r"
#define STRING_HEADER "-- PWM LED Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** 
 *  USART
 */

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
  
  /* buffer para recebimento de dados */
  uint8_t bufferRX[100];
  uint8_t bufferTX[100];
volatile uint8_t flag_led0 = 1;
volatile uint32_t flag;

/** PWM channel instance for LEDs */
volatile pwm_channel_t g_pwm_channel_led;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void LED_init(int estado);

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

/**
 * \brief Configure UART console.
 * BaudRate : 115200
 * 8 bits
 * 1 stop bit
 * sem paridade
 */

static void configure_console(void)
{

  /* Configura USART1 Pinos */
  sysclk_enable_peripheral_clock(ID_PIOB);
  sysclk_enable_peripheral_clock(ID_PIOA);
  pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
  pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
 	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
 
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief Configure UART console.
 */
static void USART0_init(void){
  
  /* Configura USART0 Pinos */
  sysclk_enable_peripheral_clock(ID_PIOB);
  pio_set_peripheral(PIOB, PIO_PERIPH_C, PIO_PB0);
  pio_set_peripheral(PIOB, PIO_PERIPH_C, PIO_PB1);
  
  /* Configura opcoes USART */
  const sam_usart_opt_t usart_settings = {
    .baudrate     = 9600,
    .char_length  = US_MR_CHRL_8_BIT,
    .parity_type  = US_MR_PAR_NO,
    .stop_bits    = US_MR_NBSTOP_1_BIT,
    .channel_mode = US_MR_CHMODE_NORMAL
  };

  /* Ativa Clock periferico USART0 */
  sysclk_enable_peripheral_clock(ID_USART0);
  
  /* Configura USART para operar em modo RS232 */
  usart_init_rs232(USART0, &usart_settings, sysclk_get_peripheral_hz());
  //usart_enable_interrupt(USART0, US_IER_RXRDY);
  //NVIC_EnableIRQ(ID_USART0);
  
  /* Enable the receiver and transmitter. */
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

 }



/**
 *  Envia para o UART uma string
 */
uint32_t usart_putString(uint8_t *pstring){
  uint32_t i = 0 ;

  while(*(pstring + i)){
    usart_serial_putchar(USART0, *(pstring+i++));
    while(!uart_is_tx_empty(USART0)){};
  }    
     
  return(i);
}

/*
 * Busca do UART uma string
 */
uint32_t usart_getString(uint8_t *pstring){
  uint32_t i = 0 ;
  
  usart_serial_getchar(USART0, (pstring+i));
  return(i);
  
}

  void USART0_Handler(void){
	  uint32_t ret = usart_get_status(USART0);
	  if(ret & US_IER_RXRDY){
		  //usart_getString(bufferRX);
		  //printf("%s", bufferRX);
		  } else if(ret & US_IER_TXRDY){
	  }
  }


/**
 * \brief Interrupt handler for the PWM controller.
 */
void PWM0_Handler(void)
{

	uint32_t events = pwm_channel_get_interrupt_status(PWM0);
	

	if ((events & (1 << PIN_PWM_LED0_CHANNEL)) == (1 << PIN_PWM_LED0_CHANNEL)) {
		if(flag){
			g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 50);
		}
		else{
			g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 100);
		}
	}
}


void Motor_state(uint32_t state){
	
	if(state){
		flag = 1;
	}
	else{
		flag = 0;
	}
	
};

void PWM0_Init(){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);


	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);
	pwm_channel_disable(PWM0, PIN_PWM_LED1_CHANNEL);


	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};

	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_LOW;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = 100;
	g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;

	pwm_channel_init(PWM0, &g_pwm_channel_led);

	/* Enable channel counter event interrupt */
	pwm_channel_enable_interrupt(PWM0, PIN_PWM_LED0_CHANNEL, 0);

	/* Configure interrupt and enable PWM interrupt */
	NVIC_DisableIRQ(PWM0_IRQn);
	NVIC_ClearPendingIRQ(PWM0_IRQn);
	NVIC_SetPriority(PWM0_IRQn, 0);
	NVIC_EnableIRQ(PWM0_IRQn);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, PIN_PWM_LED0_CHANNEL);

}

void hbridge_Init(){
	
	pmc_enable_periph_clk(PIN1_PIO_ID);
	pmc_enable_periph_clk(PIN2_PIO_ID);
	pmc_enable_periph_clk(PIN3_PIO_ID);
	pmc_enable_periph_clk(PIN4_PIO_ID);
	
	pio_set_output(PIN1_PIO, PIN1_PIN_MASK, 1, 0, 0);
	pio_set_output(PIN2_PIO, PIN2_PIN_MASK, 0, 0, 0);
	pio_set_output(PIN2_PIO, PIN3_PIN_MASK, 1, 0, 0);
	pio_set_output(PIN2_PIO, PIN4_PIN_MASK, 0, 0, 0);
}

void get_eeg_data(void)
{
	pio_set(LED_PIO, LED_PIN_MASK);
	usart_serial_getchar(USART0, &c);
	usart_serial_getchar(USART0, &d);
	if (c == 170 && d == 170){
		usart_serial_getchar(USART0, &c);
		if (c == 4) {
			usart_serial_getchar(USART0, &c);
			if (c == 128) {
				usart_serial_getchar(USART0, &c);
				if (c == 2) {
					usart_serial_getchar(USART0, &c);
					usart_serial_getchar(USART0, &d);
					int rawValue = ((int)c << 8) | d;
					Motor_state(1);
					pio_clear(LED_PIO, LED_PIN_MASK);
					printf("%d \n", rawValue);
				}
			}
		}
	}
}

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){


  /* Initialize the SAM system */
  sysclk_init();
  board_init();
  
  /* Inicializa com serial com PC*/
  configure_console();
 
  /* Configura USART0 para comunicacao com o HM-10 */
  USART0_init();

  LED_init(1);

  sprintf(bufferTX, "%s", "AT+CONA81B6AAB4B86");
[]  usart_putString(bufferTX);
[]
[] char c;
 char d;

  /* Inicializa funcao de delay */
  delay_init( sysclk_get_cpu_hz());

  /**/
  PWM0_Init();

  /**/
  hbridge_Init();
        
	while (1) {
		Motor_state(0);
		get_eeg_data();
	}
}
