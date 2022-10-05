#include <atmel_start.h>
#include <stdio.h>
#include <atstart/atmel_start.h>
#include <atstart/driver_init.h>

/*
	resolution for samc21 adc
	12bit = 1
	10bit = 2
	16bit = 3
	
	voltage references for samc21
	SAMC21_ADC_REF_INTVCC0 = 1
	SAMC21_ADC_REF_INTVCC1 = 2
	SAMC21_ADC_REF_VREFA   = 3
	SAMC21_ADC_REF_DAC     = 4
	SAMC21_ADC_REF_INTVCC2 = 5
*/
#define ADC_RES 2
#define ADC_VREF 5

/* hard coding sensor positions sense 0 is right, sense 1 is left */
#define RIGHTSEN_NIL 746 
#define RIGHTSEN_MAX 294 
#define LEFTSEN_NIL 132 
#define LEFTSEN_MAX 604
#define SLACK 40
#define MAX_THR_DIFF 10 // out of 100

static struct io_descriptor *uart;

#define NUM(a) (sizeof(a)/sizeof(*a))

uint8_t analog_sensor_inputs[] = {
	5,  // PB09, Throttle 1
	4,  // PB08, Throttle 2
	//10, // PA08, Brake pressure
	//9,  // PB07, Brake position
	//8,  // PB06, Steering position
};

/*
static void CAN_0_tx_callback(struct can_async_descriptor *const descr) {
}
*/

static void CAN_0_rx_callback(struct can_async_descriptor *const descr) {
	static struct can_message msg;
	static uint8_t data[8];
	msg.data = data;
	can_async_read(descr, &msg);
}

static void setup_can(void) {
	//can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, CAN_0_rx_callback);
	//can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, CAN_0_tx_callback);
	can_async_enable(&CAN_0);

	/*
    struct can_filter filter;
    filter.id   = 0x555;
    filter.mask = 0x7FF;
    can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);
    */
}

static void sample_sensors(void) {
	static struct can_message msg;
	static int sensor_data[8];

	for (size_t i = 0; i < NUM(analog_sensor_inputs); i++) {
		adc_sync_set_inputs(&ADC_1, analog_sensor_inputs[i], 0x18, 0);
		
		uint16_t adc_val;
		adc_sync_read_channel(&ADC_1, 0, (uint8_t *)&adc_val, 2);

		sensor_data[i] = adc_val;
		char buf[32];
		int len = snprintf(buf, 32, "%d ", adc_val);
		//int len = snprintf(buf, 32, "%.3f\n", res*5/4096.0f);
		io_write(uart, buf, len);
	}
	io_write(uart, "\n", 1);
	//Processing sensor datas
	int sense_right = sensor_data[0];
	int sense_left = sensor_data[1];
	
	int thr_right = (RIGHTSEN_NIL - sense_right - SLACK)*100/(RIGHTSEN_NIL-RIGHTSEN_MAX-SLACK);
	int thr_left = (sense_left - LEFTSEN_NIL - SLACK)*100/(LEFTSEN_MAX-LEFTSEN_NIL-SLACK);

	uint8_t validSense;  // 1(true) or 0(false)
	if(abs(thr_left - thr_right) <= MAX_THR_DIFF){
		validSense = 1;
	}else{
		validSense = 0;
	}
	int thr_valu = (thr_left + thr_right)/2;
	int thr_valu2 = (0 < thr_valu)?thr_valu:0; // bring thr_val to >= 0
	int thr_valu3 = (thr_valu2 < 100)?thr_valu2:100;// bring thr_val to 0 - 100
	uint8_t thr_val = thr_valu3;
	//print onto serial
	
	char buf[32];
	int len = snprintf(buf, 32, "%u ", validSense);
	//int len = snprintf(buf, 32, "left %d ", thr_left);
	io_write(uart, buf, len);
	len = snprintf(buf, 32, "%u ", thr_val);
	//len = snprintf(buf, 32, "right %d ", thr_right);
	io_write(uart, buf, len);
	
	
	uint8_t packet_data[8] = {0,0,0,0,0,0,0,0};
	packet_data[0] = validSense;
	packet_data[1] = thr_val;
	
	msg.id = 0x100; // TODO
	msg.type = CAN_TYPE_DATA;
	msg.data = packet_data;
	msg.len = 8;//NUM(analog_sensor_inputs);
	msg.fmt = CAN_FMT_STDID;
	
	int32_t status = can_async_write(&CAN_0, &msg);
	//char buf[32];
	len = snprintf(buf, 32, "%d ", status);
	io_write(uart, buf, len);
	io_write(uart, "\n", 1);
}

int main(void) {
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	usart_sync_get_io_descriptor(&UART_EDBG, &uart);
	usart_sync_enable(&UART_EDBG);
	io_write(uart, "Hello World!\n", 13);

	adc_sync_enable_channel(&ADC_1, 0);
	adc_sync_set_reference(&ADC_1, ADC_VREF);
	adc_sync_set_resolution(&ADC_1, ADC_RES);

	while (1) {
		if (hri_tc_get_INTFLAG_OVF_bit(TC0)) {
			hri_tc_clear_INTFLAG_OVF_bit(TC0);

			sample_sensors();
		}
	}
}
