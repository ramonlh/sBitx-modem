#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

static const char *TAG = "sBitx-Modem-LyraT";

#define MODE_USB 0
#define MODE_LSB 1
#define MODE_CW 2
#define MODE_CWR 3

#define N_SAMPLES 2048
#define MAX_BINS N_SAMPLES			// 2048
#define I2S_BUFFLEN N_SAMPLES * 4	// 8192
#define R_TUNED_BIN MAX_BINS / 4	// 512
#define SSB_MODE MODE_USB

int printed=0;

#include <esp_log.h>
#include "dsp_common.h"
#include "dsps_fft2r.h"
#include "dsps_view.h"
#include "dsps_wind_hann.h"
#include "dsps_tone_gen.h"

#include "FFT.h"

#include "audio_error.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "periph_service.h"
#include "audio_hal.h"
#include "board.h"

#include "input_key_service.h"

int minf=0;
int maxf=128;
int stepfilter=64;
int wbfilter=128;
int maxfilter=MAX_BINS*2;


#include "init_LyraT.h"

int samples[N_SAMPLES];
uint16_t countcycles=0;
__attribute__((aligned(16)))
float fft_m[MAX_BINS];					// half table, complex
__attribute__((aligned(16)))
ftab_type fft_in;						// complex
__attribute__((aligned(16)))
ftab_type fft_out;						// complex

void printtab(int n, ftab_type tab, char* title)	{
	if (printed != n) return;
	printf(title);
	printf("  -----------------\r\n");
	for (int i=0; i< MAX_BINS; i++)   {
		printf("%f;%f\n",tab[2*i],tab[2*i+1]);
		}
	printf("====================================\r\n");
}

void printsamples(int n)	{
	if (printed != n) return;
	printf("samples");
	printf("  -----------------\r\n");
	for (int i=0; i<N_SAMPLES/2; i++)   {
		printf("%i;%i\n",samples[2*i],samples[2*i+1]);
		}
	printf("====================================\r\n");
}

static void timer1(void *pvParameters)
{
	int logtime=1;
	while(1) {
//		ESP_LOGI(TAG, "Cycles:%i,  size(samples),:%i ",countcycles,sizeof(samples));
		countcycles=0;
		vTaskDelay(logtime*1000 / portTICK_PERIOD_MS);
	}
}

static void audio_test_task(void *pvParameters)
{
	ESP_LOGI(TAG,"N_SAMPLES:%i,  MAX_BINS:%i,   I2S_BUFFLEN:%i,   R_TUNED_BIN:%i", N_SAMPLES, MAX_BINS, I2S_BUFFLEN, R_TUNED_BIN);
	ESP_LOGI(TAG,"sizes  samples:%i,  fft_m:%i,   fft_in:%i:", sizeof(samples), sizeof(fft_m), sizeof(fft_in));
	size_t bytes_read = 0;
	size_t bytes_writen = 0;
    while (1) {
		i2s_read(I2S_NUM_0, samples, I2S_BUFFLEN, &bytes_read, portMAX_DELAY);
		i2s_write(I2S_NUM_0, samples, bytes_read, &bytes_writen, portMAX_DELAY);
    	printed++;
		countcycles++;
    }
    vTaskDelete(NULL);
}

void myfilter(float *data, int N, int fmin, int fmax)
{
	int xmin, xmax;
	xmin = fmin / 23.4375;
	xmax = fmax / 23.4375;
	//printf("xmin:%i,  xmax:%i\n", xmin, xmax);
	for (int i = 0; i < MAX_BINS/2; i++)	{
		if ((i<xmin) || (i>xmax))	{
			data[2*i] = 0;
			data[2*i+1] = 0;
		}
	}
}


static void audio_modem_task(void *pvParameters)
{
	FFT_init(MAX_BINS);

    float i_sample, q_sample;

	size_t bytes_read = 0;
    while (1) {
    // Step 0, read samples
		i2s_read(I2S_NUM_0, samples, I2S_BUFFLEN, &bytes_read, portMAX_DELAY);
	// Step 1, copy 1/2 fft_m to fft_in
		for (int i = 0; i < MAX_BINS/2; i++)	{
		    fft_in[2*i] = fft_m[2*i];
		    fft_in[2*i+1] = fft_m[2*i+1];
		}
    // Step 2, // add the new set of samples to 2/2 fft_m & 1/2 fft_in
    	int j = 0;
    	for (int i = MAX_BINS/2; i < MAX_BINS; i++)    	{
    		i_sample = (float)samples[2*j];    // canal L
    		q_sample = 0;
    		fft_m[2*j] = i_sample;
    		fft_m[2*j+1] = q_sample;

    		fft_in[2*i] = i_sample;
    		fft_in[2*i+1] = q_sample;
    		j++;
    	}
    	for (int i=0;i<MAX_BINS*2;i++)
    		fft_out[i]=fft_in[i];
    // Step 3, convert the time domain samples to  frequency domain
		FFT(fft_out, MAX_BINS);		// FFT forward

	    if (SSB_MODE == MODE_LSB || SSB_MODE == MODE_CWR)	{
	    	for (int i = 0; i < MAX_BINS/2; i++)
	        	{
	        	}
	    }
	    else	{		// USB
	    	for (int i = 0; i < MAX_BINS/2; i++)
	        	{
	    		fft_out[2*i] = fft_out[2*i+MAX_BINS];
	    		fft_out[2*i+1] = fft_out[2*i+MAX_BINS+1];
	    		fft_out[2*i+MAX_BINS] = 0;
	    		fft_out[2*i+MAX_BINS+1] = 0;
	        	}
	    }
		for (int i = 0; i < MAX_BINS; i++)	{
			fft_out[2*i] = fft_out[2*i] * fir_coeff[i];
			fft_out[2*i+1] = fft_out[2*i+1] * fir_coeff[i];
			}
		// Step 7,  FFT Reverse, convert to domain time
		rFFT(fft_out,MAX_BINS,3);

		// Step 7.4, 	scale
		float gain=200;
		for (int i=0;i<MAX_BINS;i++)	    {	// scale
			fft_out[2*i]=fft_out[2*i]*gain;
			fft_out[2*i+1]=fft_out[2*i]*gain;
			fft_out[2*i]=0;
			//fft_out[2*i+1]=0;
			}
		// Step 8,	CAG

	    // Step 9,  copy fft_out to samples
	    for (int i=0; i<MAX_BINS/2; i++)	    {
	    	//samples[2*i] = fft_out[2*i + (MAX_BINS/2) + 1]; // Rigth channel
	        samples[2*i] = fft_out[2*i];		// Rigth channel
	        samples[2*i+1] = fft_out[2*i+1];    // Left channel
	    }
		i2s_write(I2S_NUM_0, samples, I2S_BUFFLEN, &bytes_read, portMAX_DELAY);
    	printed++;
		countcycles++;
		vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void setupLOG()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
}

void init_data()
{
	for (int i=0; i<MAX_BINS/2; i++) {
		samples[2*i]=0; samples[2*i+1]=0;
		fft_m[i]=0;
		fft_in[2*i]=0; fft_in[2*i+1]=0;
		fft_out[2*i]=0; fft_out[2*i+1]=0;
	}
    set_filter();
}

void init_DSP()
{
    dsps_fft2r_init_fc32(NULL, N_SAMPLES);
}

#define RX_BUFFER_LEN 255
#define TX_BUFFER_LEN 255
#define I2C_NUM I2C_NUM_1

typedef struct {    // datos I2C lyraT
      uint8_t comtype;
      uint16_t min_f;    // lim inferior filtro
      uint16_t max_f;    // lim superior filtro
      uint16_t gain;     // gain
} datalyratype;
      datalyratype datalyra;
      uint8_t *bufflyra = (uint8_t *) &datalyra; // acceder a datalyratype como bytes

static void i2c_handle_task(void *pvParameters)
{
    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_SLAVE;
    i2c_config.sda_io_num = 14;			// OK, probado
    i2c_config.scl_io_num = 15;			// OK, probado
    i2c_config.sda_pullup_en = true;
    i2c_config.scl_pullup_en = true;
    i2c_config.slave.addr_10bit_en=0;
    i2c_config.slave.slave_addr=0x32;
    i2c_config.clk_flags = 0;

    i2c_param_config(I2C_NUM, &i2c_config);
    i2c_driver_install(I2C_NUM, I2C_MODE_SLAVE, RX_BUFFER_LEN, TX_BUFFER_LEN, ESP_INTR_FLAG_LEVEL1);
	ESP_LOGI(TAG, "i2c_handle_task start");
    uint8_t rx_data[RX_BUFFER_LEN] = {0};
    uint8_t tx_data[32] = "Hello from Slave";
    while (1)
    {
        int len = i2c_slave_read_buffer(I2C_NUM, bufflyra,RX_BUFFER_LEN,pdMS_TO_TICKS(100));
        if (len > 0)
        {
    		ESP_LOGI(TAG," comtype: %i  min: %i  max: %i   gain: %i", datalyra.comtype, datalyra.min_f, datalyra.max_f, datalyra.gain);
    		if (datalyra.comtype == 1)		// set filter
    		{

    		}
    		else if (datalyra.comtype == 2)	// set Gain
    		{

    		}
    		else if (datalyra.comtype == 3)	// send spectrum
    		{
    		}
        	i2c_reset_rx_fifo(I2C_NUM);
        	i2c_slave_write_buffer(I2C_NUM,tx_data,sizeof(tx_data),pdMS_TO_TICKS(1));
        	memset(rx_data,0,RX_BUFFER_LEN);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
	ESP_LOGI(TAG, "Inicio");
	// No modificar el orden de llamada
	setupLOG();			// 1
	setupCODEC();		// 2
	init_I2S();			// 3
    setMCLK();			// 4
	setupPERIPH();		// 5
	setupKEYS();		// 6
    init_data();		// 7
    init_DSP();			// 8

    //xTaskCreate(audio_test_task, "audio_test_task", 10000, NULL, 5, NULL);		// test mode passthrough  input-->output
    xTaskCreate(i2c_handle_task,"i2c_handle_task",10000,NULL,5,NULL);
    xTaskCreate(audio_modem_task, "audio_modem_task", 10000, NULL, 5, NULL);
    xTaskCreate(timer1, "timer1", 2048, NULL, 5, NULL);
}
