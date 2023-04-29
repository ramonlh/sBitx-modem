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
#define power2dB(x) (10*log10f(x))

#include <esp_log.h>
#include "math.h"
#include "dsp_common.h"
#include "dsps_fft2r.h"
//#include "dsps_view.h"
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
__attribute__((aligned(16)))
ftab_type fft_out2;						// complex
__attribute__((aligned(16)))
ftab_type fft_spectrum;						// complex

#define TX_BUFFER_LEN 255
#define I2C_NUM I2C_NUM_1
uint8_t data_spectrum[TX_BUFFER_LEN];
uint8_t data_spectrum_aver[TX_BUFFER_LEN];
uint8_t ds[MAX_BINS];

float spectrum_speed = 0.1;

typedef struct {    // datos I2C lyraT
      uint8_t comtype;
      uint16_t min_f;    // lim inferior filtro
      uint16_t max_f;    // lim superior filtro
      uint16_t gain;     // gain
      uint16_t spspan;   // spectrum span
      uint16_t spatt;    // spectrum att
      uint16_t volume;   // volume
      uint16_t ssbmode;  // lsb/usb
      uint16_t cwmode;   // cwmode
} datalyratype;
      datalyratype datalyra;
      uint8_t *bufflyra = (uint8_t *) &datalyra; // acceder a datalyratype como bytes


void printtab(int n, ftab_type tab, char* title)	{
	printf(title);
	printf("  -----------------\r\n");
	for (int i=0; i< MAX_BINS; i++)   {
		printf("%f;%f\n",tab[2*i],tab[2*i+1]);
		}
	printf("====================================\r\n");
}

void printsamples(int n)	{
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
		//countcycles=0;
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
		//countcycles++;
    }
    vTaskDelete(NULL);
}

static void audio_modem_task(void *pvParameters)
{
	FFT_init(MAX_BINS);
    float i_sample, q_sample;
	size_t bytes_read = 0;
	memset(data_spectrum_aver,0,sizeof(data_spectrum_aver));
    while (1) {
    // Step 0, read samples
		// int type range min=-2147483648, max=2147483647
		i2s_read(I2S_NUM_0, samples, I2S_BUFFLEN, &bytes_read, portMAX_DELAY);
	// Step 1, copy 1/2 fft_m to fft_in
		for (int i = 0; i < MAX_BINS/2; i++)	{
		    fft_in[2*i] = fft_m[2*i];			// real
		    fft_in[2*i+1] = fft_m[2*i+1];		// imag
		}
    // Step 2, // add the new set of samples to 2/2 fft_m & 1/2 fft_in
    	int j = 0;
    	for (int i = MAX_BINS/2; i < MAX_BINS; i++)    	{
    		i_sample = (float)samples[2*j+1];    // canal R
    		q_sample = 0;
    		fft_m[2*j] = i_sample;		// real
    		fft_m[2*j+1] = q_sample;	// imag

    		fft_in[2*i] = i_sample;		// real
    		fft_in[2*i+1] = q_sample;	// imag
    		j++;
    	}
    	for (int i=0;i<MAX_BINS*2;i++)		// pendiente de cambiar, sobra este paso ?
    	{
    		fft_out[i]=fft_in[i];
    		fft_out2[i]=fft_in[i];
    	}

    // Step 3, convert the time domain samples to  frequency domain
    	FFT(fft_out, MAX_BINS);		// FFT forward for audio

  	//////////////////////  Spectrum ////////////////////////
    // Step 3-B, convert the time domain samples to  frequency domain
   			// MULTIPLICAR POR LA VENTANA HANN ANTES DE HACER FFT SOLO PARA EL ESPECTRO
    	for (int i=0; i<MAX_BINS;i++)
    	{
    		fft_out2[2*i] = fft_out2[2*i]  * hann_window[i];
    	}
    	FFT(fft_out2, MAX_BINS);	// FFT forward for spectrum
    	int step_bin = 4;
    	int aver_bins = 1;
		int bin_ini = 1532;
		if (spectrumspan == 12)
		{
	    	step_bin = 4;
	    	aver_bins = 1;
			bin_ini= 1532;
		}
		if (spectrumspan == 24)
		{
	    	step_bin = 8;
	    	aver_bins = 1;
			bin_ini= 1024;
		}
       	for (int i = 0; i < TX_BUFFER_LEN; i++)
       	{
      		int index = (step_bin * i) + bin_ini;
      		float auxfftr = 0;
       		float auxffti = 0;
       		for (int i = 0; i < aver_bins; i++)
       		{
       			auxfftr = auxfftr + fft_out2[index+i];
       			auxffti = auxffti + fft_out2[index+i+1];
       		}
       		auxfftr = auxfftr / aver_bins;
       		auxffti = auxffti / aver_bins;
       		float y=power2dB(sqrt((auxfftr * auxfftr) + (auxffti * auxffti)) / spectrumatt);
			data_spectrum[i] =  ((1.0-spectrum_speed) * data_spectrum_aver[i]) + (spectrum_speed * (int)y);
			data_spectrum_aver[i] = data_spectrum[i];
			//printf("%i;%i\n",i,data_spectrum[i]);
       	}


///////////////////////////////////////////////////////////////////////////////

       	if (SSB_MODE == MODE_LSB || SSB_MODE == MODE_CWR)
       	{
	    }
	    else
	    {		// USB
	    	for (int i = 0; i < MAX_BINS/2; i++)
	        	{
	    		fft_out[2*i] = fft_out[2*i+MAX_BINS];
	    		fft_out[2*i+1] = fft_out[2*i+MAX_BINS+1];
	    		fft_out[2*i+MAX_BINS] = 0;
	    		fft_out[2*i+MAX_BINS+1] = 0;
	        	}
	    }
		for (int i = 0; i < MAX_BINS; i++)	{				// filter
			fft_out[2*i] = fft_out[2*i] * fir_coeff[i];
			fft_out[2*i+1] = fft_out[2*i+1] * fir_coeff[i];
			}

		// Step 7,  FFT Reverse, convert to domain time
		rFFT(fft_out,MAX_BINS,3);

		// Step 7.4, 	scale
		for (int i=0;i<MAX_BINS;i++)	    {	// scale/gain
			fft_out[2*i]=0;    									// Left channel		// 5
			fft_out[2*i+1]=fft_out[2*i+1]*datalyra.gain * 5;    // Rigth channel		// 5
			}
		// Step 8,	AGC

	    // Step 9,  copy fft_out to samples
	    for (int i=0; i<MAX_BINS/2; i++)	    {
	        samples[2*i] = 0;					// Left channel
	        samples[2*i+1] = fft_out[2*i+1];    // Rigth channel
	    }
		i2s_write(I2S_NUM_0, samples, I2S_BUFFLEN, &bytes_read, portMAX_DELAY);
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
    set_filter(datalyra.min_f,datalyra.max_f);
}

void init_DSP()
{
    dsps_fft2r_init_fc32(NULL, N_SAMPLES);
}

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
    i2c_driver_install(I2C_NUM, I2C_MODE_SLAVE, sizeof(datalyra), TX_BUFFER_LEN, ESP_INTR_FLAG_LEVEL1);
	ESP_LOGI(TAG, "i2c_handle_task start");
    while (1)
    {
    	i2c_reset_rx_fifo(I2C_NUM);
        int len = i2c_slave_read_buffer(I2C_NUM, bufflyra,sizeof(datalyra),pdMS_TO_TICKS(100));
        if (len > 0)
        {
    		//ESP_LOGI(TAG," comtype: %i  min: %i  max: %i  gain: %i  vol: %i", datalyra.comtype, datalyra.min_f, datalyra.max_f, datalyra.gain, datalyra.volume);
    		if (datalyra.comtype == 1)		// set filter
    		{
    			minf = datalyra.min_f;
    			maxf = datalyra.max_f;
                set_filter(datalyra.min_f,datalyra.max_f);
                ESP_LOGI(TAG, "recibido filter %i - %i)", datalyra.min_f, datalyra.max_f);
    		}
    		else if (datalyra.comtype == 2)	// set Gain
    		{
    			//set_gain(datalyra.gain);  //
    			ESP_LOGI(TAG, "recibido gain %i",datalyra.gain);
    		}
    		else if (datalyra.comtype == 3)	// send spectrum
    		{
            	int sentbytes = i2c_slave_write_buffer(I2C_NUM, data_spectrum, TX_BUFFER_LEN, pdMS_TO_TICKS(100));
        		//ESP_LOGI(TAG," ENVIADA RESPUESTA: %i bytes", sentbytes);
    		}
    		else if (datalyra.comtype == 4)	// set volume
    		{
    			set_volume(datalyra.volume);
    			ESP_LOGI(TAG, "recibido volume %i",datalyra.volume);
    		}
    		else if (datalyra.comtype == 5)	// set spectrum att
    		{
    			set_spectrumatt(datalyra.spatt);
    			ESP_LOGI(TAG, "recibido sp att %i",datalyra.spatt);
    		}
    		else if (datalyra.comtype == 6)	// set spectrum span
    		{
    			spectrumspan = datalyra.spspan;
    			ESP_LOGI(TAG, "recibido sp span %i",spectrumspan);
    		}
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
    xTaskCreate(audio_modem_task, "audio_modem_task", 15000, NULL, 5, NULL);
    xTaskCreate(timer1, "timer1", 2048, NULL, 5, NULL);
}
