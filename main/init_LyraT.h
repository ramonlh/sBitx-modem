/*
 * init_LyraT.h
 *
 *  Created on: 18 nov. 2022
 *      Author: Ramón
 */

#ifndef MAIN_INIT_LYRAT_H_
#define MAIN_INIT_LYRAT_H_

#include "es8388_registers.h"
#include <driver/i2s.h>
#include <driver/i2c.h>

// Basic I2S and I2C Configuration
#define I2S_NUM I2S_NUM_0

#define I2C_NUM I2C_NUM_0
#define ES8388_ADDR 0x20

#define I2S_PORT I2S_NUM_0

int player_volume=50;
audio_event_iface_handle_t evt;
audio_board_handle_t board_handle;
esp_periph_set_handle_t set;

void setMCLK()   // activar MCLK para PCM1808
	{
	ESP_LOGI(TAG, "===============setMCLK===============================");
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
	WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
	}

// ES8388 Configuration Code, audio codec over I2C for AUX IN input and headphone jack output
static esp_err_t es_write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
	esp_err_t res = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	res |= i2c_master_start(cmd);
	res |= i2c_master_write_byte(cmd, slave_add, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_write_byte(cmd, data, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_stop(cmd);
	res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return res;
}

static esp_err_t es8388_init()
{
	esp_err_t res = ESP_OK;

	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_NUM_18,
		.sda_pullup_en = true,
		.scl_io_num = GPIO_NUM_23,
		.scl_pullup_en = true,
		.master.clk_speed = 100000
	};

	res |= i2c_param_config(I2C_NUM, &i2c_config);
	res |= i2c_driver_install(I2C_NUM, i2c_config.mode, 0, 0, 0);

	/* mute DAC during setup, power up all systems, slave mode */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
	res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00);

	/* power up DAC and enable only LOUT1 / ROUT1, ADC sample rate = DAC sample rate */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x30);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);

	/* DAC I2S setup: 16 bit word length, I2S format; MCLK / Fs = 256*/
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);

	/* DAC to output route mixer configuration */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);

	/* DAC and ADC use same LRCK, enable MCLK input; output resistance setup */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);

	/* DAC volume control: 0dB (maximum, unattenuated)  */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00);

	/* power down ADC while configuring; volume: +9dB for both channels */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xff);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0x33);

	/* select LINPUT2 / RINPUT2 as ADC input; stereo; 16 bit word length, format right-justified, MCLK / Fs = 256 */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, 0x50);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0e);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);

	/* set ADC volume */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, 0x20);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, 0x20);

	/* set LOUT1 / ROUT1 volume: 0dB (unattenuated) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1e);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1e);

	/* power up and enable DAC; power up ADC (no MIC bias) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09);

	return res;
}

void init_I2S() {
	ESP_LOGI(TAG, "=============es8388_init=================================");
	if (es8388_init() != ESP_OK)
		ESP_LOGI(TAG, "[es8388_init] Audio codec initialization failed!");
	ESP_LOGI(TAG, "=============init_I2S=================================");
	i2s_config_t i2s_read_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
		.sample_rate = 96000,
		.bits_per_sample = 16,
		.communication_format = I2S_COMM_FORMAT_STAND_I2S,				// 0x01
		//		.communication_format = 0x02,
		//		.communication_format = I2S_COMM_FORMAT_STAND_MSB,			// 0x03
		//		.communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,	// 0x04 no funciona
		//		.communication_format = I2S_COMM_FORMAT_STAND_PCM_LONG,		// 0x0C	funciona, pero mal
		//		.communication_format = I2S_COMM_FORMAT_STAND_MAX,			//	funciona, pero mal
		//		.communication_format = I2S_COMM_FORMAT_I2S,				// 0x01 funciona, pero mal
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
		.dma_buf_count = 64,
		.dma_buf_len = 64,
		.use_apll = 1,
		.tx_desc_auto_clear = 1,
		.fixed_mclk = 0
	};

	i2s_pin_config_t i2s_read_pin_config = {
		.bck_io_num = GPIO_NUM_5,
		.ws_io_num = GPIO_NUM_25,
		.data_out_num = GPIO_NUM_26,
		.data_in_num = GPIO_NUM_35
	};
	i2s_driver_install(I2S_NUM, &i2s_read_config, 0, NULL);
	i2s_set_pin(I2S_NUM, &i2s_read_pin_config);

}

void set_filter()
{
  for (int i=0; i<MAX_BINS; i++)
  {
	  if ((i<minf) || (i>maxf))
		  fir_coeff[i]=0.0;
	  else
		  fir_coeff[i]=1.0;
  }
  ESP_LOGI(TAG, "[ FILTER ] %i - %i (%i+-%i)",minf, maxf,(minf)*48000/2048, (maxf)*48000/2048);
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    board_handle = (audio_board_handle_t) ctx;
    ESP_LOGI(TAG, "[ KEY ] input key event (%d)",(int)evt->data );
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);
    ESP_LOGI(TAG, "[ KEY ] Volume is %d %%", player_volume);
	if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) {
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_REC:				// 1
                ESP_LOGI(TAG, "[ KEY ] [Rec] input key event (%d)",(int)evt->data );
                minf = minf - stepfilter; if (minf < 0) minf = 0;
                maxf = minf + wbfilter;
                set_filter();
                break;
            case INPUT_KEY_USER_ID_MODE:			// 4
                ESP_LOGI(TAG, "[ KEY ] [Mode] input key event (%d)",(int)evt->data );
                minf = minf + stepfilter; if (minf > maxfilter) minf = maxfilter;
                maxf = minf + wbfilter;
                set_filter();
                break;
            case INPUT_KEY_USER_ID_PLAY:			// 3
                ESP_LOGI(TAG, "[ KEY ] [Play] input key event (%d)",(int)evt->data );
                minf = minf + 10;
                set_filter();
                break;
            case INPUT_KEY_USER_ID_SET:				// 2
                ESP_LOGI(TAG, "[ KEY ] [Set] input key event (%d)",(int)evt->data );
                maxf = maxf - 10;
                set_filter();
			    break;
            case INPUT_KEY_USER_ID_VOLDOWN:			// 5
                ESP_LOGI(TAG, "[ KEY ] [Vol-] input key event (%d)",(int)evt->data );
                player_volume -= 10;
                if (player_volume < 0) { player_volume = 0; }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ KEY ] Volume set to %d %%", player_volume);
                break;
            case INPUT_KEY_USER_ID_VOLUP:			// 6
                ESP_LOGI(TAG, "[ KEY ] [Vol+] input key event (%d)",(int)evt->data );
                player_volume += 10;
                if (player_volume > 100) { player_volume = 100; }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ KEY ] Volume set to %d %%", player_volume);
                break;
            }
        }
    return ESP_OK;
}


void setupCODEC()
{
	ESP_LOGI(TAG, "===============setupCODEC===============================");
    ESP_LOGI(TAG, "[ CODEC ] Start codec chip");
    board_handle = audio_board_init();

    ESP_LOGI(TAG, "[ CODEC ] Start audio hal");
	esp_err_t err = audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);
	if (err != ESP_OK)
		ESP_LOGI(TAG, "audio_hal_ctrl_codec failed!");
	else
		ESP_LOGI(TAG, "audio_hal_ctrl_codec OK");
	//esp_err_t ret = es8388_write_reg(ES8388_ADCCONTROL2, ADC_INPUT_LINPUT2_RINPUT2);
	//if (ret != ESP_OK) { ESP_LOGI(TAG, "[ ES8388 ] ES8388 writereg error : %d", ret); }
    audio_hal_set_volume(board_handle->audio_hal, player_volume);
    ESP_LOGI(TAG, "[ KEY ] Volume set to %d %%", player_volume);

}

void setupPERIPH()
{
	ESP_LOGI(TAG, "=============setupPERIPH=================================");
	ESP_LOGI(TAG, "[PERIPH] Initialize peripherals management");
	esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
	set = esp_periph_set_init(&periph_cfg);
	audio_board_key_init(set);
}

void setupKEYS()
{
	ESP_LOGI(TAG, "=============setupKEYS=================================");
    ESP_LOGI(TAG, "[ KEY ] Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, (void *)board_handle);
}


#endif /* MAIN_INIT_LYRAT_H_ */
