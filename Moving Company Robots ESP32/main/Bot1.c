//This is Drive Module for MOving Company Robots
// This code is specific to bot 1
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/rtc.h"
#include "time.h"
#include "sys/time.h"



#define SPP_TAG "Moving Company Robots"

#define ESP_INTR_FLAG_DEFAULT 0

//A (left) side,
#define enA 17
#define IN1 15 	//forward
#define IN2 2	//backwords
#define encA 19
float 	pwmA = 70.0;
int16_t enccntA = 0;
int16_t enccntA_first = 0;
int16_t pppA = 0 ; //pulses per period

//B (right) side
#define enB 22

#define IN3 4	//backwords
#define IN4 16	//forward
#define encB 19
float 	pwmB = 70.0;
int16_t enccntB = 0;
int16_t enccntB_first = 0;
int16_t pppB = 0; //pulse per period....

int speedreached = 0;
int encmaxcnt = 100000;


int connected = 0;
uint8_t read ;
uint8_t Okflag = 99 ;
uint32_t sendhandle;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;



void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	static struct timeval time_old;
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name("SWARM BOT 1");
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, "MCR SERVER");
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                 param->data_ind.len, param->data_ind.handle);
        esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);

        sendhandle = param->data_ind.handle;

    	read = param->data_ind.data[0];

        switch(read){
        case 01:// bluetooth connection verification
        	connected = 1;
        	break;

        case 115:// Speed calibration
        	speedreached =0;
        	int frequencyofcorectspeed = 0;
        	pcnt_counter_clear(PCNT_UNIT_0);//encA
        	pcnt_counter_clear(PCNT_UNIT_1);//encB

        	gpio_set_level(IN1, 1);
        	gpio_set_level(IN2, 0);
        	gpio_set_level(IN3, 0);
        	gpio_set_level(IN4, 1);

        	vTaskDelay(10000 / portTICK_PERIOD_MS);

			ets_printf("Begining calibration.......");

			while(!speedreached){
				pcnt_get_counter_value(PCNT_UNIT_0, &enccntA_first);
				pcnt_get_counter_value(PCNT_UNIT_1, &enccntB_first);

				vTaskDelay(50 / portTICK_PERIOD_MS);

				pcnt_get_counter_value(PCNT_UNIT_0, &enccntA);
				pcnt_get_counter_value(PCNT_UNIT_1, &enccntB);

				pppA = enccntA - enccntA_first;
				pppB = enccntB - enccntB_first;

				if(pppA > 9){
					pwmA = pwmA - 1;
					mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_A,pwmA);
					vTaskDelay(50 / portTICK_PERIOD_MS);
				}

				if(pppB > 9){
					pwmB = pwmB - 1;
					mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_B,pwmB);
					vTaskDelay(50 / portTICK_PERIOD_MS);
				}

				if(pppA < 6){
					pwmA = pwmA + 1;
					mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_A,pwmA);
					vTaskDelay(50 / portTICK_PERIOD_MS);
				}

				if(pppB < 6){
					pwmB = pwmB + 1;
					mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_B,pwmB);
					vTaskDelay(50 / portTICK_PERIOD_MS);
				}

				if((pppA < 9)&&(pppA > 6)&&(pppB < 9)&&(pppB > 6)){
					frequencyofcorectspeed++;
					if(frequencyofcorectspeed >30){
						speedreached = 1;
						gpio_set_level(GPIO_NUM_21, 1);
						encmaxcnt = 0;
					}

				}
				if((pwmA > 105.0) | (pwmB >105.0)){
				    int level = 0;
				    while (!connected) {
				        gpio_set_level(GPIO_NUM_21, level);
				        level = !level;
				        vTaskDelay(50 / portTICK_PERIOD_MS);
						esp_spp_write(sendhandle, 1, &Okflag);

				    }
				}

				ets_printf("encA: %d	encB:%d  pwmA:%d	pwmB:%d\n", pppA,pppB,(int)pwmA,(int)pwmB);//8-7 is what speed i want!
			}
        	break;


//Encoder Specified cases

        case 102://decimal value of ascii 'f' go forward some encoders value tiems 100
        		//forward
        	pcnt_counter_clear(PCNT_UNIT_0);//encA
        	pcnt_counter_clear(PCNT_UNIT_1);//encB

        	gpio_set_level(IN1, 1);
        	gpio_set_level(IN2, 0);
        	gpio_set_level(IN3, 0);
        	gpio_set_level(IN4, 1);

        	encmaxcnt = (param->data_ind.data[1])*10;
        	break;

        case 114://decimal value of ascii 'r' go forward some encoders
        		//rotate Right
        	pcnt_counter_clear(PCNT_UNIT_0);//encA
        	pcnt_counter_clear(PCNT_UNIT_1);//encB

            gpio_set_level(IN1, 1);
            gpio_set_level(IN2, 0);
            gpio_set_level(IN3, 1);
            gpio_set_level(IN4, 0);

        	encmaxcnt = (param->data_ind.data[1]);
        	break;
        case 108://decimal value of ascii 'l' go forward some encoders
        		//rotate Left
        	pcnt_counter_clear(PCNT_UNIT_0);//encA
        	pcnt_counter_clear(PCNT_UNIT_1);//encB

            gpio_set_level(IN1, 0);
            gpio_set_level(IN2, 1);
            gpio_set_level(IN3, 0);
            gpio_set_level(IN4, 1);

        	encmaxcnt = (param->data_ind.data[1]);
        	break;


 //WASD MOVEMENT CASES
        case 00:// Robot Stop
            gpio_set_level(IN1, 0);
            gpio_set_level(IN2, 0);
            gpio_set_level(IN3, 0);
            gpio_set_level(IN4, 0);
        	break;
        case 02://forward
            gpio_set_level(IN1, 1);
            gpio_set_level(IN2, 0);
            gpio_set_level(IN3, 0);
            gpio_set_level(IN4, 1);
            break;
        case 03://backwards
            gpio_set_level(IN1, 0);
            gpio_set_level(IN2, 1);
            gpio_set_level(IN3, 1);
            gpio_set_level(IN4, 0);
            break;
        case 04://Rotate right
            gpio_set_level(IN1, 1);
            gpio_set_level(IN2, 0);
            gpio_set_level(IN3, 1);
            gpio_set_level(IN4, 0);
            break;
        case 05://Rotate left
            gpio_set_level(IN1, 0);
            gpio_set_level(IN2, 1);
            gpio_set_level(IN3, 0);
            gpio_set_level(IN4, 1);
            break;

        }
        break;

    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        //ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d", param->write.len , param->write.cong);
        esp_log_buffer_hex("",&Okflag,1);
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        gettimeofday(&time_old, NULL);
        break;
    default:
        break;
    }
}

static void EncodersTask(){
	for(;;){

		/*
		Task Counts to max encoder value while keeping
		the bot in a straightforward path
		 */


		vTaskDelay(5);

		pcnt_get_counter_value(PCNT_UNIT_0, &enccntA);
		pcnt_get_counter_value(PCNT_UNIT_1, &enccntB);


		//max encoder count reached?
		if((enccntA > encmaxcnt)&&(enccntB > encmaxcnt)){
			esp_spp_write(sendhandle, 1, &Okflag);
		}
		if(enccntA > encmaxcnt){
			gpio_set_level(IN1, 0);
			gpio_set_level(IN2, 0);
		}
		if(enccntB > encmaxcnt){
			gpio_set_level(IN4, 0);
			gpio_set_level(IN3, 0);
		}

		//Straight line adjustments w/ PWM - 10
		if(speedreached){
			if((enccntB-enccntA) >= 5){
				mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_B,pwmB-.75*pwmB);
			}
			else if((enccntA-enccntB) >= 5){
				mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_A,pwmA-.75*pwmA);

			}
			else{
				mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_A,pwmA);
				mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_B,pwmB);
			}
		}

	}
}


void bt_init(void){

		esp_err_t ret = nvs_flash_init();
	    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	        ESP_ERROR_CHECK(nvs_flash_erase());
	        ret = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK( ret );

	    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT(); // @suppress("Symbol is not resolved")
	    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
	        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
	        return;
	    }

	    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
	        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
	        return;
	    }

	    if ((ret = esp_bluedroid_init()) != ESP_OK) {
	        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
	        return;
	    }

	    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
	        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
	        return;
	    }

	    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
	        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
	        return;
	    }

	    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
	        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
	        return;
	    }

	    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
	        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
	        return;
	    }

	    /* Set default parameters for Secure Simple Pairing */
	    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
	    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
	    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

	    /*
	     * Set default parameters for Legacy Pairing
	     * Use variable pin, input pin code when pairing
	     */
	    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
	    esp_bt_pin_code_t pin_code;
	    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}

void pins_init(void){

    //----------------------------Gpio initilization---------------------------------------

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO 0,2,4,5
    io_conf.pin_bit_mask = 1ULL<<GPIO_NUM_21 | 1ULL<<GPIO_NUM_23 | 1ULL<<IN1 | 1ULL<<IN2 | 1ULL<<IN3 | 1ULL<<IN4;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //----------------------------Pulse counter initilization---------------------------------------


    /* Prepare configuration for the PCNT unit */

    pcnt_config_t pcnt_configA = {
            // Set PCNT input signal and control GPIOs
            .pulse_gpio_num = encA,
            .channel = PCNT_CHANNEL_0,
            .unit = PCNT_UNIT_0,
            // What to do on the positive / negative edge of pulse input?
            .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
            .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        };
        /* Initialize PCNT unit */

    pcnt_unit_config(&pcnt_configA);

        /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_configB = {
            // Set PCNT input signal and control GPIOs
            .pulse_gpio_num = encB,
            .channel = PCNT_CHANNEL_0,
            .unit = PCNT_UNIT_1,
            // What to do on the positive / negative edge of pulse input?
            .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
            .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        };
        /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_configB);

        /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_set_filter_value(PCNT_UNIT_1, 100);
    pcnt_filter_enable(PCNT_UNIT_1);

        /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);

        /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);


        //----------------------------MCPWM initilization---------------------------------------

    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = enA,
        .mcpwm0b_out_num = enB,
    };

    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 10000;    //frequency = 10KHz
    pwm_config.cmpr_a = pwmA;       //duty cycle of PWMxA = 70.0%
    pwm_config.cmpr_b = pwmB;       //duty cycle of PWMxb = 70.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings



}


void app_main()
{
	bt_init();
	pins_init();

	xTaskCreatePinnedToCore(EncodersTask,"EncodersTask", 2048, NULL, 10, NULL, 1);

    int level = 0;
    while (!connected) {
        gpio_set_level(GPIO_NUM_23, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    gpio_set_level(GPIO_NUM_23, 1);

}






