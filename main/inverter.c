/* MCPWM 3PHASE INVERTER EXAMPLE

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "sine_table.h"

//USER INPUTS: MODULATION INDEX AND DEADTIME
#define CUSTOM_DEADTIME 5 // In TIMER_TICKS, 100 ns per tick
#define DEFAULT_MF 30     // In *of LINE_FREQ   

#define EXAMPLE_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 0.1us per TIMER_TICK
#define LINE_FREQ 60                        // In Hz
#define NUMBER_OF_POINTS 111
#define DEFAULT_FS (LINE_FREQ * DEFAULT_MF) // In Hz

#define UPDATE_LOOP_PERIOD_US 150 // 1/(111*60) us, 111 points and 60hz base frequency

//#define UPDATE_LOOP_PERIOD_S (1 / (10 * LINE_FREQ * NUMBER_OF_POINTS))
//#define UPDATE_LOOP_PERIOD_US ((int)EXAMPLE_TIMER_RESOLUTION_HZ * UPDATE_LOOP_PERIOD_S) => 
//Explanation on my notebook:
//This should result in 150us but something is wrong, so the update loop period is fix for 111 points.
//The period should be 1/LINE_FREQ*NUMBER_OF_POINTS, because in 1/LINE_FREQ seconds, i should display all of my NUMBER_OF_POINTS values of CMPA
//So, the period in US should be 10000000*1/(10*LINE_FREQ*NUMER_OF_POINTS) (10TICKS=1US)

#define TIMER_PERIOD ((int)EXAMPLE_TIMER_RESOLUTION_HZ / (DEFAULT_FS)) 
// 1000 ticks = 0.1ms
// the period should relate to default FS,

#define K_MOD (TIMER_PERIOD / 400)
//Peak of carrier = TIMER_PERIOD/2, Peak of Modulated sinusoid = 200, with this gain we gek a sinusoid going from 0 to TIMER_PERIOD/2

#define PWM_UH_GPIO 21
#define PWM_UL_GPIO 19

#define PWM_VH_GPIO 18
#define PWM_VL_GPIO 5

#define PWM_WH_GPIO 17
#define PWM_WL_GPIO 16

//Pin connected to ESP32 datalogger, 1 means Tjunction overtemperature
#define TEMP_FAULT_GPIO 23

#define MCPWM_OP_INDEX_U 0
#define MCPWM_OP_INDEX_V 1
#define MCPWM_OP_INDEX_W 2

#define MCPWM_GEN_INDEX_HIGH 0
#define MCPWM_GEN_INDEX_LOW 1

static const char *TAG = "Inverter";

static int TIMER_CMPA[3] = {0, 0, 0};
static int update_duty_flag = 0;
static int index = 0;

static void mcpwm_update_TIMER_CMPA(void *args)
{
    // Once each UPDATE_LOOP_PRD_US we increase the index of a modulated sine-wave. Because we have 111 points, for a 60hz sinewave, the UPDATE_PRD should be
    // 1/111*60, wich is the calculated by the initial defintion.
    index = (index == 111) ? 0 : (index + 1);
    
    //TIMER_CMPA[0] = 10 * sine_tableA[index]; // Multiplied by 10 because of the timer resolution. The modulated wave is compited for a 1us res.PRD
    //TIMER_CMPA[1] = 10 * sine_tableB[index]; // so, the TIMER_PRD max is 555, an thus, the max of sinusoidal wave is TIMER_PRD/2. As we increase the
    //TIMER_CMPA[2] = 10 * sine_tableC[index]; // resolution, each tick counts 0.1us instead of 1us, so, TIMER_PRD become 5550, and if we keep same peak value for
                                                //  the modulated sinusoid the result isn't right.

    TIMER_CMPA[0] = (int)(K_MOD * sine_tableA[index]);
    TIMER_CMPA[1] = (int)(K_MOD * sine_tableB[index]);
    TIMER_CMPA[2] = (int)(K_MOD * sine_tableC[index]);

    update_duty_flag = 1;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Create MCPWM timer: Period: %i us, Resolution: %i Hz", TIMER_PERIOD / 10, EXAMPLE_TIMER_RESOLUTION_HZ);
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks = TIMER_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    ESP_LOGI(TAG, "Create MCPWM operator");
    mcpwm_oper_handle_t operators[3];
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
    }

    ESP_LOGI(TAG, "Connect operators to the same timer");
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    }

    ESP_LOGI(TAG, "Create comparators");
    mcpwm_cmpr_handle_t comparators[3];
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true,
        .flags.update_cmp_on_tep = true,
    };
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 100));
    }

    ESP_LOGI(TAG, "Create over current fault detector");
    mcpwm_fault_handle_t over_cur_fault = NULL;
    mcpwm_gpio_fault_config_t gpio_fault_config = {
        .gpio_num = TEMP_FAULT_GPIO,
        .group_id = 0,
        .flags.active_level = 1, // low level means fault, refer to DRV8302 datasheet
        .flags.pull_up = false,   // internally pull up
    };
    ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gpio_fault_config, &over_cur_fault));

    ESP_LOGI(TAG, "Set brake mode on the fault event");
    mcpwm_brake_config_t brake_config = {
        .brake_mode = MCPWM_OPER_BRAKE_MODE_CBC,
        .fault = over_cur_fault,
        .flags.cbc_recover_on_tez = true,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(operators[i], &brake_config));
    }

    ESP_LOGI(TAG, "Create PWM generators");
    mcpwm_gen_handle_t generators[3][2] = {};
    mcpwm_generator_config_t gen_config = {};
    const int gen_gpios[3][2] = {
        {PWM_UH_GPIO, PWM_UL_GPIO},
        {PWM_VH_GPIO, PWM_VL_GPIO},
        {PWM_WH_GPIO, PWM_WL_GPIO},
    };

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            gen_config.gen_gpio_num = gen_gpios[i][j];
            ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i][j]));
        }
    }

    ESP_LOGI(TAG, "Set generator actions");
    for (int i = 0; i < 3; i++)
    {

        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[i][MCPWM_GEN_INDEX_HIGH],
                                                                     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_HIGH),
                                                                     MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[i][MCPWM_GEN_INDEX_HIGH],
                                                                     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators[i], MCPWM_GEN_ACTION_LOW),
                                                                     MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_brake_event(generators[i][MCPWM_GEN_INDEX_HIGH],
                                                                     MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_CBC, MCPWM_GEN_ACTION_LOW),
                                                                     MCPWM_GEN_BRAKE_EVENT_ACTION_END()));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_brake_event(generators[i][MCPWM_GEN_INDEX_HIGH],
                                                                     MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_CBC, MCPWM_GEN_ACTION_LOW),
                                                                     MCPWM_GEN_BRAKE_EVENT_ACTION_END()));
    
    }

    ESP_LOGI(TAG, "Setup deadtime: %i ns", CUSTOM_DEADTIME * 100);
    mcpwm_dead_time_config_t dt_config = {
        .posedge_delay_ticks = CUSTOM_DEADTIME,
    };
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][MCPWM_GEN_INDEX_HIGH], generators[i][MCPWM_GEN_INDEX_HIGH], &dt_config));
    }
    dt_config = (mcpwm_dead_time_config_t){
        .negedge_delay_ticks = CUSTOM_DEADTIME,
        .flags.invert_output = true,
    };
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][MCPWM_GEN_INDEX_HIGH], generators[i][MCPWM_GEN_INDEX_LOW], &dt_config));
    }

    ESP_LOGI(TAG, "Start the MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Create a timer to update COMPARE VALUE");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &mcpwm_update_TIMER_CMPA,
        .arg = NULL,
        .name = "update_timer_cmpa_value"};
    esp_timer_handle_t update_timer_cmpa_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &update_timer_cmpa_timer));

    ESP_ERROR_CHECK(esp_timer_start_periodic(update_timer_cmpa_timer, UPDATE_LOOP_PERIOD_US));
    ESP_LOGI(TAG, "SPWM Update: Update CMPA value every, %i us", UPDATE_LOOP_PERIOD_US);

    ESP_LOGI(TAG, "Start LED2 as GPIO for DEBUG");

    while (1)
    {
        if (update_duty_flag)
        {
            for (int i = 0; i < 3; i++)
            {

                if (TIMER_CMPA[i] > ((int)TIMER_PERIOD / 2))
                {
                    TIMER_CMPA[i] = ((int)TIMER_PERIOD / 2);
                }
                else if (TIMER_CMPA[i] < 0)
                {
                    TIMER_CMPA[i] = 0;
                }

                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], TIMER_CMPA[i]));
            }
            update_duty_flag = 0;
        }
    }
}
