// Tower Kit documentation https://tower.hardwario.com/
// SDK API description https://sdk.hardwario.com/
// Forum https://forum.hardwario.com/

#include <application.h>

// LED instance
twr_led_t led;

uint16_t last_adc = 0;
bool led_state = false;


twr_scheduler_task_id_t reset_led;

// Led strip
static uint32_t _twr_module_power_led_strip_dma_buffer[LED_STRIP_COUNT * LED_STRIP_TYPE * 2];
const twr_led_strip_buffer_t led_strip_buffer =
{
    .type = LED_STRIP_TYPE,
    .count = LED_STRIP_COUNT,
    .buffer = _twr_module_power_led_strip_dma_buffer
};

static struct
{
    enum
    {
        LED_STRIP_SHOW_COLOR = 0,
        LED_STRIP_SHOW_COMPOUND = 1,
        LED_STRIP_SHOW_EFFECT = 2,
        LED_STRIP_SHOW_THERMOMETER = 3

    } show;
    twr_led_strip_t self;
    uint32_t color;
    struct
    {
        uint8_t data[TWR_RADIO_NODE_MAX_COMPOUND_BUFFER_SIZE];
        int length;
    } compound;
    struct
    {
        float temperature;
        int8_t min;
        int8_t max;
        uint8_t white_dots;
        float set_point;
        uint32_t color;

    } thermometer;

    twr_scheduler_task_id_t update_task_id;

} led_strip = { .show = LED_STRIP_SHOW_COLOR, .color = 0 };

static void _adc_event_handler(twr_adc_channel_t channel, twr_adc_event_t event, void *param)
{
    (void) channel;
    (void) param;

    if (event == TWR_ADC_EVENT_DONE)
    {
        uint16_t adc;
        float voltage;
        twr_adc_async_get_value(TWR_ADC_CHANNEL_A4, &adc);
        if(adc - last_adc > 7000)
        {
            //twr_led_strip_effect_pulse_color(&led_strip.self, 0xFF000000, 30);
            twr_led_blink(&led, 5);
            //twr_scheduler_plan_from_now(reset_led, 5000);
        }
        last_adc = adc;
        twr_log_debug("%d", adc);
    }
}

void twr_radio_node_on_state_get(uint64_t *id, uint8_t state_id)
{
    (void) id;

    if (state_id == TWR_RADIO_NODE_STATE_POWER_MODULE_RELAY)
    {
        bool state = twr_module_power_relay_get_state();

        twr_radio_pub_state(TWR_RADIO_PUB_STATE_POWER_MODULE_RELAY, &state);
    }
    else if (state_id == TWR_RADIO_NODE_STATE_LED)
    {
        twr_radio_pub_state(TWR_RADIO_PUB_STATE_LED, &led_state);
    }
}

void twr_radio_node_on_state_set(uint64_t *id, uint8_t state_id, bool *state)
{
    (void) id;

    if (state_id == TWR_RADIO_NODE_STATE_POWER_MODULE_RELAY)
    {
        twr_module_power_relay_set_state(*state);

        twr_radio_pub_state(TWR_RADIO_PUB_STATE_POWER_MODULE_RELAY, state);
    }
    else if (state_id == TWR_RADIO_NODE_STATE_LED)
    {
        led_state = *state;

        twr_led_set_mode(&led, led_state ? TWR_LED_MODE_ON : TWR_LED_MODE_OFF);

        twr_radio_pub_state(TWR_RADIO_PUB_STATE_LED, &led_state);
    }
}

void led_strip_update_task(void *param)
{
    (void) param;

    if (!twr_led_strip_is_ready(&led_strip.self))
    {
        twr_scheduler_plan_current_now();

        return;
    }

    twr_led_strip_write(&led_strip.self);

    twr_scheduler_plan_current_relative(250);
}

void led_strip_fill(void)
{
    if (led_strip.show == LED_STRIP_SHOW_COLOR)
    {
        twr_led_strip_fill(&led_strip.self, led_strip.color);
    }
    else if (led_strip.show == LED_STRIP_SHOW_COMPOUND)
    {
        int from = 0;
        int to;
        uint8_t *color;

        for (int i = 0; i < led_strip.compound.length; i += 5)
        {
            color = led_strip.compound.data + i + 1;
            to = from + led_strip.compound.data[i];

            for (;(from < to) && (from < LED_STRIP_COUNT); from++)
            {
                twr_led_strip_set_pixel_rgbw(&led_strip.self, from, color[3], color[2], color[1], color[0]);
            }

            from = to;
        }
    }
    else if (led_strip.show == LED_STRIP_SHOW_THERMOMETER)
    {
        twr_led_strip_thermometer(&led_strip.self, led_strip.thermometer.temperature, led_strip.thermometer.min, led_strip.thermometer.max, led_strip.thermometer.white_dots, led_strip.thermometer.set_point, led_strip.thermometer.color);
    }
}

void twr_radio_node_on_led_strip_color_set(uint64_t *id, uint32_t *color)
{
    (void) id;

    twr_led_strip_effect_stop(&led_strip.self);

    led_strip.color = *color;

    led_strip.show = LED_STRIP_SHOW_COLOR;

    led_strip_fill();

    twr_scheduler_plan_now(led_strip.update_task_id);
}

void twr_radio_node_on_led_strip_brightness_set(uint64_t *id, uint8_t *brightness)
{
    (void) id;

    twr_led_strip_set_brightness(&led_strip.self, *brightness);

    led_strip_fill();

    twr_scheduler_plan_now(led_strip.update_task_id);
}

void twr_radio_node_on_led_strip_compound_set(uint64_t *id, uint8_t *compound, size_t length)
{
    (void) id;

    twr_led_strip_effect_stop(&led_strip.self);

    memcpy(led_strip.compound.data, compound, length);

    led_strip.compound.length = length;

    led_strip.show = LED_STRIP_SHOW_COMPOUND;

    led_strip_fill();

    twr_scheduler_plan_now(led_strip.update_task_id);
}

void twr_radio_node_on_led_strip_effect_set(uint64_t *id, twr_radio_node_led_strip_effect_t type, uint16_t wait, uint32_t *color)
{
    (void) id;

    switch (type) {
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_TEST:
        {
            twr_led_strip_effect_test(&led_strip.self);
            break;
        }
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_RAINBOW:
        {
            twr_led_strip_effect_rainbow(&led_strip.self, wait);
            break;
        }
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_RAINBOW_CYCLE:
        {
            twr_led_strip_effect_rainbow_cycle(&led_strip.self, wait);
            break;
        }
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_THEATER_CHASE_RAINBOW:
        {
            twr_led_strip_effect_theater_chase_rainbow(&led_strip.self, wait);
            break;
        }
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_COLOR_WIPE:
        {
            twr_led_strip_effect_color_wipe(&led_strip.self, *color, wait);
            break;
        }
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_THEATER_CHASE:
        {
            twr_led_strip_effect_theater_chase(&led_strip.self, *color, wait);
            break;
        }
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_STROBOSCOPE:
        {
            twr_led_strip_effect_stroboscope(&led_strip.self, *color, wait);
            break;
        }
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_ICICLE:
        {
            twr_led_strip_effect_icicle(&led_strip.self, *color, wait);
            break;
        }
        case TWR_RADIO_NODE_LED_STRIP_EFFECT_PULSE_COLOR:
        {
            twr_led_strip_effect_pulse_color(&led_strip.self, *color, wait);
            break;
        }
        default:
            return;
    }

    led_strip.show = LED_STRIP_SHOW_EFFECT;
}

void twr_radio_node_on_led_strip_thermometer_set(uint64_t *id, float *temperature, int8_t *min, int8_t *max, uint8_t *white_dots, float *set_point, uint32_t *set_point_color)
{
    (void) id;

    twr_led_strip_effect_stop(&led_strip.self);

    led_strip.thermometer.temperature = *temperature;
    led_strip.thermometer.min = *min;
    led_strip.thermometer.max = *max;
    led_strip.thermometer.white_dots = *white_dots;

    if (set_point != NULL)
    {
        led_strip.thermometer.set_point = *set_point;
        led_strip.thermometer.color = *set_point_color;
    }
    else
    {
        led_strip.thermometer.set_point = *min - 1;
    }

    led_strip.show = LED_STRIP_SHOW_THERMOMETER;

    led_strip_fill();

    twr_scheduler_plan_now(led_strip.update_task_id);
}

void reset_led_strip()
{
    twr_led_strip_effect_stop(&led_strip.self);
    twr_led_strip_fill(&led_strip.self, 0x00000000);
}

// Application initialization function which is called once after boot
void application_init(void)
{
    twr_system_pll_enable();
    twr_log_init(TWR_LOG_LEVEL_DEBUG, TWR_LOG_TIMESTAMP_OFF);

    twr_led_init(&led, TWR_GPIO_LED, 0, 0);
    twr_led_blink(&led, 3);

    twr_adc_init();
    twr_adc_set_event_handler(TWR_ADC_CHANNEL_A4, _adc_event_handler, NULL);
    twr_adc_resolution_set(TWR_ADC_CHANNEL_A4, TWR_ADC_RESOLUTION_12_BIT);
    twr_adc_oversampling_set(TWR_ADC_CHANNEL_A4, TWR_ADC_OVERSAMPLING_256);

    // Initialize power module
    //twr_module_power_init();

    //reset_led = twr_scheduler_register(reset_led_strip, NULL, TWR_TICK_INFINITY);

    // Initialize led-strip on power module
    /*twr_led_strip_init(&led_strip.self, twr_module_power_get_led_strip_driver(), &led_strip_buffer);
    led_strip.update_task_id = twr_scheduler_register(led_strip_update_task, NULL, 0);*/
}

// Application task function (optional) which is called peridically if scheduled
void application_task()
{
    twr_adc_async_measure(TWR_ADC_CHANNEL_A4);

    twr_scheduler_plan_current_relative(200);
}
