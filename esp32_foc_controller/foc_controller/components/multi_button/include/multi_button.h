#ifndef _MULTI_BUTTON_H_
#define _MULTI_BUTTON_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BTN_PRESS_DOWN = 0,
    BTN_PRESS_UP,
    BTN_PRESS_REPEAT,
    BTN_SINGLE_CLICK,
    BTN_DOUBLE_CLICK,
    BTN_TRIPLE_CLICK,
    BTN_QUADRUPLE_CLICK,
    BTN_LONG_PRESS_START,
    BTN_LONG_PRESS_HOLD,
    BTN_NONE_PRESS
} ButtonEvent;

typedef void (*BtnCallback)(void *);

typedef struct Button {
    uint8_t (*hal_button_level)(uint8_t button_id);
    uint8_t button_level;
    uint8_t active_level;
    uint8_t button_id;
    uint8_t debounce_cnt;
    uint16_t debounce_time;
    uint16_t short_press_time;
    uint16_t long_press_time;
    uint16_t ticks_interval;
    uint16_t ticks_count;
    uint8_t repeat;
    uint8_t event;
    uint8_t state;

    BtnCallback cb[BTN_NONE_PRESS];
    struct Button *next;
} Button;

void button_init(Button *handle, uint8_t (*pin_level)(uint8_t),
                 uint8_t active_level, uint8_t button_id);
void button_attach(Button *handle, ButtonEvent event, BtnCallback cb);
void button_detach(Button *handle, ButtonEvent event);
int button_start(Button *handle);
void button_stop(Button *handle);
void button_ticks(void);
ButtonEvent button_get_event(Button *handle);
uint8_t button_get_repeat_count(Button *handle);
void button_reset(Button *handle);
int button_is_pressed(Button *handle);

#ifdef __cplusplus
}
#endif
#endif
