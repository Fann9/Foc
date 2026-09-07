#include "multi_button.h"
#include <string.h>

enum ButtonState {
    STATE_NONE = 0,
    STATE_PRESS_DOWN,
    STATE_WAIT_REPEAT,
    STATE_LONG_PRESS,
};

static Button *head_handle = NULL;

static void emit_event(Button *handle, ButtonEvent event)
{
    handle->event = event;
    if (handle->cb[event]) handle->cb[event](handle);
}

void button_init(Button *handle, uint8_t (*pin_level)(uint8_t),
                 uint8_t active_level, uint8_t button_id)
{
    memset(handle, 0, sizeof(Button));
    handle->hal_button_level = pin_level;
    handle->button_level = pin_level(button_id);
    handle->active_level = active_level;
    handle->button_id = button_id;
    handle->ticks_interval = 5;
    handle->debounce_time = 15;
    handle->short_press_time = 400;
    handle->long_press_time = 1000;
    handle->event = BTN_NONE_PRESS;
}

void button_attach(Button *handle, ButtonEvent event, BtnCallback cb)
{
    if (event < BTN_NONE_PRESS) handle->cb[event] = cb;
}

void button_detach(Button *handle, ButtonEvent event)
{
    if (event < BTN_NONE_PRESS) handle->cb[event] = NULL;
}

int button_start(Button *handle)
{
    Button *current = head_handle;
    while (current) {
        if (current == handle) return -1;
        current = current->next;
    }
    handle->next = head_handle;
    head_handle = handle;
    return 0;
}

void button_stop(Button *handle)
{
    Button **current = &head_handle;
    while (*current) {
        if (*current == handle) {
            *current = handle->next;
            handle->next = NULL;
            return;
        }
        current = &(*current)->next;
    }
}

void button_ticks(void)
{
    for (Button *handle = head_handle; handle; handle = handle->next) {
        uint8_t gpio_level = handle->hal_button_level(handle->button_id);
        if (gpio_level != handle->button_level) {
            if (++handle->debounce_cnt >=
                handle->debounce_time / handle->ticks_interval) {
                handle->button_level = gpio_level;
                handle->debounce_cnt = 0;
            }
        } else {
            handle->debounce_cnt = 0;
        }

        switch (handle->state) {
            case STATE_NONE:
                if (handle->button_level == handle->active_level) {
                    handle->state = STATE_PRESS_DOWN;
                    handle->ticks_count = 0;
                    emit_event(handle, BTN_PRESS_DOWN);
                }
                break;

            case STATE_PRESS_DOWN:
                if (handle->button_level != handle->active_level) {
                    emit_event(handle, BTN_PRESS_UP);
                    if (handle->repeat < 4) handle->repeat++;
                    handle->ticks_count = 0;
                    handle->state = STATE_WAIT_REPEAT;
                } else if (++handle->ticks_count * handle->ticks_interval >=
                           handle->long_press_time) {
                    handle->repeat = 0;
                    handle->state = STATE_LONG_PRESS;
                    emit_event(handle, BTN_LONG_PRESS_START);
                }
                break;

            case STATE_WAIT_REPEAT:
                if (handle->button_level == handle->active_level) {
                    handle->state = STATE_PRESS_DOWN;
                    handle->ticks_count = 0;
                    emit_event(handle, BTN_PRESS_REPEAT);
                } else if (++handle->ticks_count * handle->ticks_interval >=
                           handle->short_press_time) {
                    static const ButtonEvent click_event[4] = {
                        BTN_SINGLE_CLICK, BTN_DOUBLE_CLICK,
                        BTN_TRIPLE_CLICK, BTN_QUADRUPLE_CLICK
                    };
                    emit_event(handle, click_event[handle->repeat - 1]);
                    handle->repeat = 0;
                    handle->state = STATE_NONE;
                }
                break;

            case STATE_LONG_PRESS:
                if (handle->button_level != handle->active_level) {
                    handle->state = STATE_NONE;
                    handle->event = BTN_NONE_PRESS;
                } else {
                    emit_event(handle, BTN_LONG_PRESS_HOLD);
                }
                break;

            default:
                button_reset(handle);
                break;
        }
    }
}

ButtonEvent button_get_event(Button *handle)
{
    return (ButtonEvent)handle->event;
}

uint8_t button_get_repeat_count(Button *handle)
{
    return handle->repeat;
}

void button_reset(Button *handle)
{
    handle->state = STATE_NONE;
    handle->repeat = 0;
    handle->ticks_count = 0;
    handle->event = BTN_NONE_PRESS;
}

int button_is_pressed(Button *handle)
{
    return handle->button_level == handle->active_level;
}
