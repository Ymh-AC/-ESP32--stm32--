#include"Key.h"

void Key_Init(void){
    gpio_config_t key_InitStructure={0};
    key_InitStructure.pin_bit_mask=(1ULL<<Key_up_gpio)|(1ULL<<Key_down_gpio)|(1ULL<<Key_left_gpio)|(1ULL<<Key_right_gpio)|(1ULL<<Key_left_x_gpio)|(1ULL<<Key_right_x_gpio);
    key_InitStructure.mode=GPIO_MODE_INPUT;
    key_InitStructure.pull_up_en=GPIO_PULLUP_ENABLE;
    key_InitStructure.pull_down_en=GPIO_PULLDOWN_DISABLE;
    key_InitStructure.intr_type=GPIO_INTR_DISABLE;
    gpio_config(&key_InitStructure);
}

Key_Type Key_Get(void){
    if(gpio_get_level(Key_up_gpio)==0){
        vTaskDelay(5);
        if(gpio_get_level(Key_up_gpio)==0){
            while(gpio_get_level(Key_up_gpio)==0){
                vTaskDelay(1);
            }
            return KEY_UP;
        }
    }
    else if(gpio_get_level(Key_down_gpio)==0){
        vTaskDelay(5);
        if(gpio_get_level(Key_down_gpio)==0){
            while(gpio_get_level(Key_down_gpio)==0){
                vTaskDelay(1);
            }
            return KEY_DOWN;
        }
    }
    else if(gpio_get_level(Key_left_gpio)==0){
        vTaskDelay(5);
        if(gpio_get_level(Key_left_gpio)==0){
            while(gpio_get_level(Key_left_gpio)==0){
                vTaskDelay(1);
            }
            return KEY_LEFT;
        }
    }
    else if(gpio_get_level(Key_right_gpio)==0){
        vTaskDelay(5);
        if(gpio_get_level(Key_right_gpio)==0){
            while(gpio_get_level(Key_right_gpio)==0){
                vTaskDelay(1);
            }
            return KEY_RIGHT;
        }
    }
    else if(gpio_get_level(Key_left_x_gpio)==0){
        vTaskDelay(5);
        if(gpio_get_level(Key_left_x_gpio)==0){
            while(gpio_get_level(Key_left_x_gpio)==0){
                vTaskDelay(1);
            }   
            return KEY_LEFT_X;
        }
    }
    else if(gpio_get_level(Key_right_x_gpio)==0){
        //长按超过1s
        TickType_t count1=xTaskGetTickCount();
        vTaskDelay(5);
        if(gpio_get_level(Key_right_x_gpio)==0){
            while(gpio_get_level(Key_right_x_gpio)==0){
                vTaskDelay(1);
            }
            TickType_t count2=xTaskGetTickCount();
            if((count2-count1)>pdMS_TO_TICKS(1000)){
                return KEY_RIGHT_X_LONG;
            }
            else{
                return KEY_RIGHT_X;
            }
        }
        
    }
    return KEY_NONE;
}

