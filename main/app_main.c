/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"

// #include "ps4.h"



// #define BLINK_LED_PIN GPIO_NUM_10
// #define MOTOR_IN1 GPIO_NUM_0
// #define MOTOR_IN2 GPIO_NUM_1
// #define BLINK_LED_BLINK 500

// #define LEDC_TIMER              LEDC_TIMER_0
// #define LEDC_SERVO_TIMER              LEDC_TIMER_1
// #define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (2) // Define the output GPIO
// #define SERVO_OUTPUT_IO          (5) // Define the output GPIO
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
// #define SERVO_CHANNEL            LEDC_CHANNEL_1
// #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
// #define LEDC_SERVO_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits 分辨率2的13次方
// #define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
// #define LEDC_SERVO_DUTY               (480) // Set duty to 5%. ((2 ** 13) - 1) * 7.5% = 12.75  0.5ms-2.5ms舵机调整范围  pwm波 20ms周期，2.5%-12.5%占空比 调整0-180度 中间值为1.5ms，占空比为7.5%  409分辨率
// #define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
// #define LEDC_SERVO_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 5 kHz

static const char *TAG = "MQTT_EXAMPLE";
static char mqtt_msg[512];
//0x 80 80 80 80 08 00 xx 00 00  ps4初始数据
//左摇杆第0和1位，右摇杆第2和3位，上下左右，方块，三角，圆，叉第4位，前端左右四键第5位
static int psdata0=128;
static int psdata1=128;
static int psdata2=128;
static int psdata3=128;
static int psdata4=8;
static int psdata5=0;
//mqtt接收消息缓冲区
// static float initspeed=0.5;
// //小车初始速度系数
// static int ledstatus=0;
// //车灯状态
// static int x=128;
// static int y=128;
// //获取方向盘油门数值
// static int xangle=128;
// static int yangle=128;
//方向盘油门中值

// static int pulseWithmidle = 480;
// static int freq = 50;      // 频率(20ms周期)
// static int channel = 8;    // 
// static int resolution = 8; // 分辨率
/*
如果 ESP32-C3 的定时器选用了RTCxM_CLK作为其时钟源，驱动会通过内部校准来得知这个时钟源的实际频率。这样确保了输出PWM信号频率的精准性。

ESP32-C3 的所有定时器共用一个时钟源。因此 ESP32-C3 不支持给不同的定时器配置不同的时钟源。
ESP32-C3 LEDC 时钟源特性

时钟名称      时钟频率      时钟功能
APB_CLK      80 MHz       /

RTC20M_CLK   ~20 MHz      支持动态调频（DFS）功能，支持Light-sleep模式

XTAL_CLK     40 MHz       支持动态调频（DFS）功能

*/



static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

char *mystrncpy(const char *string,int n){//要求截取的字符串不可以改变，但指向字符串的指针可以改变
char *p=string;
    if(p==NULL){//如果截取的字符串是空的直接返回
        return NULL;
    }else{
        int i=0;
    while(*p!='\0'){//循环直到达n个字符串终止
    if(i==n){
        break;
    }
    i++;
    p++;
    }
    *(p++)='\0';//赋值结束字符串
    return string;
    }
}

//数字转为字符串函数
char* itoa(int num,char* str,int radix)
{
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//索引表
    unsigned unum;//存放要转换的整数的绝对值,转换的整数可能是负数
    int i=0,j,k;//i用来指示设置字符串相应位，转换之后i其实就是字符串的长度；转换后顺序是逆序的，有正负的情况，k用来指示调整顺序的开始位置;j用来指示调整顺序时的交换。
 
    //获取要转换的整数的绝对值
    if(radix==10&&num<0)//要转换成十进制数并且是负数
    {
        unum=(unsigned)-num;//将num的绝对值赋给unum
        str[i++]='-';//在字符串最前面设置为'-'号，并且索引加1
    }
    else unum=(unsigned)num;//若是num为正，直接赋值给unum
 
    //转换部分，注意转换后是逆序的
    do
    {
        str[i++]=index[unum%(unsigned)radix];//取unum的最后一位，并设置为str对应位，指示索引加1
        unum/=radix;//unum去掉最后一位
 
    }while(unum);//直至unum为0退出循环
 
    str[i]='\0';//在字符串最后添加'\0'字符，c语言字符串以'\0'结束。
 
    //将顺序调整过来
    if(str[0]=='-') k=1;//如果是负数，符号不用调整，从符号后面开始调整
    else k=0;//不是负数，全部都要调整
 
    char temp;//临时变量，交换两个值时用到
    for(j=k;j<=(i-1)/2;j++)//头尾一一对称交换，i其实就是字符串的长度，索引最大值比长度少1
    {
        temp=str[j];//头部赋值给临时变量
        str[j]=str[i-1+k-j];//尾部赋值给头部
        str[i-1+k-j]=temp;//将临时变量的值(其实就是之前的头部值)赋给尾部
    }
 
    return str;//返回转换后的字符串
 
}
// static void example_ledc_init(void)
// {
//     // Prepare and then apply the LEDC PWM timer configuration
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode       = LEDC_MODE,
//         .timer_num        = LEDC_TIMER,
//         .duty_resolution  = LEDC_DUTY_RES,
//         .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

//     // Prepare and then apply the LEDC PWM channel configuration
//     ledc_channel_config_t ledc_channel = {
//         .speed_mode     = LEDC_MODE,
//         .channel        = LEDC_CHANNEL,
//         .timer_sel      = LEDC_TIMER,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .gpio_num       = LEDC_OUTPUT_IO,
//         .duty           = 0, // Set duty to 0%
//         .hpoint         = 0
//     };
//     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
// }

// static void example_servo_init(void)
// {
//     // Prepare and then apply the LEDC PWM timer configuration
//     ledc_timer_config_t servo_timer = {
//         .speed_mode       = LEDC_MODE,
//         .timer_num        = LEDC_SERVO_TIMER,
//         .duty_resolution  = LEDC_SERVO_DUTY_RES,    //2的13次方分辨率
//         .freq_hz          = LEDC_SERVO_FREQUENCY,  // Set output frequency at 50 Hz
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

//     // Prepare and then apply the LEDC PWM channel configuration
//     ledc_channel_config_t servo_channel = {
//         .speed_mode     = LEDC_MODE,
//         .channel        = SERVO_CHANNEL,
//         .timer_sel      = LEDC_SERVO_TIMER,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .gpio_num       = SERVO_OUTPUT_IO ,
//         .duty           = LEDC_SERVO_DUTY, // Set duty to 0%
//         .hpoint         = 0
//     };
//     ESP_ERROR_CHECK(ledc_channel_config(&servo_channel));
// }

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
esp_mqtt_client_handle_t client = "";
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED://mqtt连接事件
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/PS4ESP32_89860000000000000000", "connected", 0, 1, 0);//将89860000000000000000改为你的物联网卡ICCID，保持和小车端一致
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        // esp_mqtt_client_subscribe(client, "/car_89860000000000000000", 1);
        msg_id = esp_mqtt_client_subscribe(client, "/driver_89860000000000000000", 1);//将89860000000000000000改为你的物联网卡ICCID，保持和小车端一致
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED://mqtt断开事件
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        // ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA://mqtt接收数据事件
        // ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        // printf("DATA=%.*s\r\n", event->data_len, event->data);
        // printf(ty)
        // printf((event->data).toString());
        // char orderstr[event->data_len];
        // strcpy(orderstr,event->data);
        if (event->data_len >= (sizeof(mqtt_msg) - 1))
        {
            ESP_LOGE(TAG, "Received MQTT message size [%d] more than expected [%d]", event->data_len, (sizeof(mqtt_msg) - 1));
            return ESP_FAIL;
        }


        // if(event->data_len>1){
        //     // printf(strlen(event->data));
        //     // printf("DATA=%.*s\r\n",event->data);
        // }
        // char orderstr=event->data;
        // printf(orderstr);
        // strcpy(mqtt_msg, mystrncpy(event->data,event->data_len));
        // // printf(mqtt_msg);
        // // printf("\r\n");
        // if(strcmp(mqtt_msg,"start")==0){
        //     // printf("turn on or off led\r\n");
        //     if(ledstatus){
        //         //开灯
        //         gpio_set_level(BLINK_LED_PIN, 0);
        //         ledstatus = 0;
        //         return;
        //     }else{
        //         //关灯
        //         gpio_set_level(BLINK_LED_PIN, 1);
        //         ledstatus = 1;
        //         return;
        //     }
        // }else if (strcmp(mqtt_msg,"one")==0)
        // {
        //     initspeed =1;
        //     return;
        // }else if (strcmp(mqtt_msg,"half")==0)
        // {
        //     initspeed=0.5;
        //     return;
        // }else if (strcmp(mqtt_msg,"halfhalf")==0)
        // {
        //     initspeed=0.25;
        //     return;
        // }
        // cJSON* cjson = cJSON_Parse(mqtt_msg);//将JSON字符串转换成JSON结构体
        // if(cjson == NULL)						//判断转换是否成功
        // {
        //         printf("cjson error...\r\n");
        // }
        // else
        // {
        //         // printf("%s\n",cJSON_Print(cjson));//打包成功调用cJSON_Print打印输出
        // }
            
        // // printf("/*********************以下就是提取的数据**********************/\n");
        //     x = cJSON_GetObjectItem(cjson,"x")->valueint;	//解析字符串
        //     // printf("x--->>%d\n",x);
        //     y = cJSON_GetObjectItem(cjson,"y")->valueint;	//解析字符串
        //     // printf("y--->>>%d\n",y);
        //     // char *name = cJSON_GetObjectItem(cjson,"name")->valuestring;	//解析字符串
	    //     // printf("%s\n",name);
        //     // int age = cJSON_GetObjectItem(cjson,"age")->valueint;			//解析整型
        //     // printf("%d\n",age);
        //     // double height = cJSON_GetObjectItem(cjson,"height")->valuedouble;	//解析双浮点型
        //     // printf("%.1f\n",height);
        //     // int gender = cJSON_GetObjectItem(cjson,"gender")->type; 	//解析逻辑值---输出逻辑值对应的宏定义数值
        //     // printf("%d\n",gender);

        //     cJSON_Delete(cjson);//清除结构体 
        //     if(xangle-x>0){
        //         //turn left左转
        //         // en2.servoWrite(parseInt(pulseWithmidle+(xangle-JSON.parse(message.toString()).x)*3.125))
        //             ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, SERVO_CHANNEL,pulseWithmidle+(xangle-x)*3.125));
        //             // Update duty to apply the new value
        //             ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, SERVO_CHANNEL));
        //         }
        //         if(x-xangle>0){
        //         //turn right右转
        //         // en2.servoWrite(parseInt(pulseWithmidle-(JSON.parse(message.toString()).x-xangle)*3.125))
        //          ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, SERVO_CHANNEL,pulseWithmidle-(x-xangle)*3.125));
        //             // Update duty to apply the new value
        //             ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, SERVO_CHANNEL));
        //         }
        //         if(x-xangle==0){
                
        //         // console.log('midllle-->>',JSON.parse(message.toString()).x-xangle)
        //         // en2.servoWrite(pulseWithmidle)
        //         // Set duty to 2.5%
        //         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, SERVO_CHANNEL, pulseWithmidle));
        //         // Update duty to apply the new value
        //         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, SERVO_CHANNEL));
        //         }
        //         if(yangle-y>0){
        //         //油门
        //         // en1.write((yangle-JSON.parse(message.toString()).y)*initspeed/128)
        //         // in1.writeSync(1)
        //         // in2.writeSync(0)
        //         // in1.digitalWrite(1)
        //         // in2.digitalWrite(0)
        //         gpio_set_level(MOTOR_IN1, 1);
        //         gpio_set_level(MOTOR_IN2, 0);
        //         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (yangle-y)*initspeed*64));
        //         // Update duty to apply the new value
        //         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        //         // console.log('y left-->>',yangle-JSON.parse(message.toString()).y)
        //         return;
        //         }
        //         if(y-yangle>0){
        //         //油门
        //         // en1.write((JSON.parse(message.toString()).y-yangle)*initspeed/128)
        //         // in1.writeSync(0)
        //         // in2.writeSync(1)
        //         // in1.digitalWrite(0)
        //         // in2.digitalWrite(1)
        //         gpio_set_level(MOTOR_IN1, 0);
        //         gpio_set_level(MOTOR_IN2, 1);
        //         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (y-yangle)*initspeed*64));
        //         // Update duty to apply the new value
        //         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        //         // console.log('y right-->>',JSON.parse(message.toString()).y-yangle)
        //         return;
        //         }
        //         if(y-yangle==0){
        //         //停止
        //         // in1.writeSync(1)
        //         // in2.writeSync(1)
        //         // in1.digitalWrite(1)
        //         // in2.digitalWrite(1)
        //         // console.log('stop-->>',JSON.parse(message.toString()).y-yangle)
        //         gpio_set_level(MOTOR_IN1, 1);
        //         gpio_set_level(MOTOR_IN2, 1);
        //         return;
        //     }
        //     return 0;
        
        
        
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
static char jsonstr[]="";
static char xstr[]="{\"x\":";
static char ystr[]=",\"y\":";
static char endstr[]="}";
char xstring[3] = {0};
char ystring[3] = {0};
//这部分变量用于拼接json字符串
void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        // ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        // ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);

        // printf("DATA=%d\r\n",param->input.data[0]);
        // printf("recivie-->>>\n");
        // strcpy(mqtt_msg, mystrncpy(param->input.data,param->input.length));
        // printf("mqtt_msg-->>>%s \n",mqtt_msg);
        // if()
        // esp_mqtt_client_publish(client, "/car_89860000000000000000", "start", 0, 1, 0);
        //数据有变化才进行mqtt数据上报。
        if(param->input.data[0]!=psdata0||param->input.data[1]!=psdata1||param->input.data[2]!=psdata2||param->input.data[3]!=psdata3||param->input.data[4]!=psdata4||param->input.data[5]!=psdata5){
            if(param->input.data[1]!=psdata1||param->input.data[2]!=psdata2){
                itoa(param->input.data[1],ystring,10);
                itoa(param->input.data[2],xstring,10);
                strcat(jsonstr,xstr);
                strcat(jsonstr,xstring);
                strcat(jsonstr,ystr);
                strcat(jsonstr,ystring);
                strcat(jsonstr,endstr);
                
                esp_mqtt_client_publish(client, "/car_89860000000000000000", jsonstr, 0, 1, 0);//将89860000000000000000改为你的物联网卡ICCID，保持和小车端一致
                strcpy(jsonstr, "");
            }
            psdata0=param->input.data[0];
            psdata1=param->input.data[1];
            psdata2=param->input.data[2];
            psdata3=param->input.data[3];
            psdata4=param->input.data[4];
            psdata5=param->input.data[5];
            if(psdata4==136){
                esp_mqtt_client_publish(client, "/car_89860000000000000000", "start", 0, 1, 0);//将89860000000000000000改为你的物联网卡ICCID，保持和小车端一致
                return;
            }
            if(psdata4==24){
                esp_mqtt_client_publish(client, "/car_89860000000000000000", "halfhalf", 0, 1, 0);//将89860000000000000000改为你的物联网卡ICCID，保持和小车端一致
                return;
            }
            if(psdata4==40){
                esp_mqtt_client_publish(client, "/car_89860000000000000000", "half", 0, 1, 0);//将89860000000000000000改为你的物联网卡ICCID，保持和小车端一致
                return;
            }
            if(psdata4==72){
                esp_mqtt_client_publish(client, "/car_89860000000000000000", "one", 0, 1, 0);//将89860000000000000000改为你的物联网卡ICCID，保持和小车端一致
            }

           
        }
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

#define SCAN_DURATION_SECONDS 20

void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        // printf("connect results %s \n",results);
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI1111: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        // printf("connect cr %s \n",cr);
        if (cr) {
            //open the last result
            printf("connect last one \n");
            printf("connect bda %s \n",cr->bda);
            // printf("connect transport %s \n",cr->transport);
            // printf("connect ble.addr_type %s \n",cr->ble.addr_type);
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
        //free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };

    // static esp_mqtt_client_config_t    mqtt_cfg = {
    //         .host= IOT_CORE_MQTT_BROKER_URL,
    //         .event_handle = mqtt_event_handler,//注册回调函数
    //         .port = 1883,
    //         .username = mqtt_token,
    //         .client_id = my_clinet_id
    //         };
//定义并初始化MQTT Client配置结构体
//client_id 默认使用的是ESP32_%CHIPID%的形式；

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    //启动客户端，连接服务器
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// void controller_event_cb( ps4_t ps4, ps4_event_t event )
// {
//     // Event handling here...
//     if ( ps4.status.battery >= ps3_status_battery_high )
//     print("The controller still has sufficient battery charge");

// if ( ps4.status.charging )
//     print("Controller is charging");

// if ( ps4.button.triangle )
//     print("Currently pressing the trangle button");

// if ( ps4.analog.stick.lx < 0 )
//     print("Currently pulling analog stick to the left");

// if ( event.button_down.cross )
//     print("The user started pressing the X button");

// if ( event.button_up.cross )
//     print("The user released the X button");

// if ( event.analog_changed.stick.lx )
//     print("The user has moved the left stick sideways");
// }

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);



    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

     
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );
     xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);
    
    mqtt_app_start();

    

   

}
