    /*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "libuvc/libuvc.h"
#include "libuvc_adapter.h"
#include "usb/usb_host.h"
#include "msc_host.h"
#include "msc_host_vfs.h"
#include "esp_vfs_fat.h"
#include "es8311.h"

#include "jpegd2.h"
#include "bsp/esp-box.h"
#include "lvgl.h"
#include "freertos/event_groups.h"

#define TAG "ESP-BOX"

/* Default screen brightness */
#define DEFAULT_BRIGHTNESS  (50)

/* Buffer for reading/writing to I2S driver. Same length as SPIFFS buffer and I2S buffer, for optimal read/write performance.
   Recording audio data path:
   I2S peripheral -> I2S buffer (DMA) -> App buffer (RAM) -> SPIFFS buffer -> External SPI Flash.
   Vice versa for playback. */
#define BUFFER_SIZE     (1024)
#define SAMPLE_RATE     (22050)
#define DEFAULT_VOLUME  (70)
/* The recording will be RECORDING_LENGTH * BUFFER_SIZE long (in bytes)
   With sampling frequency 22050 Hz and 16bit mono resolution it equals to ~3.715 seconds */
#define RECORDING_LENGTH (160)

/* Camera settings */
#define CAMERA_FPS      30
#define CAMERA_WIDTH    320
#define CAMERA_HEIGHT   240
#define CAMERA_FORMAT   UVC_COLOR_FORMAT_MJPEG // UVC_COLOR_FORMAT_YUYV
#define CAMERA_TIMEOUT_US  2000000

#define CAMERA_PID 0
#define CAMERA_VID 0
#define CAMERA_SERIAL_NUMBER NULL

#define VIDEO_LVGL_WIDTH    320
#define VIDEO_LVGL_HEIGHT   200

/* USB drive root */
#define USB_DRIVE_PATH  "/usb"

#define UVC_CHECK(exp) do {                 \
    uvc_error_t _err_ = (exp);              \
    if(_err_ < 0) {                         \
        ESP_LOGE(TAG, "UVC error: %s",      \
                 uvc_error_string(_err_));  \
        assert(0);                          \
    }                                       \
} while(0)

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef enum
{
    APP_UVC_STATE_INIT,
    APP_UVC_STATE_WAITING,
    APP_UVC_STATE_RUN,
    APP_UVC_STATE_RUNNING,
    APP_UVC_STATE_END,
} app_uvc_state_t;

typedef enum
{
    APP_MSC_STATE_INIT,
    APP_MSC_STATE_WAITING,
    APP_MSC_STATE_CONNECTING,
    APP_MSC_STATE_CONNECTED,
    APP_MSC_STATE_END,
} app_msc_state_t;

typedef enum
{
    APP_FILE_TYPE_UNKNOWN,
    APP_FILE_TYPE_TXT,
    APP_FILE_TYPE_IMG,
    APP_FILE_TYPE_WAV,
} app_file_type_t;

// Very simple WAV header, ignores most fields
typedef struct __attribute__((packed))
{
    uint8_t ignore_0[22];
    uint16_t num_channels;
    uint32_t sample_rate;
    uint8_t ignore_1[6];
    uint16_t bits_per_sample;
    uint8_t ignore_2[4];
    uint32_t data_size;
    uint8_t data[];
} dumb_wav_header_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/
static void app_lvgl_show_files(const char * path);

/*******************************************************************************
* Local variables
*******************************************************************************/

static lv_obj_t * tabview = NULL;
static EventGroupHandle_t app_flags;
static bool camera_disconnected;

/* Camera */
static app_uvc_state_t app_uvc_state = 0;
static size_t cam_buf_size;
static lv_color_t * cam_buffer = NULL;
static lv_obj_t * camera_canvas = NULL;
static lv_obj_t * lbl_cam_info = NULL;
static uvc_context_t *camera_ctx = NULL;
static uvc_device_handle_t *camera_devh = NULL;
static int64_t last_frame = 0;
static int32_t camera_zoom = 0;
static lv_obj_t * lbl_zoom = NULL;

/* MSC */
static app_msc_state_t app_msc_state = 0;
static uint8_t msc_device_address = 0;
static msc_host_device_handle_t msc_device = NULL;
static msc_host_vfs_handle_t vfs_handle = NULL;
static lv_obj_t * lbl_usb_drive_info = NULL;
static lv_obj_t * list_usb_drive = NULL;
static lv_obj_t * img_usb_drive = NULL;
static char usb_drive_current_path[250];

/* Audio */
static SemaphoreHandle_t audio_mux;
static i2s_chan_handle_t i2s_tx_chan;
static es8311_handle_t es8311_dev = NULL;
static bool play_file_repeat = false;
static bool play_file_stop = false;
static char usb_drive_play_file[250];
static lv_obj_t * play_btn = NULL;
/*******************************************************************************
* Private functions
*******************************************************************************/

static char *uvc_error_string(uvc_error_t error)
{
    switch (error) {
    case UVC_SUCCESS: return "UVC_SUCCESS";
    case UVC_ERROR_IO: return "UVC_ERROR_IO";
    case UVC_ERROR_INVALID_PARAM: return "UVC_ERROR_INVALID_PARAM";
    case UVC_ERROR_ACCESS: return "UVC_ERROR_ACCESS";
    case UVC_ERROR_NO_DEVICE: return "UVC_ERROR_NO_DEVICE";
    case UVC_ERROR_NOT_FOUND: return "UVC_ERROR_NOT_FOUND";
    case UVC_ERROR_BUSY: return "UVC_ERROR_BUSY";
    case UVC_ERROR_TIMEOUT: return "UVC_ERROR_TIMEOUT";
    case UVC_ERROR_OVERFLOW: return "UVC_ERROR_OVERFLOW";
    case UVC_ERROR_PIPE: return "UVC_ERROR_PIPE";
    case UVC_ERROR_INTERRUPTED: return "UVC_ERROR_INTERRUPTED";
    case UVC_ERROR_NO_MEM: return "UVC_ERROR_NO_MEM";
    case UVC_ERROR_NOT_SUPPORTED: return "UVC_ERROR_NOT_SUPPORTED";
    case UVC_ERROR_INVALID_DEVICE: return "UVC_ERROR_INVALID_DEVICE";
    case UVC_ERROR_INVALID_MODE: return "UVC_ERROR_INVALID_MODE";
    case UVC_ERROR_CALLBACK_EXISTS: return "UVC_ERROR_CALLBACK_EXISTS";
    default: return "Unknown error";
    }
}

// Handles common USB host library events
static void usb_lib_handler_task(void *args)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        // Give ready_to_uninstall_usb semaphore to indicate that USB Host library
        // can be deinitialized, and terminate this task.
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
        }

        if(event_flags != 0)
            ESP_LOGW(TAG, "USB flags: 0x%lx", event_flags);
    }

    vTaskDelete(NULL);
}

static esp_err_t initialize_usb_host_lib(void)
{
    TaskHandle_t task_handle = NULL;

    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        return err;
    }

    if (xTaskCreate(usb_lib_handler_task, "usb_events", 4096, NULL, 2, &task_handle) != pdPASS) {
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static bool lcd_write_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
#if 0
    uint16_t cam_zoom = 256;
    uint16_t cam_w = VIDEO_LVGL_WIDTH;
    uint16_t cam_h = VIDEO_LVGL_HEIGHT;

    lv_img_dsc_t img;
    img.data = (void *)cam_buffer;
    img.header.cf = LV_IMG_CF_TRUE_COLOR;
    img.header.w = CAMERA_WIDTH;
    img.header.h = CAMERA_HEIGHT;
    
    /* Zoom plus */
    if (camera_zoom > 0){
        cam_zoom = 256 + (2.55)*((float)camera_zoom);
    } else if(camera_zoom < 0) {
        cam_zoom = (2.55)*(float)(100+camera_zoom);
        cam_w = (CAMERA_WIDTH/100)*(100+camera_zoom);
        cam_h = (CAMERA_HEIGHT/100)*(100+camera_zoom);
    }
    
    //ESP_LOGI(TAG, "Camera zoom (%d%%): %d, w: %d, h: %d", camera_zoom, cam_zoom, cam_w, cam_h);
    
    if (camera_zoom != 0) {
        lv_canvas_transform(camera_canvas, &img, 0, cam_zoom, 0, 0, 0 , 0, false);
        lv_canvas_set_buffer(camera_canvas, cam_buffer, cam_w, cam_h, LV_IMG_CF_TRUE_COLOR);
        lv_obj_center(camera_canvas);
    } else {
        lv_canvas_set_buffer(camera_canvas, cam_buffer, cam_w, cam_h, LV_IMG_CF_TRUE_COLOR);
        lv_obj_center(camera_canvas);
    }
#endif
    /* Invalidate object, when data changed in buffer */
    lv_obj_invalidate(camera_canvas);

    return true;
}

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void frame_callback(uvc_frame_t *frame, void *ptr)
{
    bsp_display_lock(0);

    /* JPEG decode */
    mjpegdraw(frame->data, frame->data_bytes, (uint8_t*)cam_buffer, lcd_write_bitmap);

    bsp_display_unlock();

    last_frame = esp_timer_get_time();
}

static void libuvc_adapter_cb(libuvc_adapter_event_t event)
{
    xEventGroupSetBits(app_flags, event);
}

static void app_uvc_init(void)
{
    libuvc_adapter_config_t config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = libuvc_adapter_cb
    };
    
    libuvc_adapter_set_config(&config);

    UVC_CHECK( uvc_init(&camera_ctx, NULL) );
    

    ESP_LOGI(TAG, "Waiting for device");
    app_uvc_state = APP_UVC_STATE_WAITING;
}

static void app_uvc_waiting(void)
{
    uvc_error_t res;
    uvc_device_t *dev;

    res = uvc_find_device(camera_ctx, &dev, CAMERA_PID, CAMERA_VID, CAMERA_SERIAL_NUMBER);
    if (res == UVC_ERROR_NO_DEVICE) {
        //ESP_LOGI(TAG, "Waiting for device");
    } else if (res == 0) {
        ESP_LOGI(TAG, "Device found");

        UVC_CHECK( uvc_open(dev, &camera_devh) );
    
        bsp_display_lock(0);
        lv_label_set_text_static(lbl_cam_info, "Camera found. Loading...");
        lv_obj_set_style_text_opa(lbl_cam_info, 255, 0);
        bsp_display_unlock();

        /* Show Camera tab */
        bsp_display_lock(0);
        lv_tabview_set_act(tabview, 0, true);
        bsp_display_unlock();

        app_uvc_state = APP_UVC_STATE_RUN;
    }
}

static void app_uvc_run(void)
{
    uvc_error_t res;
    uvc_stream_ctrl_t ctrl;

    // Negotiate stream profile
    res = uvc_get_stream_ctrl_format_size(camera_devh, &ctrl, CAMERA_FORMAT, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS );
    if (res != UVC_SUCCESS) {
        ESP_LOGE(TAG, "Negotiating streaming format failed, trying again...");
        return;
    }

    // dwMaxPayloadTransferSize has to be overwritten to MPS (maximum packet size)
    // supported by ESP32-S2(S3), as libuvc selects the highest possible MPS.
    ctrl.dwMaxPayloadTransferSize = 512;

    uvc_print_stream_ctrl(&ctrl, stderr);

    uint32_t trials = 3;
    do {
        printf("uvc_start_streaming\n");
        res = uvc_start_streaming(camera_devh, &ctrl, frame_callback, NULL, 0);
    } while(res < 0 && trials--);

    ESP_LOGI(TAG, "Streaming...");
    
    bsp_display_lock(0);
    lv_obj_set_style_text_opa(lbl_cam_info, 0, 0);
    bsp_display_unlock();

    last_frame = esp_timer_get_time();

    app_uvc_state = APP_UVC_STATE_RUNNING;
}

static void app_uvc_running(void)
{
    int64_t now = esp_timer_get_time();
    bool timeout_expired = (now-last_frame) > CAMERA_TIMEOUT_US;

    if (timeout_expired || camera_disconnected) {
        const char *message = timeout_expired ? "Camera Timeout!" 
                                              : "Camera disconneced!";
        ESP_LOGI(TAG, "%s", message);
    
        bsp_display_lock(0);
        memset(cam_buffer, 0, cam_buf_size);
        lv_label_set_text_static(lbl_cam_info, message);
        lv_obj_set_style_text_opa(lbl_cam_info, 255, 0);
        bsp_display_unlock();

        camera_disconnected = false;
        app_uvc_state = APP_UVC_STATE_END;
    }

}

static void app_uvc_end(void)
{
    uvc_stop_streaming(camera_devh);
    ESP_LOGI(TAG, "Done streaming.");

    uvc_close(camera_devh);
    camera_devh = NULL;

    uvc_exit(camera_ctx);
    camera_ctx = NULL;

    ESP_LOGI(TAG, "UVC exited");

    app_uvc_state = APP_UVC_STATE_INIT;
}

static void app_uvc(void)
{
    switch(app_uvc_state)
    {
    case APP_UVC_STATE_INIT:
        app_uvc_init();
        break;
    case APP_UVC_STATE_WAITING:
        app_uvc_waiting();
        break;
    case APP_UVC_STATE_RUN:
        app_uvc_run();
        break;
    case APP_UVC_STATE_RUNNING:
        app_uvc_running();
        break;
    case APP_UVC_STATE_END:
        app_uvc_end();
        break;
    }
}

static void app_lvgl_add_text(const char * text)
{
    lv_list_add_text(list_usb_drive, text);
}

static void folder_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if(code == LV_EVENT_CLICKED) {
        const char * foldername = lv_list_get_btn_text(list_usb_drive, obj);
        if (foldername != NULL) {
            strcat(usb_drive_current_path, "/");
            strcat(usb_drive_current_path, foldername);
            ESP_LOGI(TAG, "Clicked: \"%s\"", usb_drive_current_path);
            app_lvgl_show_files(usb_drive_current_path);
        }

    }
}

static void close_window_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        memset(cam_buffer, 0, cam_buf_size);
        lv_obj_del(e->user_data);
    }
}

static bool file_bitmap_cb(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
    lv_canvas_set_buffer(img_usb_drive, data, w, h, LV_IMG_CF_TRUE_COLOR);
    lv_obj_center(img_usb_drive);
    lv_obj_invalidate(img_usb_drive);

    return true;
}

static void show_window(const char * path, app_file_type_t type)
{
	struct stat st;
    lv_obj_t * label = NULL;
    lv_obj_t * btn;
    lv_obj_t * win = lv_win_create(lv_scr_act(), 40);
    lv_win_add_title(win, path);

    btn = lv_win_add_btn(win, LV_SYMBOL_CLOSE, 60);
    lv_obj_add_event_cb(btn, close_window_handler, LV_EVENT_CLICKED, win);

    lv_obj_t * cont = lv_win_get_content(win);  /*Content can be added here*/

    label = lv_label_create(cont);
    lv_obj_set_width(label, 290);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label, "");
    lv_obj_center(label);
    
    if (type == APP_FILE_TYPE_IMG) {
        img_usb_drive = lv_canvas_create(cont);
    }


    if (type == APP_FILE_TYPE_TXT || type == APP_FILE_TYPE_IMG) {
        /* Get file size */
        int f = stat(path, &st);
        if(f == 0) {
            uint32_t filesize = (uint32_t) st.st_size;
            char * file_buf = heap_caps_malloc(filesize+1, MALLOC_CAP_DMA);
            if (file_buf == NULL) {
                lv_label_set_text(label, "Not enough memory!");
                return;
            }

            /* Open file */
            f = open(path, O_RDONLY);
            if (f > 0) {
                /* Read file */
                read(f, file_buf, filesize);
                if (type == APP_FILE_TYPE_TXT && label) {
                    file_buf[filesize] = 0;
                    lv_label_set_text(label, file_buf);
                } else if (img_usb_drive) {
                    ESP_LOGI(TAG, "Decoding JPEG image...");
                    /* JPEG decode */
                    mjpegdraw((uint8_t*)file_buf, filesize, (uint8_t*)cam_buffer, file_bitmap_cb);
                }

                close(f);
            } else {
                lv_label_set_text(label, "File not found!");
            }

            free(file_buf);
        } else {
            lv_label_set_text(label, "File not found!");
        }
    } else if(label) {
        lv_label_set_text(label, "Unsupported file type!");
    }
    
}

static void play_file(void * arg)
{   
    char * path = arg;
    FILE *file = NULL;
    int16_t *wav_bytes = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DEFAULT);
    if (wav_bytes == NULL) {
        ESP_LOGE(TAG, "Not enough memory for playing!");
        goto END;
    }

    file = fopen(path, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "%s file does not exist!", path);
        goto END;
    }
    
    /* Read WAV header file */
    dumb_wav_header_t wav_header;
    if (fread((void *)&wav_header, 1, sizeof(wav_header), file) != sizeof(wav_header)) {
        ESP_LOGW(TAG, "Error in reading file");
        goto END;
    }
    ESP_LOGI(TAG, "Number of channels: %d", (int)wav_header.num_channels);
    ESP_LOGI(TAG, "Bits per sample: %d", (int)wav_header.bits_per_sample);
    ESP_LOGI(TAG, "Sample rate: %d", (int)wav_header.sample_rate);
    ESP_LOGI(TAG, "Data size: %d", (int)wav_header.data_size);

    uint32_t bytes_send_to_i2s = 0;
    do {
        bytes_send_to_i2s = 0;
        fseek(file, sizeof(wav_header), SEEK_SET);
        while (bytes_send_to_i2s < wav_header.data_size) {
            if (play_file_stop) {
                goto END;
            }
            xSemaphoreTake(audio_mux, portMAX_DELAY);

            /* Get data from SPIFFS */
            size_t bytes_read_from_spiffs = fread(wav_bytes, 1, BUFFER_SIZE, file);

            /* Send it to I2S */
            size_t i2s_bytes_written;
            assert(i2s_tx_chan);
            ESP_ERROR_CHECK(i2s_channel_write(i2s_tx_chan, wav_bytes, bytes_read_from_spiffs, &i2s_bytes_written, pdMS_TO_TICKS(500)));
            bytes_send_to_i2s += i2s_bytes_written;
            xSemaphoreGive(audio_mux);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    } while (play_file_repeat);
    

    
END:
    if (file) {
        fclose(file);
    }

    if (wav_bytes) {
        free(wav_bytes);
    }

    if (play_btn) {
        bsp_display_lock(0);
        lv_obj_clear_state(play_btn, LV_STATE_DISABLED);
        bsp_display_unlock();
    }

    vTaskDelete(NULL);
}

static void play_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if(code == LV_EVENT_CLICKED) {
        play_file_stop = false;
        lv_obj_add_state(obj, LV_STATE_DISABLED);
        xTaskCreate(play_file, "audio_task", 4096, e->user_data, 6, NULL);
    }
}

static void stop_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        play_file_stop = true;
    }
}

static void repeat_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if(code == LV_EVENT_VALUE_CHANGED) {
        play_file_repeat = ( (lv_obj_get_state(obj) & LV_STATE_CHECKED) ? true : false);
    }
}

static void volume_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    assert(slider != NULL);

    int32_t volume = lv_slider_get_value(slider);
    if (es8311_dev) {
        es8311_voice_volume_set(es8311_dev, volume, NULL);
    }
}

static void close_window_wav_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        memset(cam_buffer, 0, cam_buf_size);
        lv_obj_del(e->user_data);
        play_file_stop = true;
        
        xSemaphoreTake(audio_mux, portMAX_DELAY);
        vSemaphoreDelete(audio_mux);

        bsp_audio_poweramp_enable(false);
        es8311_delete(es8311_dev);
        es8311_dev = NULL;
    }
}

static void show_window_wav(const char * path)
{
    lv_obj_t * label;
    lv_obj_t * btn;
    lv_obj_t * win = lv_win_create(lv_scr_act(), 40);
    lv_win_add_title(win, path);

    strcpy(usb_drive_play_file, path);

    play_file_repeat = false;
    
    const es8311_clock_config_t clk_cfg = BSP_ES8311_SCLK_CONFIG(SAMPLE_RATE);

    /* Create and configure ES8311 I2C driver */
    es8311_dev = es8311_create(BSP_I2C_NUM, ES8311_ADDRRES_0);
    es8311_init(es8311_dev, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    es8311_voice_volume_set(es8311_dev, DEFAULT_VOLUME, NULL);

    bsp_audio_init(NULL, &i2s_tx_chan, NULL);
    bsp_audio_poweramp_enable(true);

    audio_mux = xSemaphoreCreateMutex();
    assert(audio_mux);

    btn = lv_win_add_btn(win, LV_SYMBOL_CLOSE, 60);
    lv_obj_add_event_cb(btn, close_window_wav_handler, LV_EVENT_CLICKED, win);

    lv_obj_t * cont = lv_win_get_content(win);  /*Content can be added here*/

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * cont_row = lv_obj_create(cont);
    lv_obj_set_size(cont_row, VIDEO_LVGL_WIDTH-20, 80);
    lv_obj_align(cont_row, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_top(cont_row, 2, 0);
    lv_obj_set_style_pad_bottom(cont_row, 2, 0);
    lv_obj_set_flex_align(cont_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* Play button */
    play_btn = lv_btn_create(cont_row);
    label = lv_label_create(play_btn);
    lv_label_set_text_static(label, LV_SYMBOL_PLAY);
    lv_obj_add_event_cb(play_btn, play_event_cb, LV_EVENT_CLICKED, (char*)usb_drive_play_file);

    /* Stop button */
    btn = lv_btn_create(cont_row);
    label = lv_label_create(btn);
    lv_label_set_text_static(label, LV_SYMBOL_STOP);
    lv_obj_add_event_cb(btn, stop_event_cb, LV_EVENT_CLICKED, NULL);

    /* Repeat button */
    btn = lv_btn_create(cont_row);
    label = lv_label_create(btn);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
    lv_label_set_text_static(label, LV_SYMBOL_LOOP);
    lv_obj_add_event_cb(btn, repeat_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    cont_row = lv_obj_create(cont);
    lv_obj_set_size(cont_row, VIDEO_LVGL_WIDTH-20, 80);
    lv_obj_align(cont_row, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_top(cont_row, 2, 0);
    lv_obj_set_style_pad_bottom(cont_row, 2, 0);
    lv_obj_set_flex_align(cont_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* Volume */
    label = lv_label_create(cont_row);
    lv_label_set_text_static(label, "Volume: ");

    /* Slider */
    lv_obj_t * slider = lv_slider_create(cont_row);
    lv_obj_set_width(slider, VIDEO_LVGL_WIDTH-180);
    lv_slider_set_range(slider, 0, 90);
    lv_slider_set_value(slider, DEFAULT_VOLUME, false);
    lv_obj_center(slider);
    lv_obj_add_event_cb(slider, volume_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

}

static app_file_type_t get_file_type(char * filepath)
{
    assert(filepath != NULL);

    /* Find last dot */
    for (int i = (strlen(filepath)-1); i>=0; i--) {
        if (filepath[i] == '.') {
            
            if (strcmp(&filepath[i+1], "JPG") == 0 || strcmp(&filepath[i+1], "jpg") == 0)
                return APP_FILE_TYPE_IMG;
            else if(strcmp(&filepath[i+1], "TXT") == 0 || strcmp(&filepath[i+1], "txt") == 0)
                return APP_FILE_TYPE_TXT;
            else if(strcmp(&filepath[i+1], "WAV") == 0 || strcmp(&filepath[i+1], "wav") == 0)
                return APP_FILE_TYPE_WAV;

            break;
        }
    }

    return APP_FILE_TYPE_UNKNOWN;
}

static void file_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if(code == LV_EVENT_CLICKED) {
        char filepath[250];
        const char * filename = lv_list_get_btn_text(list_usb_drive, obj);

        strcpy(filepath, usb_drive_current_path);
        strcat(filepath, "/");
        strcat(filepath, filename);

        ESP_LOGI(TAG, "Clicked: %s", lv_list_get_btn_text(list_usb_drive, obj));
        app_file_type_t filetype = get_file_type(filepath);
        if (filetype == APP_FILE_TYPE_WAV) {
            show_window_wav(filepath);
        } else {
            show_window(filepath, filetype);
        }
    }
}

static void remove_last_folder(char * str)
{
    assert(str != NULL);

    for (int i = (strlen(str)-1); i>=0; i--) {
        if (str[i] == '/') {
            str[i] = '\0';
            break;
        }
    }
}

static void back_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        remove_last_folder(usb_drive_current_path);
        ESP_LOGI(TAG, "Clicked back to: \"%s\"", usb_drive_current_path);
        app_lvgl_show_files(usb_drive_current_path);
    }
}

static void app_lvgl_add_back(void)
{
    lv_obj_t * btn;

    btn = lv_list_add_btn(list_usb_drive, LV_SYMBOL_LEFT, "Back");
    lv_obj_set_style_bg_color(btn, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_text_color(btn, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_add_event_cb(btn, back_handler, LV_EVENT_CLICKED, NULL);
}

static void app_lvgl_add_file(const char * filename)
{
    lv_obj_t * btn;

    btn = lv_list_add_btn(list_usb_drive, LV_SYMBOL_FILE, filename);
    lv_obj_set_style_bg_color(btn, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_text_color(btn, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_add_event_cb(btn, file_handler, LV_EVENT_CLICKED, NULL);
}

static void app_lvgl_add_folder(const char * filename)
{
    lv_obj_t * btn;

    btn = lv_list_add_btn(list_usb_drive, LV_SYMBOL_DIRECTORY, filename);
    lv_obj_set_style_bg_color(btn, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_text_color(btn, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_add_event_cb(btn, folder_handler, LV_EVENT_CLICKED, NULL);
}

static void app_lvgl_show_files(const char * path)
{
	struct dirent * dir;
	DIR * d;

    /* Clean all items in the list */
    lv_obj_clean(list_usb_drive);

    /* Current path */
    app_lvgl_add_text(path);

    /* Not root -> Add back button */
    if (strcmp(path, USB_DRIVE_PATH) != 0) {
        app_lvgl_add_back();
    }

	/* Open directory */
	d = opendir(path);
	if(d != NULL)
    {
        while ((dir = readdir(d)) != NULL)
        {
            if (dir->d_type == DT_DIR) {
                app_lvgl_add_folder(dir->d_name);
            } else {
                app_lvgl_add_file(dir->d_name);
            }
        }

        closedir(d);
    }
}

static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    if (event->event == MSC_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "MSC device connected");
        msc_device_address = event->device.address;
        if(app_msc_state == APP_MSC_STATE_WAITING)
            app_msc_state = APP_MSC_STATE_CONNECTING;
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "MSC device disconnected");
        app_msc_state = APP_MSC_STATE_END;
    }
}

static void app_msc_init(void)
{
    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 2048,
        .callback = msc_event_cb,
    };
    ESP_ERROR_CHECK( msc_host_install(&msc_config) );

    app_msc_state = APP_MSC_STATE_WAITING;
}

static void print_device_info(msc_host_device_info_t *info)
{
    const size_t megabyte = 1024 * 1024;
    uint64_t capacity = ((uint64_t)info->sector_size * info->sector_count) / megabyte;

    printf("Device info:\n");
    printf("\t Capacity: %llu MB\n", capacity);
    printf("\t Sector size: %u\n", (unsigned int)info->sector_size);
    printf("\t Sector count: %u\n", (unsigned int)info->sector_count);
    printf("\t PID: 0x%4X \n", info->idProduct);
    printf("\t VID: 0x%4X \n", info->idVendor);
    wprintf(L"\t iProduct: %S \n", info->iProduct);
    wprintf(L"\t iManufacturer: %S \n", info->iManufacturer);
    wprintf(L"\t iSerialNumber: %S \n", info->iSerialNumber);
}

static void app_msc_connect(void)
{
    msc_host_device_info_t info;

    /* Show USB tab */
    bsp_display_lock(0);
    lv_tabview_set_act(tabview, 1, true);
    lv_obj_set_style_text_opa(lbl_usb_drive_info, 0, 0);
    bsp_display_unlock();

    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 1024,
    };

    ESP_ERROR_CHECK( msc_host_install_device(msc_device_address, &msc_device) );
    //msc_host_print_descriptors(msc_device);

    ESP_ERROR_CHECK( msc_host_get_device_info(msc_device, &info) );
    print_device_info(&info);
    
    ESP_ERROR_CHECK( msc_host_vfs_register(msc_device, USB_DRIVE_PATH, &mount_config, &vfs_handle) );

    ESP_LOGI(TAG, "MSC host installed and registered as \"%s\".", USB_DRIVE_PATH);

    bsp_display_lock(0);
    strcpy(usb_drive_current_path, USB_DRIVE_PATH);
    app_lvgl_show_files(USB_DRIVE_PATH);
    bsp_display_unlock();

    app_msc_state = APP_MSC_STATE_CONNECTED;
}

static void app_msc_end(void)
{
    ESP_ERROR_CHECK( msc_host_vfs_unregister(vfs_handle) );
    ESP_ERROR_CHECK( msc_host_uninstall_device(msc_device) );

    ESP_LOGI(TAG, "MSC host unregistered and uninstalled.");

    vfs_handle = NULL;
    msc_device = NULL;
    msc_device_address = 0;

   // ESP_ERROR_CHECK( msc_host_uninstall() );
    //ESP_LOGI(TAG, "USB Uninitialized.");

    /* Show USB tab */
    bsp_display_lock(0);
    lv_obj_set_style_text_opa(lbl_usb_drive_info, 255, 0);
    /* Clean all items in the list */
    lv_obj_clean(list_usb_drive);
    bsp_display_unlock();

    app_msc_state = APP_MSC_STATE_WAITING;
}

static void app_msc(void)
{
    switch(app_msc_state)
    {
    case APP_MSC_STATE_INIT:
        app_msc_init();
        break;
    case APP_MSC_STATE_WAITING:
        break;
    case APP_MSC_STATE_CONNECTING:
        app_msc_connect();
        break;
    case APP_MSC_STATE_CONNECTED:
        break;
    case APP_MSC_STATE_END:
        app_msc_end();
        break;
    }
}

static void scroll_begin_event(lv_event_t * e)
{
    /*Disable the scroll animations. Triggered when a tab button is clicked */
    if(lv_event_get_code(e) == LV_EVENT_SCROLL_BEGIN) {
        lv_anim_t * a = lv_event_get_param(e);
        if(a)  a->time = 300;
    }
}

static void slider_camera_zoom_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    assert(slider != NULL);

    camera_zoom = lv_slider_get_value(slider);
    lv_label_set_text_fmt(lbl_zoom, "Zoom (%3d%%): ", (int)(100+camera_zoom));
}

static void slider_brightness_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    assert(slider != NULL);

    bsp_display_brightness_set(lv_slider_get_value(slider));
}

static void app_lvgl_show_settings(lv_obj_t * screen)
{
    lv_obj_t * cont_row;
    lv_obj_t * slider;

    /* Disable scrolling in camera TAB */
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    /* Camera TAB style */
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_set_style_bg_color(screen, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_bg_grad_color(screen, lv_color_make(0x05, 0x05, 0x05), 0);
    lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER,0);
    lv_obj_set_style_bg_opa(screen, 255, 0);

    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(screen, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

#if 0
    /* ZOOM */
    cont_row = lv_obj_create(screen);
    lv_obj_set_size(cont_row, VIDEO_LVGL_WIDTH-20, 80);
    lv_obj_align(cont_row, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_top(cont_row, 2, 0);
    lv_obj_set_style_pad_bottom(cont_row, 2, 0);
    lv_obj_set_flex_align(cont_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* Label */
    lbl_zoom = lv_label_create(cont_row);
	lv_obj_set_style_text_font(lbl_zoom, &lv_font_montserrat_14, 0);
	lv_label_set_text_static(lbl_zoom, "Zoom (100%): ");
	lv_obj_align(lbl_zoom, LV_ALIGN_LEFT_MID, 0, 0);

    /* Slider */
    slider = lv_slider_create(cont_row);
    lv_obj_set_width(slider, VIDEO_LVGL_WIDTH-180);
    lv_slider_set_mode(slider, LV_SLIDER_MODE_SYMMETRICAL);
    lv_slider_set_range(slider, -70, 0);
    lv_obj_center(slider);
    lv_obj_add_event_cb(slider, slider_camera_zoom_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
#endif

    /* Brightness */
    cont_row = lv_obj_create(screen);
    lv_obj_set_size(cont_row, VIDEO_LVGL_WIDTH-20, 80);
    lv_obj_align(cont_row, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_top(cont_row, 2, 0);
    lv_obj_set_style_pad_bottom(cont_row, 2, 0);
    lv_obj_set_flex_align(cont_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* Label */
    lv_obj_t * lbl = lv_label_create(cont_row);
	lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
	lv_label_set_text_static(lbl, "Brightness: ");
	lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);

    /* Slider */
    slider = lv_slider_create(cont_row);
    lv_obj_set_width(slider, VIDEO_LVGL_WIDTH-180);
    lv_slider_set_range(slider, 10, 100);
    lv_slider_set_value(slider, DEFAULT_BRIGHTNESS, false);
    lv_obj_center(slider);
    lv_obj_add_event_cb(slider, slider_brightness_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

}

static void app_lvgl_show_usb(lv_obj_t * screen)
{
    /* Disable scrolling in camera TAB */
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    /* Camera TAB style */
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_set_style_bg_color(screen, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_bg_grad_color(screen, lv_color_make(0x05, 0x05, 0x05), 0);
    lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER,0);
    lv_obj_set_style_bg_opa(screen, 255, 0);

    /* File list */
    list_usb_drive = lv_list_create(screen);
    lv_obj_set_size(list_usb_drive, 320, 200);
    lv_obj_set_style_bg_color(list_usb_drive, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_text_color(list_usb_drive, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_center(list_usb_drive);

    /* Info label */
    lbl_usb_drive_info = lv_label_create(screen);
	lv_obj_set_style_text_font(lbl_usb_drive_info, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_usb_drive_info, lv_color_make(0xFF, 0xFF, 0xFF), 0);
	lv_label_set_text_static(lbl_usb_drive_info, "USB drive not connected.");
    lv_obj_center(lbl_usb_drive_info);

}

static void app_lvgl_show_camera(lv_obj_t * screen)
{
    /* Disable scrolling in camera TAB */
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    /* Camera TAB style */
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_set_style_bg_color(screen, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_bg_grad_color(screen, lv_color_make(0x05, 0x05, 0x05), 0);
    lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER,0);
    lv_obj_set_style_bg_opa(screen, 255, 0);

    camera_canvas = lv_canvas_create(screen);
    lv_canvas_set_buffer(camera_canvas, cam_buffer, VIDEO_LVGL_WIDTH, VIDEO_LVGL_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_center(camera_canvas);

    lbl_cam_info = lv_label_create(camera_canvas);
	lv_obj_set_style_text_font(lbl_cam_info, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_cam_info, lv_color_make(0xFF, 0xFF, 0xFF), 0);
	lv_label_set_text_static(lbl_cam_info, "USB Camera not connected.");
    lv_obj_center(lbl_cam_info);

}

static void app_lvgl_show()
{
    bsp_display_lock(0);

    /* Tabview */
    tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 40);
    lv_obj_set_size(tabview, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_align(tabview, LV_ALIGN_BOTTOM_MID, 0, 0);
	lv_obj_set_style_text_font(tabview, &lv_font_montserrat_14, 0);
    lv_obj_add_event_cb(lv_tabview_get_content(tabview), scroll_begin_event, LV_EVENT_SCROLL_BEGIN, NULL);


    lv_obj_t * tab_btns = lv_tabview_get_tab_btns(tabview);
    lv_obj_set_style_bg_color(tab_btns, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_text_color(tab_btns, lv_palette_lighten(LV_PALETTE_GREEN, 5), 0);
    lv_obj_set_style_border_side(tab_btns, LV_BORDER_SIDE_BOTTOM, LV_PART_ITEMS | LV_STATE_CHECKED);

    /* Add tabs (the tabs are page (lv_page) and can be scrolled */
    lv_obj_t * tab_camera = lv_tabview_add_tab(tabview, LV_SYMBOL_VIDEO" Camera");
    lv_obj_t * tab_usb = lv_tabview_add_tab(tabview, LV_SYMBOL_USB" USB Drive");
    lv_obj_t * tab_settings = lv_tabview_add_tab(tabview, LV_SYMBOL_SETTINGS" Settings");

    /* Show camera tab page */
    app_lvgl_show_camera(tab_camera);

    /* Show USB tab page */
    app_lvgl_show_usb(tab_usb);

    /* Show settings tab page */
    app_lvgl_show_settings(tab_settings);

    bsp_display_unlock();
}

static EventBits_t wait_for_event(EventBits_t event, TickType_t timeout_ms)
{
    TickType_t tick = pdMS_TO_TICKS(timeout_ms);
    return xEventGroupWaitBits(app_flags, event, pdTRUE, pdFALSE, tick) & event;
}


void app_main(void)
{
    bsp_i2c_init();

    ESP_ERROR_CHECK( initialize_usb_host_lib() );

    bsp_display_start();
    bsp_display_brightness_set(DEFAULT_BRIGHTNESS);

    cam_buf_size = CAMERA_WIDTH * CAMERA_HEIGHT * sizeof(lv_color_t);
    cam_buffer = heap_caps_malloc(cam_buf_size, MALLOC_CAP_DEFAULT);
    assert(cam_buffer);
    memset(cam_buffer, 0, cam_buf_size);

    app_flags = xEventGroupCreate();
    assert(app_flags);

    app_lvgl_show();

    ESP_LOGI(TAG, "Display LVGL animation");

    while(1)
    {
        app_msc();
        
        /* When connected USB flash drive, not find UVC camera */
        if (app_msc_state != APP_MSC_STATE_CONNECTED)
            app_uvc();

        EventBits_t event = wait_for_event(UVC_DEVICE_CONNECTED | UVC_DEVICE_DISCONNECTED, 1000);

        if(event == UVC_DEVICE_DISCONNECTED) {
            camera_disconnected = true;
        } else {
            // Do nothing, run app_uvc() right after UVC_DEVICE_CONNECTED is received
        }
    }

}
