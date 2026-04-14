#include "web_server.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <cJSON.h>
#include <esp_system.h>
#include <lwip/sockets.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <fcntl.h>

#include "esp32_camera.h"

// Forward declarations for motor control functions
extern "C" void HandleMotorActionForApplication(int direction, int speed, int duration_ms, int priority);
extern "C" void HandleMotorActionForEmotion(const char* emotion);
extern "C" void HandleMotorActionForDance(uint8_t speed_percent);

static const char *TAG = "WebServer";

WebServer::WebServer()
    : server_handle_(nullptr), stream_server_handle_(nullptr), stream_port_(81) {
}

WebServer::~WebServer() {
    Stop();
}

bool WebServer::Start(int port) {
    ESP_LOGI(TAG, "Starting web server on port %d", port);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.max_uri_handlers = 18;
    config.max_open_sockets = 10;       // 增加并发连接（LWIP_MAX_SOCKETS=16，减去3个内部占用=13上限）
    config.backlog_conn = 5;            // 增加连接等待队列
    config.lru_purge_enable = true;     // 满时清除最不活跃连接
    config.recv_wait_timeout = 3;       // 接收超时3秒
    config.send_wait_timeout = 3;       // 发送超时3秒
    config.max_resp_headers = 8;        // 减少响应头数量

    if (httpd_start(&server_handle_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return false;
    }

    // 注册URI处理器
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &index_uri);

    httpd_uri_t control_uri = {
        .uri       = "/control",
        .method    = HTTP_POST,
        .handler   = control_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &control_uri);

    httpd_uri_t api_control_uri = {
        .uri       = "/api/control",
        .method    = HTTP_POST,
        .handler   = api_control_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_control_uri);

    httpd_uri_t api_motor_action_uri = {
        .uri       = "/api/motor/action",
        .method    = HTTP_POST,
        .handler   = api_motor_action_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_motor_action_uri);

    // 注意：/stream 处理器已移至独立的流媒体服务器（端口 81）

    httpd_uri_t debug_uri = {
        .uri       = "/api/debug/motor_test",
        .method    = HTTP_POST,
        .handler   = debug_motor_test_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &debug_uri);

    // 注册配置页面处理器
    httpd_uri_t config_uri = {
        .uri       = "/config",
        .method    = HTTP_GET,
        .handler   = config_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &config_uri);

    httpd_uri_t config_post_uri = {
        .uri       = "/config",
        .method    = HTTP_POST,
        .handler   = config_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &config_post_uri);

    httpd_uri_t api_config_get_uri = {
        .uri       = "/api/config",
        .method    = HTTP_GET,
        .handler   = api_config_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_config_get_uri);

    httpd_uri_t api_config_post_uri = {
        .uri       = "/api/config",
        .method    = HTTP_POST,
        .handler   = api_config_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_config_post_uri);

    // 注册摄像头翻转 API
    httpd_uri_t api_camera_flip_get_uri = {
        .uri       = "/api/camera/flip",
        .method    = HTTP_GET,
        .handler   = api_camera_flip_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_camera_flip_get_uri);

    httpd_uri_t api_camera_flip_post_uri = {
        .uri       = "/api/camera/flip",
        .method    = HTTP_POST,
        .handler   = api_camera_flip_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_camera_flip_post_uri);

    httpd_uri_t api_camera_photo_uri = {
        .uri       = "/api/camera/photo",
        .method    = HTTP_GET,
        .handler   = api_camera_photo_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_camera_photo_uri);

    httpd_uri_t api_camera_quality_uri = {
        .uri       = "/api/camera/quality",
        .method    = HTTP_POST,
        .handler   = api_camera_quality_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_camera_quality_uri);

    // 注册表情控制 API
    httpd_uri_t api_emotion_uri = {
        .uri       = "/api/emotion",
        .method    = HTTP_POST,
        .handler   = api_emotion_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_emotion_uri);

    ESP_LOGI(TAG, "Web server started successfully");

    // 启动独立的流媒体服务器（避免阻塞控制 API）
    if (!start_stream_server()) {
        ESP_LOGW(TAG, "Stream server failed to start, continuing without it");
    }

    return true;
}

bool WebServer::start_stream_server() {
    if (!camera_callback_) {
        ESP_LOGI(TAG, "Camera not available, skipping stream server");
        return true;
    }

    ESP_LOGI(TAG, "Starting stream server on port %d", stream_port_);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = stream_port_;
    config.ctrl_port = 0;  // 自动分配控制端口，避免与第一个httpd实例的ctrl_port冲突
    config.max_uri_handlers = 2;
    config.max_open_sockets = 5;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 3;
    config.send_wait_timeout = 3;

    if (httpd_start(&stream_server_handle_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start stream server");
        return false;
    }

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(stream_server_handle_, &stream_uri);

    ESP_LOGI(TAG, "Stream server started on port %d", stream_port_);
    return true;
}

void WebServer::Stop() {
    if (stream_server_handle_) {
        httpd_stop(stream_server_handle_);
        stream_server_handle_ = nullptr;
        ESP_LOGI(TAG, "Stream server stopped");
    }
    if (server_handle_) {
        httpd_stop(server_handle_);
        server_handle_ = nullptr;
        ESP_LOGI(TAG, "Web server stopped");
    }
}

void WebServer::SetMotorControlCallback(std::function<void(int direction, int speed)> callback) {
    motor_control_callback_ = callback;
}

void WebServer::InvokeMotorControl(int direction, int speed) {
    if (motor_control_callback_) {
        motor_control_callback_(direction, speed);
    }
}

void WebServer::SetMotorActionConfigCallback(std::function<MotorActionConfig()> get_callback,
                                           std::function<void(const MotorActionConfig&)> set_callback) {
    get_motor_config_callback_ = get_callback;
    set_motor_config_callback_ = set_callback;
}

void WebServer::SetEmotionCallback(std::function<void(const char* emotion)> callback) {
    emotion_callback_ = callback;
}

void WebServer::SetEmotion(const char* emotion) {
    if (emotion_callback_) {
        emotion_callback_(emotion);
    }
}

void WebServer::SetCameraCallback(std::function<Esp32Camera*()> callback) {
    camera_callback_ = callback;
}

void WebServer::SetCameraFlipCallback(std::function<std::pair<bool, bool>()> get_callback,
                                      std::function<void(bool, bool)> set_callback) {
    get_camera_flip_callback_ = get_callback;
    set_camera_flip_callback_ = set_callback;
}

void WebServer::SetCameraQualityCallback(std::function<bool(int)> callback) {
    camera_quality_callback_ = callback;
}

void WebServer::SetPowerSaveCallback(std::function<void(bool performance)> callback) {
    power_save_callback_ = callback;
}

void WebServer::SetCameraSnapshotCallback(std::function<bool()> callback) {
    camera_snapshot_callback_ = callback;
}

esp_err_t WebServer::api_camera_photo_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (!server->camera_callback_) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not available");
        return ESP_FAIL;
    }

    Esp32Camera* camera = server->camera_callback_();
    if (!camera || !camera->CaptureStreamFrame()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to capture frame");
        return ESP_FAIL;
    }

    // 在锁保护下读取帧尺寸，防止与 stream 并发写入冲突
    camera->LockFrame();
    size_t jpeg_size = camera->GetFrameSize();
    const uint8_t* jpeg_data = camera->GetFrameData();
    // 复制数据到独立缓冲区，避免后续被其他线程覆盖
    if (!jpeg_data || jpeg_size == 0) {
        camera->UnlockFrame();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Empty frame");
        return ESP_FAIL;
    }
    uint8_t* jpeg_copy = (uint8_t*)heap_caps_malloc(jpeg_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!jpeg_copy) {
        camera->UnlockFrame();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }
    memcpy(jpeg_copy, jpeg_data, jpeg_size);
    camera->UnlockFrame();

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char len_str[16];
    snprintf(len_str, sizeof(len_str), "%u", (unsigned)jpeg_size);
    httpd_resp_set_hdr(req, "Content-Length", len_str);
    httpd_resp_send(req, (const char*)jpeg_copy, jpeg_size);
    heap_caps_free(jpeg_copy);
    return ESP_OK;
}

esp_err_t WebServer::api_camera_quality_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }
    content[ret] = '\0';

    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *quality_item = cJSON_GetObjectItem(root, "quality");
    if (!quality_item || !cJSON_IsNumber(quality_item)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid quality field");
        return ESP_FAIL;
    }

    int quality = quality_item->valueint;
    bool success = false;
    if (server->camera_quality_callback_) {
        success = server->camera_quality_callback_(quality);
    }
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    char json[64];
    snprintf(json, sizeof(json), "{\"status\":\"%s\",\"quality\":%d}",
             success ? "ok" : "failed", quality);
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::api_emotion_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // 读取请求体
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }

    char buf[512];
    int ret = httpd_req_recv(req, buf, total_len);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    // 解析 JSON: {"emotion":"neutral"}
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *emotion_item = cJSON_GetObjectItem(json, "emotion");
    if (!emotion_item || !cJSON_IsString(emotion_item)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing emotion field");
        return ESP_FAIL;
    }

    server->SetEmotion(emotion_item->valuestring);
    cJSON_Delete(json);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::api_camera_flip_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (!server->get_camera_flip_callback_) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"h_mirror\":false,\"v_flip\":false}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    auto flip = server->get_camera_flip_callback_();
    char json[64];
    snprintf(json, sizeof(json), "{\"h_mirror\":%s,\"v_flip\":%s}",
             flip.first ? "true" : "false", flip.second ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::api_camera_flip_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }
    content[ret] = '\0';

    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *j_mirror = cJSON_GetObjectItem(root, "h_mirror");
    cJSON *j_flip = cJSON_GetObjectItem(root, "v_flip");

    bool h_mirror = cJSON_IsTrue(j_mirror);
    bool v_flip = cJSON_IsTrue(j_flip);
    cJSON_Delete(root);

    if (server->set_camera_flip_callback_) {
        server->set_camera_flip_callback_(h_mirror, v_flip);
    }

    char json[64];
    snprintf(json, sizeof(json), "{\"h_mirror\":%s,\"v_flip\":%s}",
             h_mirror ? "true" : "false", v_flip ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::index_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, server->get_html_page(), HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

esp_err_t WebServer::control_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    int direction, speed;
    server->parse_simple_control_command(content, direction, speed);

    // 调用电机控制回调（通过公共封装）
    server->InvokeMotorControl(direction, speed);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

esp_err_t WebServer::api_control_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[200];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    // 解析JSON数据
    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    int direction = cJSON_GetObjectItem(root, "direction")->valueint;
    int speed = cJSON_GetObjectItem(root, "speed")->valueint;

    cJSON_Delete(root);

    // 调用电机控制回调（通过公共封装）
    server->InvokeMotorControl(direction, speed);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// Simple struct passed to the stop task
struct debug_stop_param_t {
    WebServer* server;
    int duration_ms;
};

static void debug_stop_task(void* arg) {
    debug_stop_param_t* p = static_cast<debug_stop_param_t*>(arg);
    if (!p) vTaskDelete(NULL);
    vTaskDelay(pdMS_TO_TICKS(p->duration_ms));
    if (p->server) {
        p->server->InvokeMotorControl(0, 0); // stop via public wrapper
    }
    free(p);
    vTaskDelete(NULL);
}

esp_err_t WebServer::debug_motor_test_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(content);
    int direction = 4;
    int speed = 80;
    int duration = 1000;
    if (root) {
        cJSON* jdir = cJSON_GetObjectItem(root, "direction");
        cJSON* jspd = cJSON_GetObjectItem(root, "speed");
        cJSON* jdur = cJSON_GetObjectItem(root, "duration");
        if (cJSON_IsNumber(jdir)) direction = jdir->valueint;
        if (cJSON_IsNumber(jspd)) speed = jspd->valueint;
        if (cJSON_IsNumber(jdur)) duration = jdur->valueint;
        cJSON_Delete(root);
    }

    if (server) {
        server->InvokeMotorControl(direction, speed);
        // spawn a short-lived task to stop after duration ms
        debug_stop_param_t* p = (debug_stop_param_t*)malloc(sizeof(debug_stop_param_t));
        if (p) {
            p->server = server;
            p->duration_ms = duration;
            BaseType_t rc = xTaskCreate(debug_stop_task, "dbg_stop", 2048, p, tskIDLE_PRIORITY + 1, NULL);
            if (rc != pdPASS) {
                free(p);
            }
        }
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::api_motor_action_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* action_json = cJSON_GetObjectItem(root, "action");
    if (!cJSON_IsString(action_json)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid action parameter");
        return ESP_FAIL;
    }

    std::string action = action_json->valuestring;

    // Get motor configuration for durations
    MotorActionConfig config;
    if (server->get_motor_config_callback_) {
        config = server->get_motor_config_callback_();
    }

    // Call motor action functions directly (simulating MCP tool calls) with configured durations
    try {
        ESP_LOGI(TAG, "网页动作请求: %s", action.c_str());

        if (action == "move_forward") {
            ESP_LOGI(TAG, "执行前进动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.forward_duration_ms);
            HandleMotorActionForApplication(4, config.default_speed_percent, config.forward_duration_ms, 1);
        } else if (action == "move_backward") {
            ESP_LOGI(TAG, "执行后退动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.backward_duration_ms);
            HandleMotorActionForApplication(2, config.default_speed_percent, config.backward_duration_ms, 1);
        } else if (action == "spin_around") {
            ESP_LOGI(TAG, "执行转圈动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.spin_duration_ms);
            HandleMotorActionForApplication(3, config.default_speed_percent, config.spin_duration_ms, 1);
        } else if (action == "turn_left") {
            ESP_LOGI(TAG, "执行左转动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.left_turn_duration_ms);
            HandleMotorActionForApplication(3, config.default_speed_percent, config.left_turn_duration_ms, 1);
        } else if (action == "turn_right") {
            ESP_LOGI(TAG, "执行右转动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.right_turn_duration_ms);
            HandleMotorActionForApplication(1, config.default_speed_percent, config.right_turn_duration_ms, 1);
        } else if (action == "quick_forward") {
            ESP_LOGI(TAG, "执行快速前进动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.quick_forward_duration_ms);
            HandleMotorActionForApplication(4, config.default_speed_percent, config.quick_forward_duration_ms, 1);
        } else if (action == "quick_backward") {
            ESP_LOGI(TAG, "执行快速后退动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.quick_backward_duration_ms);
            HandleMotorActionForApplication(2, config.default_speed_percent, config.quick_backward_duration_ms, 1);
        } else if (action == "wiggle") {
            ESP_LOGI(TAG, "执行摆动动作 (情感:困惑)");
            server->SetEmotion("confused");
            HandleMotorActionForEmotion("confused");
        } else if (action == "dance") {
            ESP_LOGI(TAG, "执行跳舞动作 - 速度:%d%%", config.default_speed_percent);
            server->SetEmotion("excited");
            HandleMotorActionForDance(config.default_speed_percent);
        } else if (action == "stop") {
            ESP_LOGI(TAG, "执行停止动作");
            HandleMotorActionForApplication(0, 0, 0, 2); // stop, high priority
        } else if (action == "wake_up") {
            ESP_LOGI(TAG, "执行唤醒情感动作");
            server->SetEmotion("sleepy");  // 原版特殊动画：困倦效果
            HandleMotorActionForEmotion("wake");
        } else if (action == "happy") {
            ESP_LOGI(TAG, "执行开心情感动作");
            server->SetEmotion("laughing");  // 原版特殊动画：大笑动画
            HandleMotorActionForEmotion("happy");
        } else if (action == "sad") {
            ESP_LOGI(TAG, "执行悲伤情感动作");
            server->SetEmotion("crying");  // 原版特殊动画：哭泣动画
            HandleMotorActionForEmotion("sad");
        } else if (action == "thinking") {
            ESP_LOGI(TAG, "执行思考情感动作");
            server->SetEmotion("thinking");  // 原版特殊动画：思考动画
            HandleMotorActionForEmotion("thinking");
        } else if (action == "listening") {
            ESP_LOGI(TAG, "执行倾听情感动作");
            server->SetEmotion("wink");  // 原版特殊动画：眨眼动画
            HandleMotorActionForEmotion("listening");
        } else if (action == "speaking") {
            ESP_LOGI(TAG, "执行说话情感动作");
            server->SetEmotion("funny");  // 原版特殊动画：搞笑动画
            HandleMotorActionForEmotion("speaking");
        } else if (action == "excited") {
            ESP_LOGI(TAG, "执行兴奋情感动作");
            server->SetEmotion("shocked");  // 原版特殊动画：震惊动画
            HandleMotorActionForEmotion("excited");
        } else if (action == "loving") {
            ESP_LOGI(TAG, "执行爱慕情感动作");
            server->SetEmotion("kissy");  // 原版特殊动画：亲吻动画
            HandleMotorActionForEmotion("loving");
        } else if (action == "angry") {
            ESP_LOGI(TAG, "执行生气情感动作");
            server->SetEmotion("angry");  // 基础表情（生气）
            HandleMotorActionForEmotion("angry");
        } else if (action == "surprised") {
            ESP_LOGI(TAG, "执行惊讶情感动作");
            server->SetEmotion("surprised");  // 基础表情（惊讶）
            HandleMotorActionForEmotion("surprised");
        } else if (action == "confused") {
            ESP_LOGI(TAG, "执行困惑情感动作");
            server->SetEmotion("confused");  // 原版特殊动画：困惑动画
            HandleMotorActionForEmotion("confused");
        } else {
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown action");
            return ESP_FAIL;
        }
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Failed to execute motor action %s: %s", action.c_str(), e.what());
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to execute action");
        return ESP_FAIL;
    }

    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// 同步采集发送：避免 pipeline 缓冲延迟
esp_err_t WebServer::stream_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 所有变量声明必须在最前面（避免 goto 跳过初始化）
    int fd = -1;
    ssize_t ret = 0;
    size_t tx_buf_size = 64 * 1024;
    uint8_t* tx_buf = nullptr;
    int consecutive_failures = 0;
    const int max_consecutive_failures = 10;
    int frames_sent = 0;
    int frames_dropped = 0;
    struct timeval tv = {0, 50000};
    const char* response_headers = nullptr;

    ESP_LOGD(TAG, "Stream: handler entry, req=%p", (void*)req);

    if (!server->camera_callback_) {
        ESP_LOGW(TAG, "Stream: camera_callback_ is null");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not available");
        return ESP_FAIL;
    }

    fd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "Stream client connected, fd=%d", fd);

    // 流媒体开始时强制 WiFi 高性能模式，降低网络延迟
    if (server->power_save_callback_) {
        ESP_LOGI(TAG, "Stream: forcing WiFi PERFORMANCE mode");
        server->power_save_callback_(true);
    }

    // 手动发送 HTTP 响应头
    response_headers =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace;boundary=frame\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Expires: 0\r\n"
        "\r\n";
    ret = send(fd, response_headers, strlen(response_headers), 0);
    ESP_LOGI(TAG, "Stream: headers sent, ret=%d (expected %d)", (int)ret, (int)strlen(response_headers));
    if (ret < 0 || ret == 0) {
        ESP_LOGE(TAG, "Stream headers send failed, errno=%d, ret=%d", errno, (int)ret);
        // 必须 goto stream_end 恢复电源管理模式
        goto stream_end;
    }

    // 设置发送超时 50ms（防止单次 send() 无限阻塞）
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    // 分配 TX 缓冲区
    tx_buf = (uint8_t*)heap_caps_malloc(tx_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tx_buf) {
        ESP_LOGE(TAG, "Stream: TX buffer alloc failed");
        goto stream_end;
    }

    ESP_LOGD(TAG, "Stream: entering capture loop, fd=%d", fd);

    while (true) {
        Esp32Camera* camera = server->camera_callback_();
        if (!camera) {
            ESP_LOGW(TAG, "Stream: camera callback returned null (fail count: %d)", consecutive_failures + 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            consecutive_failures++;
            if (consecutive_failures > max_consecutive_failures) {
                ESP_LOGE(TAG, "Camera not available, stopping stream");
                break;
            }
            continue;
        }

        if (!camera->CaptureStreamFrame()) {
            ESP_LOGW(TAG, "Stream: CaptureStreamFrame failed (fail count: %d)", consecutive_failures + 1);
            consecutive_failures++;
            if (consecutive_failures > max_consecutive_failures) {
                ESP_LOGE(TAG, "Too many capture failures (%d), stopping stream", consecutive_failures);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        consecutive_failures = 0;

        // 在锁保护下复制帧数据，防止与拍照并发冲突
        camera->LockFrame();
        const uint8_t* jpeg_data = camera->GetFrameData();
        size_t jpeg_size = camera->GetFrameSize();

        if (!jpeg_data || jpeg_size == 0) {
            camera->UnlockFrame();
            ESP_LOGW(TAG, "Stream: frame has no data or size=0 after CaptureStreamFrame success");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 构建完整帧：multipart header + JPEG data（原子发送，不拆分）
        char hdr[128];
        int hdr_len = snprintf(hdr, sizeof(hdr),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            (unsigned)jpeg_size);
        size_t total_len = hdr_len + jpeg_size;

        if (total_len > tx_buf_size) {
            camera->UnlockFrame();
            ESP_LOGW(TAG, "Stream: frame too large (%u > %u), dropping", (unsigned)total_len, (unsigned)tx_buf_size);
            frames_dropped++;
            continue;
        }

        // 在锁保护下拷贝到 TX 缓冲，拷贝完即可释放
        memcpy(tx_buf, hdr, hdr_len);
        memcpy(tx_buf + hdr_len, jpeg_data, jpeg_size);
        camera->UnlockFrame();

        // 发送完整帧（send 有 50ms 超时保护，不会无限阻塞）
        size_t sent_total = 0;
        while (sent_total < total_len) {
            ssize_t sent = send(fd, tx_buf + sent_total, total_len - sent_total, 0);
            if (sent < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // 发送缓冲区满，丢弃此帧（保证原子性，不发不完整数据）
                    frames_dropped++;
                    goto next_frame;
                }
                // EPIPE：客户端已断开连接（TCP RST），或连接被对端关闭
                if (errno == EPIPE || errno == ECONNRESET) {
                    ESP_LOGI(TAG, "Stream: client disconnected, errno=%d", errno);
                    goto stream_end;
                }
                ESP_LOGW(TAG, "Stream: send failed fd=%d, errno=%d", fd, errno);
                goto stream_end;
            }
            if (sent == 0) {
                // 客户端半关闭或 socket 状态异常，继续循环会导致死锁
                ESP_LOGW(TAG, "Stream: send returned 0, client likely half-closed");
                goto stream_end;
            }
            sent_total += sent;
        }

        frames_sent++;
        if (frames_sent % 50 == 1) {
            ESP_LOGI(TAG, "Stream: %d frames sent, %d dropped", frames_sent, frames_dropped);
        }

next_frame:;
    }

stream_end:
    if (tx_buf) {
        heap_caps_free(tx_buf);
    }
    // 流媒体结束，恢复电源管理模式
    if (server->power_save_callback_) {
        ESP_LOGI(TAG, "Stream: restoring WiFi power save mode");
        server->power_save_callback_(false);
    }

    ESP_LOGI(TAG, "Stream handler exiting, fd=%d, total %d frames", fd, frames_sent);
    return ESP_OK;
}

void WebServer::parse_simple_control_command(const char* data, int& direction, int& speed) {
    // 简单解析格式：direction=X,speed=Y
    char* str = strdup(data);
    char* token = strtok(str, ",");

    direction = 0;
    speed = 0;

    while (token != NULL) {
        if (strstr(token, "direction=")) {
            direction = atoi(token + 10);
        } else if (strstr(token, "speed=")) {
            speed = atoi(token + 6);
        }
        token = strtok(NULL, ",");
    }

    free(str);
}

const char* WebServer::get_html_page() {
    static const char html_page[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>小智小车遥控器</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, 'Segoe UI', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 6px;
        }
        .container {
            background: rgba(255,255,255,0.95);
            border-radius: 16px;
            padding: 10px 14px 16px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.15);
            max-width: 400px;
            width: 100%;
        }
        .header { text-align: center; margin-bottom: 6px; }
        .header h1 { color: #333; font-size: 1.15em; margin-bottom: 1px; }
        .header .subtitle { color: #888; font-size: 0.7em; }

        /* 视频控制栏 - 左边视频状态，右边遥控状态 */
        .video-bar {
            display: flex; align-items: center; justify-content: space-between;
            padding: 6px 8px; background: #1a1a2e;
            border-radius: 10px; margin-bottom: 0;
        }
        .video-bar .left {
            display: flex; align-items: center; flex: 1; min-width: 0;
        }
        .video-bar .status-dot {
            width: 8px; height: 8px; border-radius: 50%;
            background: #555; flex-shrink: 0; margin-right: 6px;
        }
        .video-bar .status-dot.on { background: #4CAF50; }
        .video-bar .label { color: #aaa; font-size: 11px; }
        .video-bar .label.active { color: #4CAF50; }
        .video-bar .remote-badge {
            background: #d4edda; color: #155724;
            padding: 3px 10px; border-radius: 10px;
            font-size: 10px; font-weight: 600; white-space: nowrap;
            margin-left: 8px;
        }
        .bar-btn {
            background: rgba(255,255,255,0.1);
            color: #ccc; border: 1px solid rgba(255,255,255,0.15);
            border-radius: 8px; padding: 4px 10px; font-size: 12px;
            cursor: pointer; white-space: nowrap; margin-left: 4px;
        }
        .bar-btn:active { background: rgba(255,255,255,0.2); }

        /* 视频面板（开启时显示） */
        .video-panel {
            background: #1a1a2e; border-radius: 0 0 10px 10px;
            overflow: hidden; margin-bottom: 6px;
            display: none;
        }
        .video-panel.show { display: block; }
        .video-panel img { width: 100%; display: block; }

        /* 视频底部操作栏：设置 + 拍照 居中 */
        .video-actions {
            display: flex; align-items: center; justify-content: center;
            padding: 6px 8px; gap: 8px; background: #16213e;
        }
        .video-actions .act-btn {
            background: rgba(255,255,255,0.1);
            color: #ccc; border: 1px solid rgba(255,255,255,0.15);
            border-radius: 8px; padding: 5px 14px; font-size: 12px;
            cursor: pointer; transition: all 0.2s;
        }
        .video-actions .act-btn:active { background: rgba(255,255,255,0.2); }
        .video-actions .act-btn.photo {
            background: linear-gradient(135deg, #FF6B6B, #EE5A24);
            color: white; border: none;
        }

        /* 翻转选项（展开时显示） */
        .flip-options {
            display: none; padding: 6px 8px; gap: 8px;
            background: #16213e; justify-content: center;
        }
        .flip-options.show { display: flex; }
        .flip-options .flip-toggle {
            background: rgba(255,255,255,0.08);
            color: #888; border: 1px solid rgba(255,255,255,0.15);
            border-radius: 8px; padding: 5px 12px; font-size: 11px;
            cursor: pointer; transition: all 0.2s;
        }
        .flip-options .flip-toggle.active {
            background: rgba(76,175,80,0.25); color: #4CAF50; border-color: #4CAF50;
        }

        /* 底部按钮栏 */
        .bottom-bar {
            display: flex; align-items: center; justify-content: center;
            gap: 6px; margin: 6px 0;
        }
        .bottom-btn {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white; border: none; border-radius: 10px;
            padding: 10px 6px; font-size: 13px; cursor: pointer;
            font-weight: 600; text-align: center; text-decoration: none;
            transition: transform 0.15s;
        }
        .bottom-btn:active { transform: scale(0.95); }
        .bottom-btn.stop {
            background: linear-gradient(135deg, #DC3545, #C82333);
        }
        .bottom-btn.config {
            background: linear-gradient(135deg, #FF9800, #F57C00);
        }
        .bottom-btn.refresh {
            background: linear-gradient(135deg, #2196F3, #1976D2);
        }

        /* 拍照成功提示（浮动，不覆盖状态栏） */
        .toast {
            position: fixed; top: 20px; left: 50%; transform: translateX(-50%);
            background: rgba(76,175,80,0.95); color: white;
            padding: 8px 20px; border-radius: 20px; font-size: 13px;
            z-index: 9999; opacity: 0; transition: opacity 0.3s;
            pointer-events: none;
        }
        .toast.show { opacity: 1; }
        .toast.error { background: rgba(220,53,69,0.95); }

        /* 摇杆（居中显示） */
        .joystick-section {
            display: flex; flex-direction: column; align-items: center;
            margin: 4px 0 8px;
        }
        .joystick-wrap { margin: 0 auto; max-width: 280px; width: 70%; }
        .joystick-container {
            position: relative; width: 100%; aspect-ratio: 1;
            border-radius: 50%; background: #f0f0f0;
            border: 3px solid #ddd; touch-action: none;
        }
        .joystick {
            position: absolute; width: 28%; height: 28%;
            background: linear-gradient(135deg, #4CAF50, #45a049);
            border-radius: 50%; top: 50%; left: 50%;
            transform: translate(-50%, -50%);
            box-shadow: 0 4px 8px rgba(0,0,0,0.2);
            transition: all 0.1s ease; cursor: pointer;
        }
        .joystick.active {
            background: linear-gradient(135deg, #2196F3, #1976D2);
            transform: translate(-50%, -50%) scale(0.95);
        }
        .direction-indicator {
            position: absolute; top: 50%; left: 50%;
            transform: translate(-50%, -50%);
            font-size: 16px; font-weight: bold; color: #333;
            pointer-events: none; transition: opacity 0.3s;
        }
        .direction-indicator.active { opacity: 1; }

        /* 动作控制（可折叠，默认折叠） */
        .actions-section { margin-top: 4px; }
        .actions-header {
            display: flex; align-items: center; justify-content: center;
            padding: 8px; cursor: pointer; user-select: none;
            background: #f8f9fa; border-radius: 10px; margin-bottom: 4px;
        }
        .actions-header h3 {
            font-size: 0.9em; color: #333; margin: 0;
        }
        .actions-header .arrow {
            margin-left: 8px; font-size: 12px; color: #666;
            transition: transform 0.2s;
        }
        .actions-header .arrow.open { transform: rotate(180deg); }
        .actions-grid {
            display: none; grid-template-columns: repeat(3, 1fr); gap: 6px;
        }
        .actions-grid.show { display: grid; }
        .action-btn {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white; border: none; border-radius: 10px;
            padding: 10px 6px; font-size: 13px; cursor: pointer;
            transition: transform 0.15s;
        }
        .action-btn:active { transform: scale(0.95); }
        .action-group-title {
            grid-column: 1 / -1; font-size: 0.8em;
            color: #666; font-weight: 600; padding: 4px 0 2px;
            border-bottom: 1px solid #e9ecef; margin-bottom: 2px;
        }

        @media (max-width: 480px) {
            .container { padding: 8px 10px 12px; }
            .header h1 { font-size: 1em; }
            .joystick-wrap { max-width: 240px; }
            .action-btn { font-size: 12px; padding: 9px 5px; }
        }
    </style>
</head>
<body>
    <!-- 浮动提示（不覆盖状态栏） -->
    <div class="toast" id="toast"></div>
    <div class="container">
        <div class="header">
            <h1>小智小车遥控器</h1>
            <div class="subtitle">拖拽摇杆控制方向</div>
        </div>

        <!-- 视频控制栏 -->
        <div class="video-bar">
            <div class="left">
                <div class="status-dot" id="video-dot"></div>
                <div class="label" id="video-label">视频流未连接</div>
                <div class="remote-badge" id="remote-badge">遥控已连接</div>
            </div>
            <button class="bar-btn" id="btn-stream-toggle" onclick="toggleStream()">📷 开启</button>
        </div>

        <!-- 视频面板（开启时显示） -->
        <div class="video-panel" id="video-panel">
            <div id="stream-container"></div>
            <div class="video-actions">
                <button class="act-btn" id="btn-settings" onclick="toggleSettings()">⚙ 设置</button>
                <button class="act-btn photo" onclick="takePhoto()">📷 拍照</button>
            </div>
            <div class="flip-options" id="flip-options">
                <button class="flip-toggle" id="btn-h-mirror" onclick="toggleFlip('h_mirror')">↔ 水平</button>
                <button class="flip-toggle" id="btn-v-flip" onclick="toggleFlip('v_flip')">↕ 垂直</button>
            </div>
        </div>

        <!-- 摇杆（居中） -->
        <div class="joystick-section">
            <div class="joystick-wrap">
                <div class="joystick-container" id="joystick-container">
                    <div class="joystick" id="joystick"></div>
                    <div class="direction-indicator" id="direction-indicator">⏹</div>
                </div>
            </div>
        </div>

        <!-- 底部按钮：停止、配置 -->
        <div class="bottom-bar">
            <button class="bottom-btn stop" onclick="stopCar()">🛑 停止</button>
            <a href="/config" class="bottom-btn config">⚙ 配置</a>
        </div>

        <!-- 动作控制（可折叠，默认折叠） -->
        <div class="actions-section">
            <div class="actions-header" onclick="toggleActions()">
                <h3>🎭 动作控制</h3>
                <span class="arrow" id="actions-arrow">▼</span>
            </div>
            <div class="actions-grid" id="actions-grid">
                <div class="action-group-title">🚗 基本移动</div>
                <button class="action-btn" onclick="executeAction('move_forward')">⬆ 前进</button>
                <button class="action-btn" onclick="executeAction('move_backward')">⬇ 后退</button>
                <button class="action-btn" onclick="executeAction('turn_left')">⬅ 左转</button>
                <button class="action-btn" onclick="executeAction('turn_right')">➡ 右转</button>
                <button class="action-btn" onclick="executeAction('spin_around')">🔄 转圈</button>
                <div class="action-group-title">😊 情感表达</div>
                <button class="action-btn" onclick="executeAction('wake_up')">🌅 唤醒</button>
                <button class="action-btn" onclick="executeAction('happy')">😄 开心</button>
                <button class="action-btn" onclick="executeAction('sad')">😢 悲伤</button>
                <button class="action-btn" onclick="executeAction('thinking')">🤔 思考</button>
                <button class="action-btn" onclick="executeAction('listening')">👂 倾听</button>
                <button class="action-btn" onclick="executeAction('speaking')">💬 说话</button>
                <button class="action-btn" onclick="executeAction('wiggle')">🌊 摆动</button>
                <button class="action-btn" onclick="executeAction('dance')">💃 跳舞</button>
                <div class="action-group-title">🎭 高级情感</div>
                <button class="action-btn" onclick="executeAction('excited')">🤩 兴奋</button>
                <button class="action-btn" onclick="executeAction('loving')">😍 爱慕</button>
                <button class="action-btn" onclick="executeAction('angry')">😠 生气</button>
                <button class="action-btn" onclick="executeAction('surprised')">😲 惊讶</button>
                <button class="action-btn" onclick="executeAction('confused')">😕 困惑</button>
            </div>
        </div>
    </div>

    <script>
        // 摇杆变量
        let joystick = document.getElementById('joystick');
        let joystickContainer = document.getElementById('joystick-container');
        let directionIndicator = document.getElementById('direction-indicator');
        let remoteBadge = document.getElementById('remote-badge');
        let isDragging = false;
        let currentDirection = 0;
        let currentSpeed = 0;
        let isRequestPending = false;

        // 视频流变量
        let streamState = 'idle'; // idle | opening | open | closing
        let settingsOpen = false;
        let actionsOpen = false;
        const videoDot = document.getElementById('video-dot');
        const videoLabel = document.getElementById('video-label');
        const videoPanel = document.getElementById('video-panel');
        const btnStreamToggle = document.getElementById('btn-stream-toggle');
        const flipOptions = document.getElementById('flip-options');
        const actionsGrid = document.getElementById('actions-grid');
        const actionsArrow = document.getElementById('actions-arrow');
        const streamContainer = document.getElementById('stream-container');
        let streamImg = null; // 动态创建的 img 元素
        let streamLoadTimer = null;

        // 浮动提示
        let toastTimer = null;
        function showToast(msg, isError) {
            const t = document.getElementById('toast');
            t.textContent = msg;
            t.className = 'toast show' + (isError ? ' error' : '');
            if (toastTimer) clearTimeout(toastTimer);
            toastTimer = setTimeout(() => { t.className = 'toast'; }, 2000);
        }

        // 摇杆事件
        function initJoystick() {}

        function updateJoystickPosition(x, y) {
            const rect = joystickContainer.getBoundingClientRect();
            const cx = rect.left + rect.width / 2;
            const cy = rect.top + rect.height / 2;
            let rx = x - cx;
            let ry = y - cy;
            const maxR = rect.width / 2 - 30;
            const dist = Math.sqrt(rx * rx + ry * ry);
            if (dist > maxR) { rx = (rx / dist) * maxR; ry = (ry / dist) * maxR; }
            joystick.style.left = `calc(50% + ${rx}px)`;
            joystick.style.top = `calc(50% + ${ry}px)`;
            const nx = rx / maxR, ny = ry / maxR;
            let angle = Math.atan2(ny, nx) * (180 / Math.PI);
            if (angle < 0) angle += 360;
            const speed = Math.min(dist / maxR, 1) * 100;
            let dir = 0;
            if (speed > 5) {
                if (angle >= 315 || angle < 45) dir = 1;
                else if (angle >= 45 && angle < 135) dir = 2;
                else if (angle >= 135 && angle < 225) dir = 3;
                else dir = 4;
            }
            return { direction: dir, speed: Math.round(speed) };
        }

        function updateDirectionIndicator(dir, speed) {
            let icon = '⏹';
            if (speed > 5) {
                switch(dir) {
                    case 1: icon = '➡'; break;
                    case 2: icon = '⬇'; break;
                    case 3: icon = '⬅'; break;
                    case 4: icon = '⬆'; break;
                }
            }
            directionIndicator.textContent = icon;
            directionIndicator.classList.toggle('active', speed > 5);
        }

        async function sendControl(dir, speed) {
            if (dir === 0 && speed === 0) {
                currentDirection = 0; currentSpeed = 0;
                try {
                    const r = await fetch('/api/control', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ direction: 0, speed: 0 })
                    });
                    if (!r.ok) throw new Error();
                } catch (e) {
                    showToast('发送失败', true);
                }
                return;
            }
            // 后端有 dedup，前端不拦截（否则按住不动时后端收不到心跳）
            if (isRequestPending) return;
            currentDirection = dir; currentSpeed = speed;
            isRequestPending = true;
            try {
                const r = await fetch('/api/control', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ direction: dir, speed: speed })
                });
                if (!r.ok) throw new Error();
                remoteBadge.textContent = '遥控已连接';
                remoteBadge.style.background = '#d4edda';
                remoteBadge.style.color = '#155724';
            } catch (e) {
                remoteBadge.textContent = '连接断开';
                remoteBadge.style.background = '#f8d7da';
                remoteBadge.style.color = '#721c24';
                showToast('发送失败', true);
            } finally {
                isRequestPending = false;
            }
        }

        function stopCar() {
            isDragging = false;
            currentDirection = 0; currentSpeed = 0;
            joystick.style.left = '50%'; joystick.style.top = '50%';
            joystick.classList.remove('active');
            updateDirectionIndicator(0, 0);
            sendControl(0, 0);
        }

        async function executeAction(action) {
            try {
                const r = await fetch('/api/motor/action', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: action })
                });
                if (!r.ok) throw new Error();
                showToast('已执行', false);
                // 延迟后恢复默认表情
                setTimeout(async () => {
                    try {
                        await fetch('/api/emotion', {
                            method: 'POST',
                            headers: { 'Content-Type': 'application/json' },
                            body: JSON.stringify({ emotion: 'neutral' })
                        });
                    } catch (e) {}
                }, 3000);
            } catch (e) {
                showToast('执行失败', true);
            }
        }

        // 摇杆事件绑定
        joystickContainer.addEventListener('mousedown', (e) => {
            isDragging = true; joystick.classList.add('active');
            const { direction, speed } = updateJoystickPosition(e.clientX, e.clientY);
            updateDirectionIndicator(direction, speed);
            sendControl(direction, speed);
        });
        document.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const { direction, speed } = updateJoystickPosition(e.clientX, e.clientY);
                updateDirectionIndicator(direction, speed);
                sendControl(direction, speed);
            }
        });
        document.addEventListener('mouseup', () => { if (isDragging) stopCar(); });
        joystickContainer.addEventListener('touchstart', (e) => {
            e.preventDefault(); isDragging = true; joystick.classList.add('active');
            const t = e.touches[0];
            const { direction, speed } = updateJoystickPosition(t.clientX, t.clientY);
            updateDirectionIndicator(direction, speed);
            sendControl(direction, speed);
        });
        joystickContainer.addEventListener('touchmove', (e) => {
            e.preventDefault();
            if (isDragging) {
                const t = e.touches[0];
                const { direction, speed } = updateJoystickPosition(t.clientX, t.clientY);
                updateDirectionIndicator(direction, speed);
                sendControl(direction, speed);
            }
        });
        joystickContainer.addEventListener('touchend', (e) => {
            e.preventDefault(); if (isDragging) stopCar();
        });
        document.addEventListener('touchend', (e) => {
            if (isDragging && e.target !== joystick && e.target !== joystickContainer) stopCar();
        });
        setInterval(() => { if (isDragging) sendControl(currentDirection, currentSpeed); }, 200);
        window.addEventListener('resize', initJoystick);

        // 视频流控制 - 使用 MJPEG 长连接，服务端限帧 10fps
        let maxStreamRetries = 1; // 首次连接失败后仅重试1次
        function startStream() {
            if (streamState !== 'idle') return;
            streamState = 'opening';

            // 销毁旧 img
            if (streamImg) {
                streamImg.onload = null;
                streamImg.onerror = null;
                streamImg.src = '';
                streamImg.remove();
                streamImg = null;
            }
            streamContainer.innerHTML = '';

            // 创建 img 元素
            streamImg = document.createElement('img');
            streamImg.style.width = '100%';
            streamImg.style.display = 'block';
            streamContainer.appendChild(streamImg);

            // 立即展开视频面板
            videoPanel.classList.add('show');
            videoLabel.textContent = '加载中...';
            videoLabel.classList.add('active');
            videoDot.classList.remove('on');
            btnStreamToggle.textContent = '📷 关闭';
            settingsOpen = false;
            flipOptions.classList.remove('show');

            // 先绑定事件，再设置 src（避免 race condition）
            var streamUrl = window.location.protocol + '//' + window.location.hostname + ':81/stream?' + Date.now() + '_' + Math.random();
            console.log('[Stream] Connecting to:', streamUrl);
            streamImg.onload = function() {
                console.log('[Stream] onload - connected');
                videoDot.classList.add('on');
                videoLabel.textContent = '视频流已连接';
                streamState = 'open';
                if (streamLoadTimer) clearTimeout(streamLoadTimer);
                loadFlipStates();
            };
            streamImg.onerror = function() {
                console.log('[Stream] onerror - failed, retries left:', maxStreamRetries);
                videoDot.classList.remove('on');
                videoLabel.classList.remove('active');
                if (streamLoadTimer) clearTimeout(streamLoadTimer);
                if (maxStreamRetries > 0) {
                    maxStreamRetries--;
                    streamState = 'idle';
                    videoLabel.textContent = '连接中... (重试)';
                    setTimeout(function() {
                        if (streamState === 'idle') startStream();
                    }, 800);
                } else {
                    videoLabel.textContent = '视频流不可用';
                    streamState = 'idle';
                    btnStreamToggle.textContent = '📷 开启';
                }
            };
            // 直接设置真实流 URL（移除透明 GIF 中间步骤）
            streamImg.src = streamUrl;

            // 超时检测（15秒，给首次连接更多时间）
            streamLoadTimer = setTimeout(() => {
                if (streamState === 'opening') {
                    console.log('[Stream] Timeout after 15s');
                    videoLabel.textContent = '视频流不可用';
                    videoLabel.classList.remove('active');
                    videoDot.classList.remove('on');
                    streamState = 'idle';
                    btnStreamToggle.textContent = '📷 开启';
                }
            }, 15000);
        }

        function stopStream() {
            if (streamState !== 'open' && streamState !== 'opening') return;
            streamState = 'closing';
            if (streamLoadTimer) clearTimeout(streamLoadTimer);

            videoLabel.textContent = '正在关闭...';
            videoLabel.classList.add('active');
            videoDot.classList.remove('on');
            btnStreamToggle.textContent = '📷 关闭中';

            // 关闭 img src 断开连接，等待 500ms 后刷新页面确保连接彻底清理
            if (streamImg) {
                streamImg.src = '';
            }
            setTimeout(() => {
                window.location.reload();
            }, 500);
        }

        function toggleStream() {
            streamState === 'idle' ? startStream() : stopStream();
        }

        // 设置面板（展开/折叠翻转选项）
        function toggleSettings() {
            settingsOpen = !settingsOpen;
            flipOptions.classList.toggle('show', settingsOpen);
        }

        // 动作控制折叠/展开
        function toggleActions() {
            actionsOpen = !actionsOpen;
            actionsGrid.classList.toggle('show', actionsOpen);
            actionsArrow.classList.toggle('open', actionsOpen);
        }

        // 翻转控制（独立状态）
        async function loadFlipStates() {
            try {
                const resp = await fetch('/api/camera/flip');
                if (!resp.ok) return;
                const data = await resp.json();
                document.getElementById('btn-h-mirror').classList.toggle('active', data.h_mirror);
                document.getElementById('btn-h-mirror').dataset.state = data.h_mirror ? '1' : '0';
                document.getElementById('btn-v-flip').classList.toggle('active', data.v_flip);
                document.getElementById('btn-v-flip').dataset.state = data.v_flip ? '1' : '0';
            } catch (e) { console.log('Flip load error:', e); }
        }

        async function toggleFlip(key) {
            const hBtn = document.getElementById('btn-h-mirror');
            const vBtn = document.getElementById('btn-v-flip');
            const hCur = hBtn.dataset.state === '1';
            const vCur = vBtn.dataset.state === '1';
            const hNew = key === 'h_mirror' ? !hCur : hCur;
            const vNew = key === 'v_flip' ? !vCur : vCur;
            try {
                const resp = await fetch('/api/camera/flip', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ h_mirror: hNew, v_flip: vNew })
                });
                if (!resp.ok) return;
                const data = await resp.json();
                hBtn.classList.toggle('active', data.h_mirror);
                hBtn.dataset.state = data.h_mirror ? '1' : '0';
                vBtn.classList.toggle('active', data.v_flip);
                vBtn.dataset.state = data.v_flip ? '1' : '0';
            } catch (e) { console.error('Flip error:', e); }
        }

        // 拍照（浮动提示，不覆盖状态栏）
        async function takePhoto() {
            try {
                const resp = await fetch('/api/camera/photo?' + Date.now());
                if (!resp.ok) throw new Error();
                const blob = await resp.blob();
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = 'photo_' + Date.now() + '.jpg';
                a.click();
                URL.revokeObjectURL(url);
                showToast('拍照成功', false);
            } catch (e) {
                showToast('拍照失败', true);
            }
        }
    </script>
</body>
</html>
)html";

    return html_page;
}


// 解析JSON控制命令
void WebServer::parse_json_control_command(const char* data, int& direction, int& speed) {
    cJSON *json = cJSON_Parse(data);
    if (json == NULL) {
        direction = 0;
        speed = 0;
        return;
    }

    cJSON *j_direction = cJSON_GetObjectItem(json, "direction");
    cJSON *j_speed = cJSON_GetObjectItem(json, "speed");

    if (cJSON_IsNumber(j_direction) && cJSON_IsNumber(j_speed)) {
        direction = j_direction->valueint;
        speed = j_speed->valueint;
    } else {
        direction = 0;
        speed = 0;
    }

    cJSON_Delete(json);
}

// 配置页面处理器
esp_err_t WebServer::config_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, server->get_config_html_page(), HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// 配置页面POST处理器
esp_err_t WebServer::config_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    // 解析表单数据
    MotorActionConfig config;
    server->parse_config_form_data(content, config);

    // 保存配置
    if (server->set_motor_config_callback_) {
        server->set_motor_config_callback_(config);
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, R"html(<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>配置已保存</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }

        .container {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
            text-align: center;
            max-width: 500px;
            width: 100%;
        }

        .success-icon {
            font-size: 4em;
            margin-bottom: 20px;
        }

        h1 {
            color: #333;
            margin-bottom: 10px;
            font-size: 2.2em;
        }

        .message {
            color: #666;
            margin-bottom: 30px;
            font-size: 1.1em;
        }

        .buttons {
            display: flex;
            gap: 15px;
            justify-content: center;
            flex-wrap: wrap;
        }

        .btn {
            background: linear-gradient(135deg, #4CAF50, #45a049);
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 25px;
            font-size: 16px;
            cursor: pointer;
            text-decoration: none;
            display: inline-block;
            transition: all 0.3s ease;
            box-shadow: 0 4px 15px rgba(76, 175, 80, 0.3);
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(76, 175, 80, 0.4);
        }

        .btn.secondary {
            background: linear-gradient(135deg, #2196F3, #1976D2);
        }

        .btn.secondary:hover {
            box-shadow: 0 6px 20px rgba(33, 150, 243, 0.4);
        }

        @media (max-width: 480px) {
            .container {
                padding: 30px 20px;
            }

            .buttons {
                flex-direction: column;
                align-items: center;
            }

            .btn {
                width: 200px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="success-icon">✅</div>
        <h1>配置已保存！</h1>
        <div class="message">您的电机动作配置已成功保存到设备中。</div>

        <div class="buttons">
            <a href="/config" class="btn secondary">⚙️ 返回配置页面</a>
            <a href="/" class="btn">🏠 返回遥控器</a>
        </div>
    </div>
</body>
</html>)html", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Configuration callback not set");
    }

    return ESP_OK;
}

// API配置GET处理器
esp_err_t WebServer::api_config_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (server->get_motor_config_callback_) {
        MotorActionConfig config = server->get_motor_config_callback_();

        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "forward_ms", config.forward_duration_ms);
        cJSON_AddNumberToObject(root, "backward_ms", config.backward_duration_ms);
        cJSON_AddNumberToObject(root, "left_turn_ms", config.left_turn_duration_ms);
        cJSON_AddNumberToObject(root, "right_turn_ms", config.right_turn_duration_ms);
        cJSON_AddNumberToObject(root, "spin_ms", config.spin_duration_ms);
        cJSON_AddNumberToObject(root, "quick_fwd_ms", config.quick_forward_duration_ms);
        cJSON_AddNumberToObject(root, "quick_bwd_ms", config.quick_backward_duration_ms);
        cJSON_AddNumberToObject(root, "def_speed_pct", config.default_speed_percent);

        char *json_str = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
            cJSON_free(json_str);
        } else {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate JSON");
        }
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Configuration callback not set");
    }

    return ESP_OK;
}

// API配置POST处理器
esp_err_t WebServer::api_config_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    // 解析JSON数据
    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    MotorActionConfig config;
    config.forward_duration_ms = cJSON_GetObjectItem(root, "forward_ms")->valueint;
    config.backward_duration_ms = cJSON_GetObjectItem(root, "backward_ms")->valueint;
    config.left_turn_duration_ms = cJSON_GetObjectItem(root, "left_turn_ms")->valueint;
    config.right_turn_duration_ms = cJSON_GetObjectItem(root, "right_turn_ms")->valueint;
    config.spin_duration_ms = cJSON_GetObjectItem(root, "spin_ms")->valueint;
    config.quick_forward_duration_ms = cJSON_GetObjectItem(root, "quick_fwd_ms")->valueint;
    config.quick_backward_duration_ms = cJSON_GetObjectItem(root, "quick_bwd_ms")->valueint;
    config.default_speed_percent = cJSON_GetObjectItem(root, "def_speed_pct")->valueint;

    cJSON_Delete(root);

    // 保存配置
    if (server->set_motor_config_callback_) {
        server->set_motor_config_callback_(config);

        // 在串口中输出保存的配置信息
        ESP_LOGI(TAG, "网页配置已保存:");
        ESP_LOGI(TAG, "  前进时间: %d ms", config.forward_duration_ms);
        ESP_LOGI(TAG, "  后退时间: %d ms", config.backward_duration_ms);
        ESP_LOGI(TAG, "  左转时间: %d ms", config.left_turn_duration_ms);
        ESP_LOGI(TAG, "  右转时间: %d ms", config.right_turn_duration_ms);
        ESP_LOGI(TAG, "  转圈时间: %d ms", config.spin_duration_ms);
        ESP_LOGI(TAG, "  快速前进时间: %d ms", config.quick_forward_duration_ms);
        ESP_LOGI(TAG, "  快速后退时间: %d ms", config.quick_backward_duration_ms);
        ESP_LOGI(TAG, "  默认速度: %d%%", config.default_speed_percent);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"status\":\"success\"}", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Configuration callback not set");
    }

    return ESP_OK;
}

// 解析配置表单数据
void WebServer::parse_config_form_data(const char* data, MotorActionConfig& config) {
    char* str = strdup(data);
    char* token = strtok(str, "&");

    while (token != NULL) {
        char* eq_pos = strchr(token, '=');
        if (eq_pos) {
            *eq_pos = '\0';
            char* key = token;
            char* value = eq_pos + 1;

            if (strcmp(key, "forward_ms") == 0) {
                config.forward_duration_ms = atoi(value);
            } else if (strcmp(key, "backward_ms") == 0) {
                config.backward_duration_ms = atoi(value);
            } else if (strcmp(key, "left_turn_ms") == 0) {
                config.left_turn_duration_ms = atoi(value);
            } else if (strcmp(key, "right_turn_ms") == 0) {
                config.right_turn_duration_ms = atoi(value);
            } else if (strcmp(key, "spin_ms") == 0) {
                config.spin_duration_ms = atoi(value);
            } else if (strcmp(key, "quick_fwd_ms") == 0) {
                config.quick_forward_duration_ms = atoi(value);
            } else if (strcmp(key, "quick_bwd_ms") == 0) {
                config.quick_backward_duration_ms = atoi(value);
            } else if (strcmp(key, "def_speed_pct") == 0) {
                config.default_speed_percent = atoi(value);
            }
        }
        token = strtok(NULL, "&");
    }

    free(str);
}

// 获取配置页面HTML
const char* WebServer::get_config_html_page() {
    static const char config_html_page[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>电机动作配置</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }

        .container {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
            max-width: 600px;
            width: 100%;
        }

        h1 {
            color: #333;
            text-align: center;
            margin-bottom: 30px;
        }

        .form-group {
            margin-bottom: 20px;
        }

        label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
            color: #555;
        }

        input[type="number"] {
            width: 100%;
            padding: 10px;
            border: 2px solid #ddd;
            border-radius: 8px;
            font-size: 16px;
            transition: border-color 0.3s ease;
        }

        input[type="number"]:focus {
            outline: none;
            border-color: #4CAF50;
        }

        .unit {
            color: #666;
            font-size: 14px;
            margin-left: 5px;
        }

        .description {
            color: #777;
            font-size: 14px;
            margin-top: 3px;
            font-weight: normal;
        }

        .buttons {
            text-align: center;
            margin-top: 30px;
        }

        .btn {
            background: linear-gradient(135deg, #4CAF50, #45a049);
            color: white;
            border: none;
            padding: 12px 30px;
            border-radius: 25px;
            font-size: 16px;
            cursor: pointer;
            margin: 0 10px;
            text-decoration: none;
            display: inline-block;
            transition: all 0.3s ease;
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(76, 175, 80, 0.4);
        }

        .btn.secondary {
            background: linear-gradient(135deg, #2196F3, #1976D2);
        }

        .btn.secondary:hover {
            box-shadow: 0 6px 20px rgba(33, 150, 243, 0.4);
        }

        .grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }

        @media (max-width: 480px) {
            .grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>⚙️ 电机动作配置</h1>
        <form method="POST" action="/config">
            <div class="grid">
                <div class="form-group">
                    <label for="forward_ms">前进时间</label>
                    <input type="number" id="forward_ms" name="forward_ms" min="100" max="30000" step="100" required>
                    <span class="unit">毫秒</span>
                    <div class="description">默认前进动作的持续时间</div>
                </div>

                <div class="form-group">
                    <label for="backward_ms">后退时间</label>
                    <input type="number" id="backward_ms" name="backward_ms" min="100" max="30000" step="100" required>
                    <span class="unit">毫秒</span>
                    <div class="description">默认后退动作的持续时间</div>
                </div>

                <div class="form-group">
                    <label for="left_turn_ms">左转时间</label>
                    <input type="number" id="left_turn_ms" name="left_turn_ms" min="100" max="10000" step="50" required>
                    <span class="unit">毫秒</span>
                    <div class="description">左转动作的持续时间</div>
                </div>

                <div class="form-group">
                    <label for="right_turn_ms">右转时间</label>
                    <input type="number" id="right_turn_ms" name="right_turn_ms" min="100" max="10000" step="50" required>
                    <span class="unit">毫秒</span>
                    <div class="description">右转动作的持续时间</div>
                </div>

                <div class="form-group">
                    <label for="spin_ms">转圈时间</label>
                    <input type="number" id="spin_ms" name="spin_ms" min="500" max="10000" step="100" required>
                    <span class="unit">毫秒</span>
                    <div class="description">转圈动作的持续时间</div>
                </div>


                <div class="form-group">
                    <label for="def_speed_pct">默认速度</label>
                    <input type="number" id="def_speed_pct" name="def_speed_pct" min="10" max="100" step="5" required>
                    <span class="unit">%</span>
                    <div class="description">电机动作的默认速度百分比</div>
                </div>
            </div>

            <div class="buttons">
                <button type="submit" class="btn">💾 保存配置</button>
                <a href="/" class="btn secondary">🏠 返回遥控器</a>
            </div>
        </form>
    </div>

    <script>
        // 页面加载时获取当前配置
        window.onload = function() {
            fetch('/api/config')
                .then(response => response.json())
                .then(config => {
                    document.getElementById('forward_ms').value = config.forward_ms;
                    document.getElementById('backward_ms').value = config.backward_ms;
                    document.getElementById('left_turn_ms').value = config.left_turn_ms;
                    document.getElementById('right_turn_ms').value = config.right_turn_ms;
                    document.getElementById('spin_ms').value = config.spin_ms;
                    document.getElementById('def_speed_pct').value = config.def_speed_pct;
                })
                .catch(error => console.error('Failed to load config:', error));
        };

        // 处理表单提交
        document.getElementById('config-form').addEventListener('submit', function(e) {
            e.preventDefault(); // 阻止默认表单提交

            const formData = new FormData(this);
            const data = Object.fromEntries(formData.entries());

            fetch('/api/config', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(data)
            })
            .then(response => response.json())
            .then(result => {
                if (result.status === 'success') {
                    // 显示成功消息
                    alert('配置保存成功！');
                    // 自动跳转回遥控器界面
                    window.location.href = '/';
                } else {
                    alert('配置保存失败，请重试');
                }
            })
            .catch(error => {
                console.error('Failed to save config:', error);
                alert('配置保存失败，请检查网络连接');
            });
        });
    </script>
</body>
</html>
)html";

    return config_html_page;
}