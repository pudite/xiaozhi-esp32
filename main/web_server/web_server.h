#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <esp_http_server.h>
#include <string>
#include <functional>
#include <vector>

// Forward declaration
class Esp32Camera;

class WebServer {
public:
    WebServer();
    ~WebServer();

    bool Start(int port = 80);
    void Stop();

    // 注册电机控制回调函数 - 使用 std::function 支持 lambda
    void SetMotorControlCallback(std::function<void(int direction, int speed)> callback);
    // 由外部调用以触发电机控制（封装私有回调）
    void InvokeMotorControl(int direction, int speed);

    // 注册 WebSocket 断开清理回调
    void SetWebControlStateClearCallback(std::function<void()> callback);
    // 由 WebSocket handler 调用以清理 web 控制状态
    void InvokeWebControlStateClear();

    // 注册表情设置回调函数
    void SetEmotionCallback(std::function<void(const char* emotion)> callback);
    // 设置表情（封装私有回调）
    void SetEmotion(const char* emotion);

    // 注册摄像头获取回调
    void SetCameraCallback(std::function<Esp32Camera*()> callback);

    // 注册摄像头翻转回调
    void SetCameraFlipCallback(std::function<std::pair<bool, bool>()> get_callback,
                               std::function<void(bool, bool)> set_callback);

    // 注册摄像头 JPEG 质量设置回调（quality: 1-63，OV2640 值越小质量越高）
    void SetCameraQualityCallback(std::function<bool(int)> callback);

    // 注册电源管理回调（流媒体期间强制高性能模式）
    void SetPowerSaveCallback(std::function<void(bool performance)> callback);

    // 获取流媒体服务器端口（用于前端引用）
    int GetStreamPort() const { return stream_port_; }

    // 拍照相关
    void SetCameraSnapshotCallback(std::function<bool()> callback);

    // 电机动作配置回调
    struct MotorActionConfig {
        int forward_duration_ms = 5000;
        int backward_duration_ms = 5000;
        int left_turn_duration_ms = 600;
        int right_turn_duration_ms = 600;
        int spin_duration_ms = 2500;
        int wiggle_duration_ms = 600;
        int dance_duration_ms = 1500;
        int quick_forward_duration_ms = 5000;
        int quick_backward_duration_ms = 5000;
        int default_speed_percent = 100;
    };

    void SetMotorActionConfigCallback(std::function<MotorActionConfig()> get_callback,
                                     std::function<void(const MotorActionConfig&)> set_callback);

    // Debug handler 注册（/api/debug/motor_test）
    static esp_err_t debug_motor_test_handler(httpd_req_t *req);

    // Emotion API handler
    static esp_err_t api_emotion_post_handler(httpd_req_t *req);

private:
    httpd_handle_t server_handle_;
    httpd_handle_t stream_server_handle_;
    int stream_port_;
    std::function<void(int direction, int speed)> motor_control_callback_;
    std::function<void()> web_control_state_clear_callback_;
    std::function<void(const char* emotion)> emotion_callback_;
    std::function<Esp32Camera*()> camera_callback_;
    std::function<std::pair<bool, bool>()> get_camera_flip_callback_;
    std::function<void(bool, bool)> set_camera_flip_callback_;
    std::function<bool()> camera_snapshot_callback_;
    std::function<bool(int)> camera_quality_callback_;
    std::function<void(bool performance)> power_save_callback_;
    std::function<MotorActionConfig()> get_motor_config_callback_;
    std::function<void(const MotorActionConfig&)> set_motor_config_callback_;

    // 启动流媒体服务器
    bool start_stream_server();

    // HTTP请求处理函数
    static esp_err_t index_get_handler(httpd_req_t *req);
    static esp_err_t control_post_handler(httpd_req_t *req);
    static esp_err_t api_control_handler(httpd_req_t *req);
    static esp_err_t api_motor_action_handler(httpd_req_t *req);
    static esp_err_t stream_get_handler(httpd_req_t *req);
    static esp_err_t config_get_handler(httpd_req_t *req);
    static esp_err_t config_post_handler(httpd_req_t *req);
    static esp_err_t api_config_get_handler(httpd_req_t *req);
    static esp_err_t api_config_post_handler(httpd_req_t *req);
    static esp_err_t api_camera_flip_get_handler(httpd_req_t *req);
    static esp_err_t api_camera_flip_post_handler(httpd_req_t *req);
    static esp_err_t api_camera_photo_get_handler(httpd_req_t *req);
    static esp_err_t api_camera_quality_post_handler(httpd_req_t *req);

    // WebSocket 电机控制
    static esp_err_t ws_control_handler(httpd_req_t *req);

    // CORS处理
    static esp_err_t cors_handler(httpd_req_t *req);

    // 获取HTML页面内容
    static const char* get_html_page();

    // 解析控制命令
    void parse_simple_control_command(const char* data, int& direction, int& speed);
    void parse_json_control_command(const char* data, int& direction, int& speed);

    // 配置相关方法
    void parse_config_form_data(const char* data, MotorActionConfig& config);
    static const char* get_config_html_page();
};

#endif // WEB_SERVER_H