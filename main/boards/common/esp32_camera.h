#pragma once
#include "sdkconfig.h"

#ifndef CONFIG_IDF_TARGET_ESP32
#include <lvgl.h>
#include <thread>
#include <memory>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "camera.h"
#include "jpg/image_to_jpeg.h"
#include "esp_video_init.h"
#include <mutex>

struct JpegChunk {
    uint8_t* data;
    size_t len;
};

class Esp32Camera : public Camera {
private:
    struct FrameBuffer {
        uint8_t *data = nullptr;
        size_t len = 0;
        uint16_t width = 0;
        uint16_t height = 0;
        v4l2_pix_fmt_t format = 0;
    } frame_;
    v4l2_pix_fmt_t sensor_format_ = 0;
#ifdef CONFIG_XIAOZHI_ENABLE_ROTATE_CAMERA_IMAGE
    uint16_t sensor_width_ = 0;
    uint16_t sensor_height_ = 0;
#endif  // CONFIG_XIAOZHI_ENABLE_ROTATE_CAMERA_IMAGE
    int video_fd_ = -1;
    bool streaming_on_ = false;
    bool h_mirror_ = false;
    bool v_flip_ = false;
    struct MmapBuffer { void *start = nullptr; size_t length = 0; };
    std::vector<MmapBuffer> mmap_buffers_;
    std::string explain_url_;
    std::string explain_token_;
    std::thread encoder_thread_;
    // 流媒体预分配缓冲区（避免每次 CaptureStreamFrame 都 malloc/free）
    uint8_t* stream_buf_ = nullptr;
    size_t stream_buf_size_ = 0;
    // 保护 frame_ 和 stream_buf_ 的多线程访问（httpd 并发请求、Explain 与 CaptureStreamFrame 并发）
    mutable std::mutex frame_mutex_;

public:
    Esp32Camera(const esp_video_init_config_t& config);
    ~Esp32Camera();

    virtual void SetExplainUrl(const std::string& url, const std::string& token);
    virtual bool Capture();
    // 翻转控制函数
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    bool IsHMirror() const { return h_mirror_; }
    bool IsVFlip() const { return v_flip_; }
    // JPEG 质量控制
    bool SetJpegQuality(int quality);
    virtual std::string Explain(const std::string& question);

    // 获取当前帧的 JPEG 数据（用于视频流）
    const uint8_t* GetFrameData() const { return frame_.data; }
    size_t GetFrameSize() const { return frame_.len; }
    uint16_t GetFrameWidth() const { return frame_.width; }
    uint16_t GetFrameHeight() const { return frame_.height; }

    // 轻量级流媒体帧捕获：直接从 mmap 缓冲区读取 JPEG 数据，
    // 跳过 LVGL 预览解码和 PSRAM 分配。仅适用于 JPEG 格式摄像头。
    bool CaptureStreamFrame();

    // 帧数据访问的锁保护（供 web_server 并发读取时使用）
    void LockFrame() const { frame_mutex_.lock(); }
    void UnlockFrame() const { frame_mutex_.unlock(); }
};

#endif // ndef CONFIG_IDF_TARGET_ESP32