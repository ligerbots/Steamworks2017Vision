#include <jni.h>
#include <opencv2/opencv.hpp>
#include <android/log.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <libavutil/log.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/channel_layout.h>
#include <libavutil/common.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>

#define TAG "AppNative"
#define FF_TAG "AppNative_ffmpeg"

void lb_av_log_callback(void *ptr, int level, const char *fmt, va_list vl) {
    int androidLevel;
    switch(level) {
        case AV_LOG_QUIET:
            androidLevel = ANDROID_LOG_SILENT;
            break;
        case AV_LOG_PANIC:
        case AV_LOG_FATAL:
            androidLevel = ANDROID_LOG_FATAL;
            break;
        case AV_LOG_ERROR:
            androidLevel = ANDROID_LOG_ERROR;
            break;
        case AV_LOG_WARNING:
            androidLevel = ANDROID_LOG_WARN;
            break;
        case AV_LOG_INFO:
            androidLevel = ANDROID_LOG_INFO;
            break;
        case AV_LOG_VERBOSE:
            androidLevel = ANDROID_LOG_VERBOSE;
            break;
        case AV_LOG_DEBUG:
            androidLevel = ANDROID_LOG_DEBUG;
            break;
        case AV_LOG_TRACE:
            androidLevel = ANDROID_LOG_SILENT;
            break;
        default:
            androidLevel = ANDROID_LOG_DEFAULT;
    }
    // we don't really care. I don't think ffmpeg wants to execute any format string vulnerabilities
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    __android_log_vprint(androidLevel, FF_TAG, fmt, vl);
#pragma GCC diagnostic pop
}

AVCodecID codec_id = AV_CODEC_ID_H264;
AVCodec *codec = NULL;
AVCodecContext *c = NULL;
AVFormatContext* avfctx = NULL;
AVFrame *frame = NULL;
int framePts = 0;
size_t frameLength = 0;

bool streamH264Init(std::string ip, int port) {
    int ret;
    codec = avcodec_find_encoder(codec_id);
    if (!codec) {
        av_log(NULL, AV_LOG_ERROR, "Codec not found");
        return false;
    }
    c = avcodec_alloc_context3(codec);
    if (!c) {
        av_log(NULL, AV_LOG_ERROR, "Could not allocate video codec context");
        return false;
    }
    /* put sample parameters */
    c->bit_rate = 2000000;
    /* resolution must be a multiple of two */
    c->width = 640;
    c->height = 480;
    /* frames per second */
    c->time_base.num = 1;
    c->time_base.den = 30;
    c->gop_size = 30;
    c->max_b_frames = 1;
    c->pix_fmt = AV_PIX_FMT_YUV420P;
    c->codec_type = AVMEDIA_TYPE_VIDEO;

    ret = av_opt_set(c->priv_data, "preset", "ultrafast", 0);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Could not set preset");
        return false;
    }
    ret = av_opt_set(c->priv_data, "tune", "zerolatency", 0);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Could not set tune");
        return false;
    }

    /* open it */
    if (avcodec_open2(c, codec, NULL) < 0) {
        av_log(NULL, AV_LOG_ERROR, "Could not open codec");
        return false;
    }

    frame = av_frame_alloc();
    if (!frame) {
        av_log(NULL, AV_LOG_ERROR, "Could not allocate frame");
        return false;
    }
    frame->format = c->pix_fmt;
    frame->width = c->width;
    frame->height = c->height;
    /* the image can be allocated by any means and av_image_alloc() is
    * just the most convenient way if av_malloc() is to be used */
    ret = av_image_alloc(frame->data, frame->linesize, c->width, c->height,
                         c->pix_fmt, 32);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Could not allocate raw picture buffer");
        return false;
    } else {
        frameLength = (size_t) ret;
    }

    AVOutputFormat* fmt = av_guess_format("rtp", NULL, NULL);
    if (!fmt) {
        av_log(NULL, AV_LOG_ERROR, "Could not create RTP format");
        return false;
    }

    std::ostringstream url;
    url << "rtp://";
    url << ip;
    url << ":";
    url << port;
    ret = avformat_alloc_output_context2(&avfctx, fmt, fmt->name, url.str().c_str());
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Could not allocate avf context");
        return false;
    }

    av_log(NULL, AV_LOG_INFO, "Writing to %s", avfctx->filename);

    if (avio_open(&avfctx->pb, avfctx->filename, AVIO_FLAG_WRITE) < 0) {
        av_log(NULL, AV_LOG_ERROR, "AVIO open failed");
        return false;
    }

    struct AVStream* stream = avformat_new_stream(avfctx, codec);
    ret = avcodec_parameters_from_context(stream->codecpar, c);

    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Stream parameters failed");
        return false;
    }

    char buf[200000];
    AVFormatContext *ac[] = { avfctx };
    av_sdp_create(ac, 1, buf, 20000);
    av_log(NULL, AV_LOG_INFO, "sdp:\n%s\n", buf);

    ret = avformat_write_header(avfctx, NULL);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Write header failed");
        return false;
    }

    return true;
}

bool streamH264Cleanup() {
    avcodec_close(c);
    av_free(c);
    av_freep(&frame->data[0]);
    av_frame_free(&frame);
    return true;
}

bool streamH264Send() {
    AVPacket pkt;
    pkt.data = NULL;
    pkt.size = 0;
    av_init_packet(&pkt);
    int ret = avcodec_receive_packet(c, &pkt);

    if (ret == AVERROR_EOF) {
        av_log(NULL, AV_LOG_INFO, "Stream EOF");
        return false;
    } else if (ret == AVERROR(EAGAIN)) {
        return false;
    } else if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Stream error: %d", ret);
        return true;
    } else {
        av_log(NULL, AV_LOG_DEBUG, "Write frame (size = %05d)", pkt.size);
        ret = avformat_write_header(avfctx, NULL);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Write header failed");
            return false;
        }
        ret = av_interleaved_write_frame(avfctx, &pkt);
        if(ret < 0) {
            av_log(NULL, AV_LOG_INFO, "Failed to write packet");
            return false;
        }
        av_packet_unref(&pkt);
    }

    return true;
}

bool streamH264EncodeFrame(cv::Mat* src) {
    int ret;

    if(src->rows * src->cols != (int) frameLength) {
        av_log(NULL, AV_LOG_ERROR, "Mat data is different length %d/%d", src->rows * src->cols, frameLength);
        return false;
    }

    memcpy(frame->data[0], src->data, frameLength);

    frame->pts = framePts++;
    /* encode the image */
    ret = avcodec_send_frame(c, frame);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Could not encode frame!");
        return false;
    }
    return true;
}

JNIEXPORT jlong JNICALL
Java_erik_android_vision_visiontest_1native_AppNative_copyNV21BufferToMat(JNIEnv *env, jclass type,
                                                                           jobject buffer, jint width, jint height) {
    uchar* source = (uchar*) env->GetDirectBufferAddress(buffer);

    cv::Mat* destMat = new cv::Mat(width, height, CV_8UC1, source);
    return (jlong) destMat;
}

JNIEXPORT void JNICALL
Java_erik_android_vision_visiontest_1native_AppNative_copyMatOfByteToCameraServerPacket(JNIEnv *env, jclass type,
                                                                          jobject buffer, jlong matAddr) {
    cv::Mat* source = (cv::Mat*) matAddr;
    uchar* dest = (uchar*) env->GetDirectBufferAddress(buffer);
    uchar* writePosition = &dest[8];
    memcpy(writePosition, source->data, (size_t) (source->rows * source->cols));
}

JNIEXPORT void JNICALL
Java_erik_android_vision_visiontest_1native_AppNative_ffmpegInit(JNIEnv *env, jclass type) {
    // avoid /dev/null redirection (yes Android is annoying)
    av_log_set_callback(lb_av_log_callback);
    avcodec_register_all();
    av_register_all();
    avformat_network_init();

    __android_log_print(ANDROID_LOG_INFO, TAG, "ffmpeg inited");
}

JNIEXPORT jboolean JNICALL
Java_erik_android_vision_visiontest_1native_AppNative_streamingInit(JNIEnv *env, jclass type,
                                                                    jstring ip_, jint port) {
    const char *ip = env->GetStringUTFChars(ip_, 0);

    bool result = streamH264Init(ip, port);

    env->ReleaseStringUTFChars(ip_, ip);

    return (jboolean) result;
}

JNIEXPORT jboolean JNICALL
Java_erik_android_vision_visiontest_1native_AppNative_streamingEncodeFrame(JNIEnv *env, jclass type,
                                                                           jlong matAddr) {
    cv::Mat* source = (cv::Mat*) matAddr;
    return (jboolean) streamH264EncodeFrame(source);
}

JNIEXPORT jboolean JNICALL
Java_erik_android_vision_visiontest_1native_AppNative_streamingSend(JNIEnv *env, jclass type) {
    return (jboolean) streamH264Send();
}

#ifdef __cplusplus
}
#endif

