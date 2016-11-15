#include <jni.h>
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jstring JNICALL
Java_erik_android_vision_visiontest_1native_AppNative_helloWorldJni(JNIEnv *env, jclass type) {
    cv::Mat m;
    return env->NewStringUTF("Hello, World!");
}

#ifdef __cplusplus
}
#endif

