#include <jni.h>
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

