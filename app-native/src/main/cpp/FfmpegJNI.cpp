#include <jni.h>
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     org_ffmpeg_FfmpegJNI
 * Method:    helloWorldJNI
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_org_ffmpeg_FfmpegJNI_helloWorldJNI
        (JNIEnv * env, jclass type){
    cv::Mat m;
    return env->NewStringUTF("Hello world from JNI!");
}

#ifdef __cplusplus
}
#endif

