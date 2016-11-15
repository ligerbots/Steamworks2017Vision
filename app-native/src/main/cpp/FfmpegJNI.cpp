#include <jni.h>

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
    return env->NewStringUTF("Hello world from JNI!");
}

#ifdef __cplusplus
}
#endif

