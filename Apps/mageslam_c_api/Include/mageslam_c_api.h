#pragma once

extern "C"
{
    void mageslam_initialize();
    void mageslam_uninitialize();

    void mageslam_process_frame(int width, int height, void* data);

    extern void mageslam_frame_processed_callback(
        bool isPoseGood,
        bool isPoseSkipped,
        int trackingState,
        float m00,
        float m01,
        float m02,
        float m03,
        float m10,
        float m11,
        float m12,
        float m13,
        float m20,
        float m21,
        float m22,
        float m23,
        float m30,
        float m31,
        float m32,
        float m33);
}
