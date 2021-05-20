#pragma once

extern "C"
{
    void mageslam_initialize();
    void mageslam_uninitialize();

    void mageslam_process_frame();

    extern void mageslam_frame_processed_callback();
}
