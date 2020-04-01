#pragma once

namespace mage
{
    namespace platform
    {
        void set_thread_name(const char* name);
        void profile_memory();

        void wait_for_debugger();
    }
}
