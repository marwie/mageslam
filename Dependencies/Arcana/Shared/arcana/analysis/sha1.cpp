#include "sha1.h"

//#pragma warning (push)
//#pragma warning (disable:4245)
//#pragma warning (disable:4244)
#include <boost\uuid\sha1.hpp>
//#pragma warning (pop)

namespace mira
{
    using sha1_t = boost::uuids::detail::sha1;

    std::array<unsigned int, 5> sha1(const void* data, size_t size)
    {
        sha1_t computer;
        computer.process_bytes(data, size);

        std::array<unsigned int, 5> digest;
        computer.get_digest((sha1_t::digest_type)digest);
        return digest;
    }

    struct continuous_sha1::impl
    {
        sha1_t computer;
    };

    continuous_sha1::continuous_sha1()
        : m_impl{ std::make_unique<impl>() }
    {}

    continuous_sha1::~continuous_sha1() = default;
    continuous_sha1::continuous_sha1(continuous_sha1&&) = default;
    continuous_sha1& continuous_sha1::operator=(continuous_sha1&&) = default;

    void continuous_sha1::process(const void* data, size_t size)
    {
        m_impl->computer.process_bytes(data, size);
    }

    std::array<unsigned int, 5> continuous_sha1::get_digest() const
    {
        std::array<unsigned int, 5> digest;
        m_impl->computer.get_digest((sha1_t::digest_type)digest);
        return digest;
    }
}