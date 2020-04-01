#include "DynamicIMUPosePriorProvider.h"

#include "Fuser/AnalogVFTFilter.h"
#include "Fuser/AnalogCASIMU.h"

#include <arcana/threading/dispatcher.h>
#include <arcana/mixin_ptr.h>

namespace mage
{
    struct DynamicIMUPosePriorProvider::Impl
    {
        mira::mixin_ptr<IIMUReceiver> Imu;

        std::unique_ptr<mage::AnalogVFTFilter> VftFilter;
        std::unique_ptr<mage::AnalogCASIMU> CasFilter;

        bool AssumeSamePosition{};

        mira::background_dispatcher<32> Dispatcher;

        Impl(const device::IMUCharacterization& characterization, bool assumeSamePosition)
            : VftFilter{ std::make_unique<AnalogVFTFilter>(characterization) }
            , AssumeSamePosition{ assumeSamePosition }
        {
            Imu = VftFilter.get();
        }
    };

    DynamicIMUPosePriorProvider::DynamicIMUPosePriorProvider(const device::IMUCharacterization& characterization, bool assumeSamePosition)
        : BaseWorker{0, 0}
        , m_impl{ new Impl{ characterization, assumeSamePosition } }
    {}

    DynamicIMUPosePriorProvider::~DynamicIMUPosePriorProvider() = default;

    void DynamicIMUPosePriorProvider::AddSample(const mage::SensorSample& sample)
    {
        Pending() += mira::make_task(m_impl->Dispatcher, Cancellation(), [this, sample]
        {
            m_impl->Imu.get<IIMUReceiver>()->AddSample(sample);
        });
    }

    void DynamicIMUPosePriorProvider::OnTrackingLost()
    {
        Pending() += mira::make_task(m_impl->Dispatcher, Cancellation(), [this]
        {
            if (m_impl->CasFilter)
            {
                m_impl->CasFilter->OnTrackingLost();
            }
        });
    }

    mira::task<Pose> DynamicIMUPosePriorProvider::GetPoseForTime(const TrackingFrameHistory& history, const std::chrono::system_clock::time_point& timestamp)
    {
        return Pending() += mira::make_task(m_impl->Dispatcher, Cancellation(), [this, history, timestamp]
        {
            const auto& keyframe = history.newest().Keyframe;
            auto keyframeTimestamp = keyframe->GetAnalyzedImage()->GetTimeStamp();

            if (m_impl->VftFilter)
            {
                m_impl->VftFilter->PredictUpTo(keyframeTimestamp);

                if (m_impl->VftFilter->IsConverged())
                {
                    m_impl->CasFilter = std::make_unique<AnalogCASIMU>(*m_impl->VftFilter, keyframe->GetPose(), keyframeTimestamp, m_impl->AssumeSamePosition);
                    m_impl->Imu = m_impl->CasFilter.get();
                    m_impl->VftFilter = nullptr;
                }

                // fallback to motion model
                return EstimateNextPoseFromHistory(history, timestamp);
            }
            else
            {
                m_impl->CasFilter->PredictUpTo(keyframeTimestamp);
                m_impl->CasFilter->AddPose(keyframe->GetPose(), keyframeTimestamp);

                m_impl->CasFilter->PredictUpTo(timestamp);

                return  m_impl->CasFilter->GetPose();
            }
        });
    }
}
