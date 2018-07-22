//
// Created by ernest on 18-7-19.
//

#ifndef DSO_JDEROBOTOUTPUTIO_H
#define DSO_JDEROBOTOUTPUTIO_H

#include "../Output3DWrapper.h"
#include "../Pangolin/KeyFrameDisplay.h"
#include "../../FullSystem/HessianBlocks.h"

namespace dso {

    class FrameHessian;

    class CalibHessian;

    class FrameShell;


    namespace IOWrap {

        class JdeRobotOutputIO : public Output3DWrapper{
        public:
            JdeRobotOutputIO(std::string filename = "cloud.xyz");

            // ==================== Output3DWrapper Functionality ======================
            virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override;
            virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override;
            virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override;


            virtual void pushLiveFrame(FrameHessian* image) override;
            virtual void pushDepthImage(MinimalImageB3* image) override;
            virtual bool needPushDepthImage() override;

            virtual void join() override;

            virtual void reset() override;

        private:

            std::string target_filename;

            // 3D model rendering
            boost::mutex model3DMutex;
            std::vector<KeyFrameDisplay*> keyframes;
            std::vector<Vec3f,Eigen::aligned_allocator<Vec3f>> allFramePoses;
            std::map<int, KeyFrameDisplay*> keyframesByKFID;

            // camera
            CalibHessian Hcalib_;
        };

    }
}


#endif //DSO_JDEROBOTOUTPUTIO_H
