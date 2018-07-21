//
// Created by ernest on 18-7-19.
//

#include "JdeRobotOutputIO.h"

namespace dso {

    class FrameHessian;

    class CalibHessian;

    class FrameShell;


    namespace IOWrap {

        JdeRobotOutputIO::JdeRobotOutputIO(std::string filename) : target_filename(filename) {

        }

        void JdeRobotOutputIO::publishGraph(
                const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
                        Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) {

        }

        void JdeRobotOutputIO::publishKeyframes(std::vector<dso::FrameHessian *> &frames, bool final,
                                                dso::CalibHessian *HCalib) {


            boost::unique_lock<boost::mutex> lk(model3DMutex);
            for(FrameHessian* fh : frames)
            {
                if(keyframesByKFID.find(fh->frameID) == keyframesByKFID.end())
                {
                    KeyFrameDisplay* kfd = new KeyFrameDisplay();
                    keyframesByKFID[fh->frameID] = kfd;
                    keyframes.push_back(kfd);
                }
                keyframesByKFID[fh->frameID]->setFromKF(fh, HCalib);
            }

        }

        void JdeRobotOutputIO::publishCamPose(FrameShell *frame, CalibHessian *HCalib) {

        }

        void JdeRobotOutputIO::pushLiveFrame(FrameHessian *image) {

        }

        void JdeRobotOutputIO::pushDepthImage(MinimalImageB3 *image) {

        }

        bool JdeRobotOutputIO::needPushDepthImage() {

        }

        void JdeRobotOutputIO::reset() {
            throw std::runtime_error("Not implemented yet");
        }

        void JdeRobotOutputIO::join() {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (auto ph : keyframes){
                // apply threshold
                ph->refreshPC(true, 0.001, 0.001, 1, 0.1, 1);

                cloud = cloud + ph->getPC();
            }

            std::cerr << "number of keyframes : "<< keyframes.size() << std::endl;
            std::cerr << "number of points : "<< cloud.points.size() << std::endl;


            pcl::io::savePCDFileASCII(target_filename,cloud);
            std::cout << "saving pointcloud to " << target_filename << "." << std::endl;
        }
    }
}