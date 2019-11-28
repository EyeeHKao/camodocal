#ifndef CAMODOCALIBRATION_H
#define CAMODOCALIBRATION_H

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>

namespace camodocal
{
/*@brief
    odometer-camera 外参校正类，主要参考了这篇文献An analytical least-squares solution to the odometer-camera extrinsic calibration problem
CX Guo ，FM Mirzaei ，SI Roumeliotis  - 10.1109/ICRA.2012.6225339
*/
   
class CamOdoCalibration
{
public:
    CamOdoCalibration();
    
    //添加相对应的里程计估计的相对位姿和相机估计的相对位姿态数据，这里的数据最好要做时间同步处理
    bool addMotionSegment(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_cam,
                          const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_odo);

    void getCurrentCameraMotion(Eigen::Vector3d& rotation, Eigen::Vector3d& translation) const;

    bool motionsEnough(void) const;
    size_t getCurrentMotionCount(void) const;
    size_t getMotionCount(void) const;
    void setMotionCount(size_t count);

    //外参校正/标定
    bool calibrate(Eigen::Matrix4d& H_cam_odo);

    //读盘或写盘
    bool readMotionSegmentsFromFile(const std::string& filename);
    bool writeMotionSegmentsToFile(const std::string& filename) const;

    bool getVerbose(void);
    void setVerbose(bool on = false);
    
    //估计外参和尺度，
    bool estimate(const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2,
                  Eigen::Matrix4d& H_cam_odo,
                  std::vector<double>& scales) const;

private:
    //估计q_yz
    bool estimateRyx(const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                     const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                     const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                     const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2,
                     Eigen::Matrix3d& R_yx) const;
    //校正估计值
    void refineEstimate(Eigen::Matrix4d& H_cam_odo, std::vector<double>& scales,
                        const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                        const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                        const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                        const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2) const;

    // 求解一元二次方程：solve ax^2 + bx + c = 0
    bool solveQuadraticEquation(double a, double b, double c, double& x1, double& x2) const;

    typedef struct
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d rotation;
        Eigen::Vector3d translation;
    } Motion;   ///一次运动(相邻两时刻的相对位姿)数据结构

    typedef struct
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::vector<Motion, Eigen::aligned_allocator<Motion> > odoMotions;  ///多次里程计运动
        std::vector<Motion, Eigen::aligned_allocator<Motion> > camMotions;  ///多次相机运动
    } MotionSegment;    ///多次运动数据结构

    std::vector<MotionSegment> mSegments;

    size_t mMinMotions; ///标定所需最小运动次数

    bool mVerbose;
};

}

#endif
