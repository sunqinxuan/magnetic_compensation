#include "mag_ins/impl/mag_kf_gins.h"
#include "kf-gins/insmech.h"
#include "common/rotation.h"

MAG_NAMESPACE_START

using pva_t = PVA;
using imu_t = IMU;

static mag_pose_t pva_to_mag_pose(const pva_t& pva)
{
    mag_pose_t pose;
    pose.attitude = pva.att.qbn.cast<float32_t>();
    pose.position = pva.pos.cast<float32_t>();
    pose.velocity = pva.vel.cast<float32_t>();
    return pose;
}

static pva_t mag_pose_to_pva(const mag_pose_t& pose)
{
    pva_t pva;
    pva.att.qbn = pose.attitude.cast<float64_t>();
    pva.att.euler = Rotation::quaternion2euler(pva.att.qbn);
    pva.att.cbn = Rotation::quaternion2matrix(pva.att.qbn);

    pva.pos = pose.position.cast<float64_t>();
    pva.vel = pose.velocity.cast<float64_t>();
    return pva;
}

// static mag_ins_data_t imu_to_mag_ins_data(const imu_t& imu)
// {
//     mag_ins_data_t data;
//     data.time_stamp = imu.time;
//     data.acc = imu.dvel.cast<float32_t>();
//     data.acc_bias = vector_3f_t::Zero();
//     data.gyr = imu.dtheta.cast<float32_t>();
//     data.gyr_bias = vector_3f_t::Zero();
//     return data;
// }

static imu_t mag_ins_data_to_imu(const mag_ins_data_t& ins_data)
{
    imu_t imu;
    imu.dt = 0.;
    imu.odovel = 0.;
    imu.time = ins_data.time_stamp;
    imu.dtheta = ins_data.gyr.cast<float64_t>();
    imu.dvel = ins_data.acc.cast<float64_t>();
    return imu;
}

mag_pose_t MagKalmanFilterGlobalIns::update_pose(
    const mag_pose_t& last_pose,
    const mag_ins_data_t& last_ins_data,
    const mag_ins_data_t& curr_ins_data)
{
    pva_t pva = mag_pose_to_pva(last_pose);
    pva_t curr_pva = pva;
    imu_t last_imu = mag_ins_data_to_imu(last_ins_data);
    imu_t curr_imu = mag_ins_data_to_imu(curr_ins_data);
    INSMech::insMech(pva, curr_pva, last_imu, curr_imu);
    return pva_to_mag_pose(curr_pva);
}

MAG_NAMESPACE_END