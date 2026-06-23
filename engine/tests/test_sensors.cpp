#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "sensors/gps.hpp"
#include "sensors/imu.hpp"
#include "sensors/gyro.hpp"
#include "sensors/magnetometer.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::sensors;

TEST(GPS, RotationPreservesMagnitude) {
    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const Eigen::Matrix3d R = GpsSensor::R_ecef_eci(k_j2000_jd);
    const Eigen::Vector3d r_ecef = R * r_eci;
    EXPECT_NEAR(r_ecef.norm(), r0, 1.0);
    EXPECT_NEAR(r_ecef.z(), 0.0, 1.0);
}

TEST(GPS, RotationIsOrthogonal) {
    const Eigen::Matrix3d R = GpsSensor::R_ecef_eci(k_j2000_jd + 0.5);
    const Eigen::Matrix3d residual = R * R.transpose() - Eigen::Matrix3d::Identity();
    EXPECT_LT(residual.norm(), 1e-12);
}

TEST(GPS, NoiseStatistics) {
    constexpr double sigma = 10.0;
    GpsSensor gps(sigma, 123);

    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const double jd = k_j2000_jd;
    const Eigen::Vector3d r_ecef = GpsSensor::R_ecef_eci(jd) * r_eci;

    constexpr int N = 2000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    double sq_err = 0.0;
    for (int i = 0; i < N; ++i) {
        const Eigen::Vector3d noise = gps.measure(r_eci, jd) - r_ecef;
        sum     += noise;
        sq_err  += noise.squaredNorm();
    }
    const double mean_norm  = sum.norm() / N;
    const double rms_per_ax = std::sqrt(sq_err / (3.0 * N));

    EXPECT_LT(mean_norm, 1.5);
    EXPECT_NEAR(rms_per_ax, sigma, 0.2 * sigma);
}

TEST(IMU, InitialBiasIsZero) {
    ImuSensor imu;
    EXPECT_TRUE(imu.bias.isZero());
}

TEST(IMU, BiasDriftsWithTime) {
    ImuSensor imu(0.05, 0.001, 42);
    for (int i = 0; i < 1000; ++i) imu.measure(Eigen::Vector3d::Zero(), 1.0);
    EXPECT_GT(imu.bias.norm(), 1e-4);
}

TEST(IMU, NoBiasWalkWithZeroDt) {
    ImuSensor imu(0.05, 0.001, 42);
    for (int i = 0; i < 1000; ++i) imu.measure(Eigen::Vector3d::Zero(), 0.0);
    EXPECT_TRUE(imu.bias.isZero());
}

TEST(IMU, NoiseMeanNearTruth) {
    constexpr double sigma_acc = 0.05;
    ImuSensor imu(sigma_acc, 0.0, 77);
    const Eigen::Vector3d a_true(1.0, -2.0, 3.5);

    constexpr int N = 2000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (int i = 0; i < N; ++i) sum += imu.measure(a_true, 0.0);

    EXPECT_TRUE((sum / N - a_true).cwiseAbs().maxCoeff() < 3.0 * sigma_acc / std::sqrt(N));
}

TEST(Magnetometer, FieldInPhysicalRange) {
    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const Eigen::Vector3d B = MagnetometerSensor::field_eci(r_eci, k_j2000_jd);
    EXPECT_GT(B.norm(), 15000.0);
    EXPECT_LT(B.norm(), 65000.0);
}

TEST(Magnetometer, FieldChangesWithPosition) {
    const double r0 = k_re + 408e3;
    const Eigen::Vector3d pos1(r0, 0.0, 0.0);
    const Eigen::Vector3d pos2(0.0, r0, 0.0);

    const Eigen::Vector3d B1 = MagnetometerSensor::field_eci(pos1, k_j2000_jd);
    const Eigen::Vector3d B2 = MagnetometerSensor::field_eci(pos2, k_j2000_jd);

    EXPECT_GT((B1 - B2).norm() / std::max(B1.norm(), B2.norm()), 0.01);
}

TEST(Magnetometer, NoiseIsAdditive) {
    constexpr double sigma = 100.0;
    MagnetometerSensor mag(sigma, 99);

    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const double jd = k_j2000_jd;
    const Eigen::Vector3d B_true = MagnetometerSensor::field_eci(r_eci, jd);

    constexpr int N = 2000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (int i = 0; i < N; ++i) sum += mag.measure(r_eci, jd);

    EXPECT_LT((sum / N - B_true).cwiseAbs().maxCoeff(), 4.0 * sigma / std::sqrt(N));
}

TEST(Magnetometer, MeasureBodyRotatesEciField) {
    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const double jd = k_j2000_jd;
    const Eigen::Vector3d B_eci = MagnetometerSensor::field_eci(r_eci, jd);

    const Eigen::Matrix3d R_body_eci = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d expected_body = R_body_eci * B_eci;

    MagnetometerSensor mag(0.0, 11);
    const Eigen::Vector3d measured = mag.measure_body(r_eci, jd, R_body_eci);
    EXPECT_TRUE(measured.isApprox(expected_body, 1e-9));
}

TEST(Magnetometer, MeasureBodyIdentityMatchesEci) {
    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const double jd = k_j2000_jd;
    const Eigen::Vector3d B_eci = MagnetometerSensor::field_eci(r_eci, jd);

    MagnetometerSensor mag(0.0, 12);
    const Eigen::Vector3d measured = mag.measure_body(r_eci, jd, Eigen::Matrix3d::Identity());
    EXPECT_TRUE(measured.isApprox(B_eci, 1e-9));
}

TEST(Gyro, InitialBiasIsZero) {
    GyroSensor gyro;
    EXPECT_TRUE(gyro.bias.isZero());
}

TEST(Gyro, BiasDriftsWithTime) {
    GyroSensor gyro(0.001, 0.0005, 43);
    for (int i = 0; i < 1000; ++i) gyro.measure(Eigen::Vector3d::Zero(), 1.0);
    EXPECT_GT(gyro.bias.norm(), 1e-6);
}

TEST(Gyro, NoBiasWalkWithZeroDt) {
    GyroSensor gyro(0.001, 0.0005, 43);
    for (int i = 0; i < 1000; ++i) gyro.measure(Eigen::Vector3d::Zero(), 0.0);
    EXPECT_TRUE(gyro.bias.isZero());
}

TEST(Gyro, NoiseMeanNearTruth) {
    constexpr double sigma_gyro = 0.001;
    GyroSensor gyro(sigma_gyro, 0.0, 78);
    const Eigen::Vector3d omega_true(0.05, -0.02, 0.03);

    constexpr int N = 2000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (int i = 0; i < N; ++i) sum += gyro.measure(omega_true, 0.0);

    EXPECT_TRUE((sum / N - omega_true).cwiseAbs().maxCoeff() < 3.0 * sigma_gyro / std::sqrt(N));
}
