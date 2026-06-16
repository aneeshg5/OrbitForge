#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "sensors/gps.hpp"
#include "sensors/imu.hpp"
#include "sensors/magnetometer.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::sensors;

// ───────────────────────────────── GPS ────────────────────────────────────────

TEST(GPS, RotationPreservesMagnitude) {
    // ECI→ECEF rotation must be isometric (doesn't change vector length).
    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const Eigen::Matrix3d R = GpsSensor::R_ecef_eci(k_j2000_jd);
    const Eigen::Vector3d r_ecef = R * r_eci;
    EXPECT_NEAR(r_ecef.norm(), r0, 1.0);       // 1 m tolerance
    EXPECT_NEAR(r_ecef.z(), 0.0, 1.0);          // z unchanged (rotation around z)
}

TEST(GPS, RotationIsOrthogonal) {
    // R·Rᵀ must equal I₃ to 1e-12 relative tolerance.
    const Eigen::Matrix3d R = GpsSensor::R_ecef_eci(k_j2000_jd + 0.5);
    const Eigen::Matrix3d residual = R * R.transpose() - Eigen::Matrix3d::Identity();
    EXPECT_LT(residual.norm(), 1e-12);
}

TEST(GPS, NoiseStatistics) {
    // With N=2000 measurements at fixed seed, per-axis std dev should be within
    // 20% of sigma_m and the mean bias should be < 1 m.
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

    EXPECT_LT(mean_norm, 1.5);                          // bias < 1.5 m
    EXPECT_NEAR(rms_per_ax, sigma, 0.2 * sigma);        // std dev within 20%
}

// ───────────────────────────────── IMU ────────────────────────────────────────

TEST(IMU, InitialBiasIsZero) {
    ImuSensor imu;
    EXPECT_TRUE(imu.bias.isZero());
}

TEST(IMU, BiasDriftsWithTime) {
    // After 1000 unit steps, random-walk bias must have grown beyond zero.
    ImuSensor imu(0.05, 0.001, 42);
    for (int i = 0; i < 1000; ++i) imu.measure(Eigen::Vector3d::Zero(), 1.0);
    EXPECT_GT(imu.bias.norm(), 1e-4);   // > 0.1 mm/s² drift after ~1000 s
}

TEST(IMU, NoBiasWalkWithZeroDt) {
    // dt=0 must not advance the bias random walk.
    ImuSensor imu(0.05, 0.001, 42);
    for (int i = 0; i < 1000; ++i) imu.measure(Eigen::Vector3d::Zero(), 0.0);
    EXPECT_TRUE(imu.bias.isZero());
}

TEST(IMU, NoiseMeanNearTruth) {
    // With no bias walk (sigma_bias=0), mean of 2000 measurements should equal a_true.
    constexpr double sigma_acc = 0.05;
    ImuSensor imu(sigma_acc, 0.0, 77);
    const Eigen::Vector3d a_true(1.0, -2.0, 3.5);

    constexpr int N = 2000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (int i = 0; i < N; ++i) sum += imu.measure(a_true, 0.0);

    // Per-axis mean within 2-sigma of a_true (sigma_mean = sigma/sqrt(N) ≈ 0.0011)
    EXPECT_TRUE((sum / N - a_true).cwiseAbs().maxCoeff() < 3.0 * sigma_acc / std::sqrt(N));
}

// ──────────────────────────── Magnetometer ────────────────────────────────────

TEST(Magnetometer, FieldInPhysicalRange) {
    // At ISS altitude the total field magnitude must lie in [15000, 65000] nT.
    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const Eigen::Vector3d B = MagnetometerSensor::field_eci(r_eci, k_j2000_jd);
    EXPECT_GT(B.norm(), 15000.0);
    EXPECT_LT(B.norm(), 65000.0);
}

TEST(Magnetometer, FieldChangesWithPosition) {
    // B is position-dependent: two orthogonal orbit positions must give different fields.
    const double r0 = k_re + 408e3;
    const Eigen::Vector3d pos1(r0, 0.0, 0.0);
    const Eigen::Vector3d pos2(0.0, r0, 0.0);

    const Eigen::Vector3d B1 = MagnetometerSensor::field_eci(pos1, k_j2000_jd);
    const Eigen::Vector3d B2 = MagnetometerSensor::field_eci(pos2, k_j2000_jd);

    // Relative difference must exceed 1% — would fail for a constant model
    EXPECT_GT((B1 - B2).norm() / std::max(B1.norm(), B2.norm()), 0.01);
}

TEST(Magnetometer, NoiseIsAdditive) {
    // Mean of N measurements must be close to noiseless field.
    constexpr double sigma = 100.0;
    MagnetometerSensor mag(sigma, 99);

    const double r0 = k_re + 408e3;
    const Eigen::Vector3d r_eci(r0, 0.0, 0.0);
    const double jd = k_j2000_jd;
    const Eigen::Vector3d B_true = MagnetometerSensor::field_eci(r_eci, jd);

    constexpr int N = 2000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (int i = 0; i < N; ++i) sum += mag.measure(r_eci, jd);

    // Mean bias per axis < 3-sigma/sqrt(N) ≈ 6.7 nT
    EXPECT_LT((sum / N - B_true).cwiseAbs().maxCoeff(), 4.0 * sigma / std::sqrt(N));
}
