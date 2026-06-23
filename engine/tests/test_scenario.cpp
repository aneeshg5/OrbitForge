#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>

#include "constants.hpp"
#include "scenario.hpp"

using namespace orbitforge;

namespace {

constexpr double k_deg_to_rad = 1.7453292519943295769e-2;

std::string make_tle_line1(int sat_num, int epoch_year_2d, double epoch_day) {
    std::string line(69, ' ');
    line[0] = '1';
    char field[16];
    std::snprintf(field, sizeof(field), "%05d", sat_num);
    line.replace(2, 5, field);
    line[7] = 'U';
    std::snprintf(field, sizeof(field), "%02d", epoch_year_2d);
    line.replace(18, 2, field);
    std::snprintf(field, sizeof(field), "%012.8f", epoch_day);
    line.replace(20, 12, field);
    return line;
}

std::string make_tle_line2(int sat_num, double incl_deg, double raan_deg, double ecc,
                            double argp_deg, double ma_deg, double mean_motion_rev_day) {
    std::string line(69, ' ');
    line[0] = '2';
    char field[16];
    std::snprintf(field, sizeof(field), "%05d", sat_num);
    line.replace(2, 5, field);
    std::snprintf(field, sizeof(field), "%8.4f", incl_deg);
    line.replace(8, 8, field);
    std::snprintf(field, sizeof(field), "%8.4f", raan_deg);
    line.replace(17, 8, field);
    std::snprintf(field, sizeof(field), "%07d", static_cast<int>(std::lround(ecc * 1e7)));
    line.replace(26, 7, field);
    std::snprintf(field, sizeof(field), "%8.4f", argp_deg);
    line.replace(34, 8, field);
    std::snprintf(field, sizeof(field), "%8.4f", ma_deg);
    line.replace(43, 8, field);
    std::snprintf(field, sizeof(field), "%11.8f", mean_motion_rev_day);
    line.replace(52, 11, field);
    return line;
}

TleElements iss_like_elements() {
    const std::string l1 = make_tle_line1(25544, 24, 1.5);
    const std::string l2 =
        make_tle_line2(25544, 51.6400, 247.4627, 0.0006703, 130.5360, 325.0288, 15.49000000);
    return parse_tle(l1.c_str(), l2.c_str());
}

}

TEST(ParseTle, ExtractsKnownElements) {
    const TleElements elem = iss_like_elements();

    EXPECT_NEAR(elem.inclination_rad, 51.6400 * k_deg_to_rad, 1e-9);
    EXPECT_NEAR(elem.raan_rad, 247.4627 * k_deg_to_rad, 1e-9);
    EXPECT_NEAR(elem.eccentricity, 0.0006703, 1e-9);
    EXPECT_NEAR(elem.arg_perigee_rad, 130.5360 * k_deg_to_rad, 1e-9);
    EXPECT_NEAR(elem.mean_anomaly_rad, 325.0288 * k_deg_to_rad, 1e-9);

    const double expected_n = 15.49000000 * 2.0 * 3.14159265358979323846 / k_sec_per_day;
    EXPECT_NEAR(elem.mean_motion_rad_s, expected_n, 1e-12);
}

TEST(ParseTle, EpochJulianDateInExpectedRange) {
    const TleElements elem = iss_like_elements();
    EXPECT_GT(elem.epoch_jd, 2460300.0);
    EXPECT_LT(elem.epoch_jd, 2460320.0);
}

TEST(SolveKepler, ZeroEccentricityMeanEqualsEccentric) {
    for (double M = 0.0; M < 6.0; M += 0.5) {
        EXPECT_NEAR(solve_kepler_eccentric_anomaly(M, 0.0), M, 1e-12);
    }
}

TEST(SolveKepler, SatisfiesKeplersEquation) {
    const double M = 2.3;
    const double e = 0.6;
    const double E = solve_kepler_eccentric_anomaly(M, e);
    EXPECT_NEAR(E - e * std::sin(E), M, 1e-10);
}

TEST(TleElementsToEci, VisVivaConsistency) {
    const TleElements elem = iss_like_elements();
    const double a = std::cbrt(k_mu / (elem.mean_motion_rad_s * elem.mean_motion_rad_s));

    const EciState state = tle_elements_to_eci(elem);
    const double r = state.pos.norm();
    const double v2_expected = k_mu * (2.0 / r - 1.0 / a);

    EXPECT_NEAR(state.vel.squaredNorm(), v2_expected, v2_expected * 1e-9);
}

TEST(TleElementsToEci, AngularMomentumInclinationMatches) {
    const TleElements elem = iss_like_elements();
    const EciState state = tle_elements_to_eci(elem);

    const Eigen::Vector3d h = state.pos.cross(state.vel);
    const double inclination = std::acos(h.z() / h.norm());

    EXPECT_NEAR(inclination, elem.inclination_rad, 1e-9);
}

TEST(TleElementsToEci, CircularOrbitRadiusMatchesSemiMajorAxis) {
    const std::string l1 = make_tle_line1(99999, 24, 1.0);
    const std::string l2 = make_tle_line2(99999, 51.6, 0.0, 0.0, 0.0, 0.0, 15.5);
    const TleElements elem = parse_tle(l1.c_str(), l2.c_str());

    const double a = std::cbrt(k_mu / (elem.mean_motion_rad_s * elem.mean_motion_rad_s));
    const EciState state = tle_elements_to_eci(elem);

    EXPECT_NEAR(state.pos.norm(), a, a * 1e-9);
    EXPECT_NEAR(state.vel.norm(), std::sqrt(k_mu / a), std::sqrt(k_mu / a) * 1e-9);
}
