#pragma once

#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>

#include "constants.hpp"

namespace orbitforge {

struct ScenarioCfg {
    char   tle_line1[70]{};
    char   tle_line2[70]{};
    double gps_sigma    = 10.0;
    double imu_sigma    = 0.05;
    bool   enable_j2    = true;
    bool   enable_drag  = true;
    bool   enable_srp   = false;
    double drag_coeff   = 2.2;
    double area_to_mass = 0.01;
    double q_pos        = 1.0;
    double q_vel        = 0.01;
    double sim_speed    = 1.0;
    int    seed         = -1;

    double inertia_x   = 1.0;
    double inertia_y   = 1.0;
    double inertia_z   = 1.0;
    // math.md §7.4.
    double gyro_sigma  = 0.001;
    // math.md §7.4.
    double mag_sigma   = 100.0;
    double q_att        = 1e-6;
    double q_omega       = 1e-8;
    double init_omega_x = 0.0;
    double init_omega_y = 0.0;
    double init_omega_z = 0.05;
};

struct TleElements {
    double epoch_jd          = 0.0;
    double inclination_rad   = 0.0;
    double raan_rad          = 0.0;
    double eccentricity      = 0.0;
    double arg_perigee_rad   = 0.0;
    double mean_anomaly_rad  = 0.0;
    double mean_motion_rad_s = 0.0;
};

struct EciState {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
};

namespace scenario_detail {

constexpr double k_deg_to_rad = 1.7453292519943295769e-2;
constexpr double k_two_pi     = 6.283185307179586477;

inline double parse_field(const char* line, int start, int len) {
    char buf[16] = {};
    const int n = (len < 15) ? len : 15;
    for (int i = 0; i < n; ++i) buf[i] = line[start + i];
    return std::atof(buf);
}

inline double tle_epoch_to_jd(int year_2d, double day_of_year) {
    const int year = (year_2d >= 57) ? 1900 + year_2d : 2000 + year_2d;
    const int yp = year - 1;
    const int a = yp / 100;
    const int b = 2 - a + (a / 4);
    const double jd0 = std::floor(365.25 * (yp + 4716)) + 428.0 + b - 1524.5;
    return jd0 + day_of_year;
}

}

inline TleElements parse_tle(const char* line1, const char* line2) {
    using namespace scenario_detail;

    const int year_2d = static_cast<int>(parse_field(line1, 18, 2));
    const double day_of_year = parse_field(line1, 20, 12);

    TleElements elem;
    elem.epoch_jd         = tle_epoch_to_jd(year_2d, day_of_year);
    elem.inclination_rad  = parse_field(line2, 8, 8) * k_deg_to_rad;
    elem.raan_rad         = parse_field(line2, 17, 8) * k_deg_to_rad;
    elem.eccentricity     = parse_field(line2, 26, 7) * 1e-7;
    elem.arg_perigee_rad  = parse_field(line2, 34, 8) * k_deg_to_rad;
    elem.mean_anomaly_rad = parse_field(line2, 43, 8) * k_deg_to_rad;

    const double mean_motion_rev_day = parse_field(line2, 52, 11);
    elem.mean_motion_rad_s = mean_motion_rev_day * scenario_detail::k_two_pi / k_sec_per_day;
    return elem;
}

inline double solve_kepler_eccentric_anomaly(double mean_anomaly_rad, double eccentricity) {
    double E = mean_anomaly_rad;
    for (int iter = 0; iter < 50; ++iter) {
        const double f  = E - eccentricity * std::sin(E) - mean_anomaly_rad;
        const double fp = 1.0 - eccentricity * std::cos(E);
        const double dE = f / fp;
        E -= dE;
        if (std::fabs(dE) < 1e-12) break;
    }
    return E;
}

inline EciState tle_elements_to_eci(const TleElements& elem) {
    const double n = elem.mean_motion_rad_s;
    const double a = std::cbrt(k_mu / (n * n));
    const double e = elem.eccentricity;

    const double E = solve_kepler_eccentric_anomaly(elem.mean_anomaly_rad, e);
    const double cos_E = std::cos(E);
    const double sin_E = std::sin(E);

    const double r_mag = a * (1.0 - e * cos_E);
    const double p = a * (1.0 - e * e);
    const double sqrt_mu_p = std::sqrt(k_mu / p);

    const double denom = 1.0 - e * cos_E;
    const double sin_nu = (std::sqrt(1.0 - e * e) * sin_E) / denom;
    const double cos_nu = (cos_E - e) / denom;

    const Eigen::Vector3d r_pqw(r_mag * cos_nu, r_mag * sin_nu, 0.0);
    const Eigen::Vector3d v_pqw(sqrt_mu_p * -sin_nu, sqrt_mu_p * (e + cos_nu), 0.0);

    const double cos_o = std::cos(elem.raan_rad);
    const double sin_o = std::sin(elem.raan_rad);
    const double cos_i = std::cos(elem.inclination_rad);
    const double sin_i = std::sin(elem.inclination_rad);
    const double cos_w = std::cos(elem.arg_perigee_rad);
    const double sin_w = std::sin(elem.arg_perigee_rad);

    Eigen::Matrix3d R;
    R(0, 0) = cos_o * cos_w - sin_o * sin_w * cos_i;
    R(0, 1) = -cos_o * sin_w - sin_o * cos_w * cos_i;
    R(0, 2) = sin_o * sin_i;
    R(1, 0) = sin_o * cos_w + cos_o * sin_w * cos_i;
    R(1, 1) = -sin_o * sin_w + cos_o * cos_w * cos_i;
    R(1, 2) = -cos_o * sin_i;
    R(2, 0) = sin_w * sin_i;
    R(2, 1) = cos_w * sin_i;
    R(2, 2) = cos_i;

    EciState state;
    state.pos = R * r_pqw;
    state.vel = R * v_pqw;
    return state;
}

}
