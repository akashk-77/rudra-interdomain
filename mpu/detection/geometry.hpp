
#pragma once
#include <cmath>

namespace geom {

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

struct Intrinsics {            
    double fx = 0, fy = 0, cx = 0, cy = 0;
    bool valid = false;
};

struct Mount {                 // how the camera sits on the rover
    double height_m = 0;       // lens height above the ground
    double pitch_deg = 0;      // downward tilt below horizontal (positive = looking down)
    bool valid = false;
};

struct GroundPoint {
    bool on_ground = false;    // false if the ray never meets the ground (at/above horizon)
    double forward_m = 0;      // along the rover's forward axis
    double lateral_m = 0;      // positive = to the right
    double distance_m = 0;     // straight-line distance on the ground plane
    double bearing_deg = 0;    // positive = to the right of straight ahead
};

inline Intrinsics scale_intrinsics(const Intrinsics& k, double sx, double sy) {
    Intrinsics o = k;
    o.fx *= sx; o.cx *= sx;
    o.fy *= sy; o.cy *= sy;
    return o;
}

// Ray direction in a level world frame (x right, y down, z forward) for a camera
// pitched down by pitch_deg.
struct Ray { double dx, dy, dz; };
inline Ray ray_world(double xn, double yn, double pitch_deg) {
    const double th = deg2rad(pitch_deg);
    return { xn, yn * std::cos(th) + std::sin(th), std::cos(th) - yn * std::sin(th) };
}

// Horizontal bearing of the ray (degrees, + = right).  With pitch_deg = 0 this is
// atan(xn), the angle off the optical axis.
inline double bearing_deg(double xn, double yn, double pitch_deg = 0.0) {
    const Ray r = ray_world(xn, yn, pitch_deg);
    return rad2deg(std::atan2(r.dx, r.dz));
}

// Intersect the ray with flat ground.
inline GroundPoint ground_from_normalized(double xn, double yn, const Mount& m) {
    GroundPoint g;
    const Ray r = ray_world(xn, yn, m.pitch_deg);
    if (!m.valid || m.height_m <= 0.0 || r.dy <= 1e-6) return g;   // at/above horizon
    const double t = m.height_m / r.dy;
    g.forward_m = t * r.dz;
    g.lateral_m = t * r.dx;
    if (g.forward_m <= 0.0) return g;                               // behind the camera
    g.distance_m = std::hypot(g.forward_m, g.lateral_m);
    g.bearing_deg = rad2deg(std::atan2(g.lateral_m, g.forward_m));
    g.on_ground = true;
    return g;
}

}  
