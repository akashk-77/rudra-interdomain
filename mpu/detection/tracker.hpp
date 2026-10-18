#pragma once
#include <cmath>
#include <optional>

struct Vec2 { float x = 0, y = 0; };

struct Candidate {
    Vec2  centroid;         
    float area_frac = 0;    // blob area / searched area
    float mean_z    = 0;    // mean anomaly score inside the blob
};

struct TrackerConfig {      
    int   confirm_frames = 3;
    int   lost_frames    = 5;
    float gate_px        = 60.f;   // max centroid jump between frames to count as "same"
    float ema_alpha      = 0.5f;   
};

enum class TrackStatus { Searching, Tracking, Confirmed, Coasting };

struct TrackOutput {
    TrackStatus status = TrackStatus::Searching;
    Vec2  centroid;
    float area_frac = 0, mean_z = 0;
    int   hits = 0, misses = 0;
    bool  newly_confirmed = false;   // true only on the frame confirmation happens
};

class SiteTracker {
public:
    explicit SiteTracker(const TrackerConfig& cfg = {}) : cfg_(cfg) {}

    TrackOutput update(const std::optional<Candidate>& c) {
        bool newly = false;
        if (!active_) {
            if (c) start(*c);
        } else if (c && dist(c->centroid, t_.centroid) <= cfg_.gate_px) {
            const float a = cfg_.ema_alpha;
            t_.centroid.x = a * c->centroid.x + (1 - a) * t_.centroid.x;
            t_.centroid.y = a * c->centroid.y + (1 - a) * t_.centroid.y;
            t_.area_frac  = a * c->area_frac  + (1 - a) * t_.area_frac;
            t_.mean_z     = a * c->mean_z     + (1 - a) * t_.mean_z;
            t_.hits++;
            t_.misses = 0;
        } else {
            t_.misses++;
            if (t_.misses >= cfg_.lost_frames) {
                active_ = false;
                if (c) start(*c);          
            }
        }
        if (active_ && !t_.confirmed && t_.hits >= cfg_.confirm_frames) {
            t_.confirmed = true;
            newly = true;
        }
        return output(newly);
    }

    void reset() { active_ = false; }

private:
    struct Track {
        Vec2 centroid; float area_frac = 0, mean_z = 0;
        int hits = 0, misses = 0; bool confirmed = false;
    };

    static float dist(const Vec2& a, const Vec2& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }
    void start(const Candidate& c) {
        t_ = Track{};
        t_.centroid = c.centroid; t_.area_frac = c.area_frac; t_.mean_z = c.mean_z;
        t_.hits = 1;
        active_ = true;
    }
    TrackOutput output(bool newly) const {
        TrackOutput o;
        o.newly_confirmed = newly;
        if (!active_) return o;
        o.centroid = t_.centroid; o.area_frac = t_.area_frac; o.mean_z = t_.mean_z;
        o.hits = t_.hits; o.misses = t_.misses;
        if (!t_.confirmed)        o.status = TrackStatus::Tracking;
        else if (t_.misses > 0)   o.status = TrackStatus::Coasting;
        else                      o.status = TrackStatus::Confirmed;
        return o;
    }

    TrackerConfig cfg_;
    Track t_;
    bool active_ = false;
};
