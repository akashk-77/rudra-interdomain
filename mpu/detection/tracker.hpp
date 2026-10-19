#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

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


// ---------------------------------------------------------------------------
// MultiTracker - follows several sites at once, each with its own stable id.
// Every frame the detector gives a list of candidates; each is matched to the
// nearest existing track (within gate_px).  Unmatched candidates start new
// tracks (up to max_tracks).  A track is CONFIRMED after confirm_frames hits and
// dropped after lost_frames consecutive misses.
// ---------------------------------------------------------------------------
struct TrackedSite {
    int         id = 0;       // stable for the life of the track
    TrackOutput out;
};

class MultiTracker {
public:
    explicit MultiTracker(const TrackerConfig& cfg = {}, int max_tracks = 3)
        : cfg_(cfg), max_tracks_(max_tracks < 1 ? 1 : static_cast<size_t>(max_tracks)) {}

    std::vector<TrackedSite> update(const std::vector<Candidate>& cands) {
        const size_t nt = tracks_.size(), nc = cands.size();
        for (Track& t : tracks_) t.newly = false;

        // 1. Greedy nearest-first matching of candidates to existing tracks.
        struct Pair { float d; size_t t, c; };
        std::vector<Pair> pairs;
        for (size_t t = 0; t < nt; ++t)
            for (size_t c = 0; c < nc; ++c) {
                const float d = dist(cands[c].centroid, tracks_[t].centroid);
                if (d <= cfg_.gate_px) pairs.push_back({d, t, c});
            }
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });

        std::vector<char> tUsed(nt, 0), cUsed(nc, 0);
        for (const Pair& p : pairs) {
            if (tUsed[p.t] || cUsed[p.c]) continue;
            tUsed[p.t] = cUsed[p.c] = 1;
            Track& t = tracks_[p.t];
            const Candidate& c = cands[p.c];
            const float a = cfg_.ema_alpha;
            t.centroid.x = a * c.centroid.x + (1 - a) * t.centroid.x;
            t.centroid.y = a * c.centroid.y + (1 - a) * t.centroid.y;
            t.area_frac  = a * c.area_frac  + (1 - a) * t.area_frac;
            t.mean_z     = a * c.mean_z     + (1 - a) * t.mean_z;
            t.hits++;
            t.misses = 0;
        }

        // 2. Unmatched tracks miss a frame; drop the ones that are lost.
        for (size_t t = 0; t < nt; ++t)
            if (!tUsed[t]) tracks_[t].misses++;
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                          [&](const Track& t) { return t.misses >= cfg_.lost_frames; }),
                      tracks_.end());

        // 3. Unmatched candidates start new tracks while there is room.
        for (size_t c = 0; c < nc && tracks_.size() < max_tracks_; ++c) {
            if (cUsed[c]) continue;
            Track t;
            t.id = next_id_++;
            t.centroid = cands[c].centroid; t.area_frac = cands[c].area_frac; t.mean_z = cands[c].mean_z;
            t.hits = 1;
            tracks_.push_back(t);
        }

        // 4. Confirmation and output.
        std::vector<TrackedSite> out;
        for (Track& t : tracks_) {
            if (!t.confirmed && t.hits >= cfg_.confirm_frames) { t.confirmed = true; t.newly = true; }
            TrackedSite s;
            s.id = t.id;
            s.out.centroid = t.centroid; s.out.area_frac = t.area_frac; s.out.mean_z = t.mean_z;
            s.out.hits = t.hits; s.out.misses = t.misses;
            s.out.newly_confirmed = t.newly;
            if (!t.confirmed)       s.out.status = TrackStatus::Tracking;
            else if (t.misses > 0)  s.out.status = TrackStatus::Coasting;
            else                    s.out.status = TrackStatus::Confirmed;
            out.push_back(s);
        }
        return out;
    }

    void reset() { tracks_.clear(); }

private:
    struct Track {
        int id = 0;
        Vec2 centroid; float area_frac = 0, mean_z = 0;
        int hits = 0, misses = 0;
        bool confirmed = false, newly = false;
    };
    static float dist(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

    TrackerConfig cfg_;
    size_t max_tracks_;
    std::vector<Track> tracks_;
    int next_id_ = 1;
};
