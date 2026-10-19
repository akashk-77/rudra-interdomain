// test_core.cpp - tests for geometry.hpp and tracker.hpp
// Build/run: make test
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <utility>
#include <vector>

#include "geometry.hpp"
#include "tracker.hpp"

static int g_checks = 0, g_failed = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_failed++; \
    std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

using namespace geom;

static Mount mount(double h, double pitch) { Mount m; m.height_m = h; m.pitch_deg = pitch; m.valid = true; return m; }

static void test_centre_pixel_45deg_down() {
    GroundPoint g = ground_from_normalized(0, 0, mount(1.0, 45.0));
    CHECK(g.on_ground);
    NEAR(g.forward_m, 1.0, 1e-9);          // 45 deg down from 1 m height -> 1 m ahead
    NEAR(g.lateral_m, 0.0, 1e-9);
    NEAR(g.bearing_deg, 0.0, 1e-9);
}

static void test_horizontal_ray_never_hits_ground() {
    CHECK(!ground_from_normalized(0, 0, mount(1.0, 0.0)).on_ground);
}

static void test_ray_above_horizon_rejected() {
    CHECK(!ground_from_normalized(0, -2.0, mount(1.0, 30.0)).on_ground);   // 30 - 63.4 deg
}

static void test_invalid_mount_rejected() {
    Mount m; CHECK(!ground_from_normalized(0, 0.5, m).on_ground);
    CHECK(!ground_from_normalized(0, 0.5, mount(0.0, 30.0)).on_ground);
}

static void test_distance_matches_independent_formula() {
    // Independent check: depression = pitch + atan(yn); forward = h / tan(depression)
    const double h = 0.8, pitch = 30.0, yn = 0.5;
    const double dep = deg2rad(pitch) + std::atan(yn);
    GroundPoint g = ground_from_normalized(0, yn, mount(h, pitch));
    CHECK(g.on_ground);
    NEAR(g.forward_m, h / std::tan(dep), 1e-9);
}

static void test_lateral_sign_and_bearing() {
    GroundPoint r = ground_from_normalized(0.5, 0, mount(1.0, 45.0));
    GroundPoint l = ground_from_normalized(-0.5, 0, mount(1.0, 45.0));
    CHECK(r.lateral_m > 0 && l.lateral_m < 0);
    NEAR(r.bearing_deg, rad2deg(std::atan(0.5 / std::cos(deg2rad(45.0)))), 1e-9);
    NEAR(r.bearing_deg, -l.bearing_deg, 1e-9);
}

static void test_bearing_without_mount() {
    NEAR(bearing_deg(0.0, 0.0), 0.0, 1e-12);
    NEAR(bearing_deg(1.0, 0.0), 45.0, 1e-9);
    NEAR(bearing_deg(-1.0, 0.0), -45.0, 1e-9);
}

static void test_scale_intrinsics() {
    Intrinsics k; k.fx = 1000; k.fy = 1000; k.cx = 640; k.cy = 360; k.valid = true;
    Intrinsics s = scale_intrinsics(k, 0.5, 0.5);
    NEAR(s.fx, 500, 1e-9); NEAR(s.cx, 320, 1e-9); NEAR(s.cy, 180, 1e-9);
    CHECK(s.valid);
}

// tracker 
static Candidate cand(float x, float y) { Candidate c; c.centroid = {x, y}; c.area_frac = 0.01f; c.mean_z = 10.f; return c; }
static const std::optional<Candidate> NONE = std::nullopt;

static void test_tracker_confirms_after_n_hits_once() {
    SiteTracker t;                               // confirm_frames = 3
    CHECK(t.update(cand(100, 100)).status == TrackStatus::Tracking);
    CHECK(t.update(cand(102, 101)).status == TrackStatus::Tracking);
    TrackOutput o = t.update(cand(101, 100));
    CHECK(o.status == TrackStatus::Confirmed && o.newly_confirmed);
    CHECK(!t.update(cand(101, 100)).newly_confirmed);
}

static void test_tracker_single_frame_blip_is_ignored() {
    SiteTracker t;
    t.update(cand(100, 100));
    for (int i = 0; i < 6; i++) CHECK(t.update(NONE).status != TrackStatus::Confirmed);
    CHECK(t.update(NONE).status == TrackStatus::Searching);
}

static void test_tracker_coasts_then_drops() {
    SiteTracker t;
    for (int i = 0; i < 3; i++) t.update(cand(100, 100));
    for (int i = 0; i < 4; i++) CHECK(t.update(NONE).status == TrackStatus::Coasting);
    CHECK(t.update(NONE).status == TrackStatus::Searching);       // 5th miss -> lost
}

static void test_tracker_recovers_after_short_gap() {
    SiteTracker t;
    for (int i = 0; i < 3; i++) t.update(cand(100, 100));
    t.update(NONE); t.update(NONE);
    TrackOutput o = t.update(cand(101, 100));
    CHECK(o.status == TrackStatus::Confirmed && o.misses == 0 && !o.newly_confirmed);
}

static void test_tracker_far_candidate_does_not_hijack() {
    SiteTracker t;
    for (int i = 0; i < 3; i++) t.update(cand(100, 100));
    TrackOutput o = t.update(cand(500, 400));                     
    CHECK(o.status == TrackStatus::Coasting);
    NEAR(o.centroid.x, 100.0, 1e-3);
}

static void test_tracker_switches_to_new_site_after_lost() {
    SiteTracker t;
    for (int i = 0; i < 3; i++) t.update(cand(100, 100));
    TrackOutput o;
    for (int i = 0; i < 5; i++) o = t.update(cand(500, 400));
    CHECK(o.status == TrackStatus::Tracking);                      
    NEAR(o.centroid.x, 500.0, 1e-3);
    for (int i = 0; i < 2; i++) o = t.update(cand(500, 400));
    CHECK(o.status == TrackStatus::Confirmed && o.newly_confirmed);   // new site confirmed
}

static void test_tracker_smooths_centroid() {
    TrackerConfig c; c.ema_alpha = 0.5f;
    SiteTracker t(c);
    t.update(cand(100, 100));
    TrackOutput o = t.update(cand(120, 100));
    NEAR(o.centroid.x, 110.0, 1e-4);
}


// ---- multi tracker ---------------------------------------------------------
typedef std::vector<Candidate> Cands;
static Cands C(std::initializer_list<std::pair<float, float>> pts) {
    Cands v; for (auto& p : pts) v.push_back(cand(p.first, p.second)); return v;
}
static const TrackedSite* near_site(const std::vector<TrackedSite>& s, float x, float y) {
    for (auto& t : s) if (std::hypot(t.out.centroid.x - x, t.out.centroid.y - y) < 30.f) return &t;
    return nullptr;
}

static void test_multi_confirms_each_site_with_distinct_ids() {
    MultiTracker m;
    std::vector<TrackedSite> s;
    for (int i = 0; i < 3; i++) s = m.update(C({{100, 100}, {300, 200}, {500, 300}}));
    CHECK(s.size() == 3);
    for (auto& t : s) CHECK(t.out.status == TrackStatus::Confirmed && t.out.newly_confirmed);
    CHECK(s[0].id != s[1].id && s[1].id != s[2].id && s[0].id != s[2].id);
    s = m.update(C({{100, 100}, {300, 200}, {500, 300}}));
    for (auto& t : s) CHECK(!t.out.newly_confirmed);
}

static void test_multi_ids_follow_position_not_list_order() {
    MultiTracker m;
    std::vector<TrackedSite> s = m.update(C({{100, 100}, {300, 200}, {500, 300}}));
    const int idA = near_site(s, 100, 100)->id, idB = near_site(s, 300, 200)->id;
    s = m.update(C({{500, 300}, {100, 100}, {300, 200}}));        // same sites, different order
    CHECK(near_site(s, 100, 100)->id == idA);
    CHECK(near_site(s, 300, 200)->id == idB);
}

static void test_multi_one_site_flickers_others_stay_confirmed() {
    MultiTracker m;
    std::vector<TrackedSite> s;
    for (int i = 0; i < 3; i++) s = m.update(C({{100, 100}, {300, 200}, {500, 300}}));
    const int idB = near_site(s, 300, 200)->id;
    s = m.update(C({{100, 100}, {500, 300}}));                    // B missing this frame
    CHECK(near_site(s, 100, 100)->out.status == TrackStatus::Confirmed);
    CHECK(near_site(s, 500, 300)->out.status == TrackStatus::Confirmed);
    CHECK(near_site(s, 300, 200)->out.status == TrackStatus::Coasting);
    s = m.update(C({{100, 100}, {300, 200}, {500, 300}}));        // B is back
    CHECK(near_site(s, 300, 200)->id == idB);
    CHECK(near_site(s, 300, 200)->out.status == TrackStatus::Confirmed);
}

static void test_multi_capacity_limit() {
    MultiTracker m(TrackerConfig(), 3);
    auto s = m.update(C({{50, 50}, {200, 50}, {350, 50}, {500, 50}}));
    CHECK(s.size() == 3);
}

static void test_multi_lost_track_frees_slot_for_new_site() {
    MultiTracker m(TrackerConfig(), 3);
    std::vector<TrackedSite> s;
    for (int i = 0; i < 3; i++) s = m.update(C({{100, 100}, {300, 200}, {500, 300}}));
    for (int i = 0; i < 5; i++) s = m.update(C({{100, 100}, {300, 200}}));   // C vanishes
    CHECK(s.size() == 2);
    s = m.update(C({{100, 100}, {300, 200}, {50, 400}}));
    CHECK(s.size() == 3);
    const TrackedSite* n = near_site(s, 50, 400);
    CHECK(n && n->id == 4 && n->out.status == TrackStatus::Tracking);
}

static void test_multi_empty_frames_drop_everything() {
    MultiTracker m;
    for (int i = 0; i < 3; i++) m.update(C({{100, 100}, {300, 200}}));
    std::vector<TrackedSite> s;
    for (int i = 0; i < 5; i++) s = m.update(Cands());
    CHECK(s.empty());
}

static void test_multi_far_candidate_does_not_steal_a_track() {
    MultiTracker m;
    std::vector<TrackedSite> s;
    for (int i = 0; i < 3; i++) s = m.update(C({{100, 100}}));
    const int id = s[0].id;
    s = m.update(C({{100, 100}, {400, 400}}));
    CHECK(near_site(s, 100, 100)->id == id);
    CHECK(near_site(s, 400, 400)->id != id);
}

int main() {
    struct T { const char* n; void (*f)(); } tests[] = {
        {"centre_pixel_45deg_down", test_centre_pixel_45deg_down},
        {"horizontal_ray_never_hits_ground", test_horizontal_ray_never_hits_ground},
        {"ray_above_horizon_rejected", test_ray_above_horizon_rejected},
        {"invalid_mount_rejected", test_invalid_mount_rejected},
        {"distance_matches_independent_formula", test_distance_matches_independent_formula},
        {"lateral_sign_and_bearing", test_lateral_sign_and_bearing},
        {"bearing_without_mount", test_bearing_without_mount},
        {"scale_intrinsics", test_scale_intrinsics},
        {"tracker_confirms_after_n_hits_once", test_tracker_confirms_after_n_hits_once},
        {"tracker_single_frame_blip_is_ignored", test_tracker_single_frame_blip_is_ignored},
        {"tracker_coasts_then_drops", test_tracker_coasts_then_drops},
        {"tracker_recovers_after_short_gap", test_tracker_recovers_after_short_gap},
        {"tracker_far_candidate_does_not_hijack", test_tracker_far_candidate_does_not_hijack},
        {"tracker_switches_to_new_site_after_lost", test_tracker_switches_to_new_site_after_lost},
        {"tracker_smooths_centroid", test_tracker_smooths_centroid},
        {"multi_confirms_each_site_with_distinct_ids", test_multi_confirms_each_site_with_distinct_ids},
        {"multi_ids_follow_position_not_list_order", test_multi_ids_follow_position_not_list_order},
        {"multi_one_site_flickers_others_stay_confirmed", test_multi_one_site_flickers_others_stay_confirmed},
        {"multi_capacity_limit", test_multi_capacity_limit},
        {"multi_lost_track_frees_slot_for_new_site", test_multi_lost_track_frees_slot_for_new_site},
        {"multi_empty_frames_drop_everything", test_multi_empty_frames_drop_everything},
        {"multi_far_candidate_does_not_steal_a_track", test_multi_far_candidate_does_not_steal_a_track},
    };
    int bad = 0;
    for (auto& t : tests) {
        int before = g_failed; t.f();
        bool ok = (g_failed == before);
        std::printf("[%s] %s\n", ok ? " OK " : "FAIL", t.n);
        if (!ok) bad++;
    }
    std::printf("\n%zu tests, %d failed, %d checks\n", sizeof(tests) / sizeof(tests[0]), bad, g_checks);
    return bad ? 1 : 0;
}
