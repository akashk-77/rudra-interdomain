#pragma once
#include <opencv2/core.hpp>
#include <optional>

#include "tracker.hpp"

struct DetectorConfig {            
    int   work_width      = 640;   // frames wider than this are downscaled first
    int   blur_k          = 5;
    int   max_candidates  = 3;
    float bg_sigma_frac   = 0.12f; 
    float tex_sigma_frac  = 0.012f;
    float roi_top_frac    = 0.30f; // ignore this fraction of rows at the top (sky)
    float roi_bottom_frac = 0.0f;  // ignore this fraction at the bottom (rover body)
    float w_color         = 1.0f;
    float w_texture       = 1.0f;
    float z_thresh        = 6.0f;  
    float min_scale       = 1.0f;  
    int   morph_open      = 3;     
    int   morph_close     = 9;     
    float min_area_frac   = 0.003f;
    float max_area_frac   = 0.35f; 
};

struct ScoredCandidate {
    Candidate candidate;
    cv::Rect  box;
    double    rank = 0.0;
};

struct DetectResult {
    std::vector<ScoredCandidate> candidates; 
    cv::Rect box;                  
    double   scale = 1.0;          
    int      roi_top = 0;          
};

class AnomalyDetector {
public:
    explicit AnomalyDetector(const DetectorConfig& cfg = {}) : cfg_(cfg) {}
    DetectResult detect(const cv::Mat& bgr, cv::Mat* debug_score = nullptr) const;
    const DetectorConfig& config() const { return cfg_; }
private:
    DetectorConfig cfg_;
};
