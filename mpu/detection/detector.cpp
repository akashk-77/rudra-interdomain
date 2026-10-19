// detector.cpp - see detector.hpp. Mirrors the algorithm prototyped in Python/cv2.
#include "detector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace {

int makeOdd(int k) { return std::max(1, k) | 1; }

// Replace m by its robust z-score (median/MAD), then clamp negatives to 0.
void robustZ(cv::Mat& m, float minScale) {
    CV_Assert(m.type() == CV_32FC1);
    std::vector<float> v(m.begin<float>(), m.end<float>());
    if (v.empty()) return;
    auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
    std::nth_element(v.begin(), mid, v.end());
    const float med = *mid;
    for (float& x : v) x = std::fabs(x - med);
    mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
    std::nth_element(v.begin(), mid, v.end());
    const float scale = std::max(1.4826f * (*mid), minScale);
    m.convertTo(m, CV_32F, 1.0 / scale, -static_cast<double>(med) / scale);
    cv::threshold(m, m, 0.0, 0.0, cv::THRESH_TOZERO);   // keep only values > 0
}

}  // namespace

DetectResult AnomalyDetector::detect(const cv::Mat& bgr, cv::Mat* debugScore) const {
    DetectResult res;
    CV_Assert(!bgr.empty() && bgr.type() == CV_8UC3);

    // 1. Working resolution.
    const double scale = std::min(1.0, static_cast<double>(cfg_.work_width) / bgr.cols);
    cv::Mat small;
    if (scale < 1.0) cv::resize(bgr, small, cv::Size(), scale, scale, cv::INTER_AREA);
    else small = bgr;
    res.scale = scale;
    const int H = small.rows, W = small.cols;

    // 2. Searched region (drops sky above the horizon and the rover body).
    const int top = static_cast<int>(std::lround(cfg_.roi_top_frac * H));
    const int bottom = H - static_cast<int>(std::lround(cfg_.roi_bottom_frac * H));
    if (bottom - top < 16) return res;
    res.roi_top = top;
    const cv::Mat crop = small(cv::Rect(0, top, W, bottom - top)).clone();

    // 3. Colour/brightness deviation from the local surroundings (Lab space).
    const int k = makeOdd(cfg_.blur_k);
    cv::Mat blurred, lab8, lab, bg;
    cv::GaussianBlur(crop, blurred, cv::Size(k, k), 0);
    cv::cvtColor(blurred, lab8, cv::COLOR_BGR2Lab);
    lab8.convertTo(lab, CV_32FC3);
    const double sigma = static_cast<double>(cfg_.bg_sigma_frac) * W;
    cv::GaussianBlur(lab, bg, cv::Size(0, 0), sigma);
    const cv::Mat diff = lab - bg;
    std::vector<cv::Mat> ch;
    cv::split(diff, ch);
    cv::Mat color, lum;
    cv::magnitude(ch[1], ch[2], color);                 // chroma deviation
    lum = cv::abs(ch[0]);                               // brightness deviation (weaker)
    cv::addWeighted(color, 1.0, lum, 0.5, 0.0, color);

    // 4. Texture deviation: local edge energy vs. its surroundings.
    cv::Mat gray8, gray, gx, gy, mag, tex, texBg, texd;
    cv::cvtColor(crop, gray8, cv::COLOR_BGR2GRAY);
    gray8.convertTo(gray, CV_32F);
    cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
    cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
    cv::magnitude(gx, gy, mag);
    cv::GaussianBlur(mag, tex, cv::Size(0, 0), static_cast<double>(cfg_.tex_sigma_frac) * W);
    cv::GaussianBlur(tex, texBg, cv::Size(0, 0), sigma);
    cv::absdiff(tex, texBg, texd);

    // 5. Robust z-scores, combined with max() so "any" kind of irregularity counts.
    robustZ(color, cfg_.min_scale);
    robustZ(texd, cfg_.min_scale);
    color.convertTo(color, CV_32F, cfg_.w_color);
    texd.convertTo(texd, CV_32F, cfg_.w_texture);
    cv::Mat score;
    cv::max(color, texd, score);
    if (debugScore) score.convertTo(*debugScore, CV_8U, 255.0 / (3.0 * cfg_.z_thresh));

    // 6. Threshold, clean up, pick multiple compact blobs.
    cv::Mat bin;
    cv::compare(score, static_cast<double>(cfg_.z_thresh), bin, cv::CMP_GT);
    const int ko = makeOdd(cfg_.morph_open), kc = makeOdd(cfg_.morph_close);
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ko, ko)));
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kc, kc)));

    cv::Mat labels, stats, cents;
    const int n = cv::connectedComponentsWithStats(bin, labels, stats, cents, 8, CV_32S);
    const double roiPx = static_cast<double>(score.total());
    
    std::vector<ScoredCandidate> all_candidates;
    for (int i = 1; i < n; ++i) {
        const double af = stats.at<int>(i, cv::CC_STAT_AREA) / roiPx;
        const cv::Rect blobBox(stats.at<int>(i, cv::CC_STAT_LEFT),
                               stats.at<int>(i, cv::CC_STAT_TOP) + top,
                               stats.at<int>(i, cv::CC_STAT_WIDTH),
                               stats.at<int>(i, cv::CC_STAT_HEIGHT));
        if (af < cfg_.min_area_frac) {                       // remember near-misses for tuning
            if (af >= cfg_.min_area_frac * 0.25) res.rejected_small.push_back(blobBox);
            continue;
        }
        if (af > cfg_.max_area_frac) { res.rejected_large.push_back(blobBox); continue; }
        const cv::Mat mask = (labels == i);
        const double mz = cv::mean(score, mask)[0];
        const double rank = mz * std::sqrt(af);

        ScoredCandidate sc;
        sc.rank = rank;
        sc.candidate.centroid = { static_cast<float>(cents.at<double>(i, 0)),
                                  static_cast<float>(cents.at<double>(i, 1) + top) };
        sc.candidate.area_frac = static_cast<float>(af);
        sc.candidate.mean_z = static_cast<float>(mz);
        sc.box = cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                          stats.at<int>(i, cv::CC_STAT_TOP) + top,
                          stats.at<int>(i, cv::CC_STAT_WIDTH),
                          stats.at<int>(i, cv::CC_STAT_HEIGHT));
        all_candidates.push_back(sc);
    }

    // Sort candidates by rank descending (highest score first)
    std::sort(all_candidates.begin(), all_candidates.end(), [](const ScoredCandidate& a, const ScoredCandidate& b) {
        return a.rank > b.rank;
    });

    // Keep up to max_candidates (default 3)
    if (all_candidates.size() > static_cast<size_t>(cfg_.max_candidates)) {
        all_candidates.resize(cfg_.max_candidates);
    }
    res.candidates = all_candidates;

    return res;
}
