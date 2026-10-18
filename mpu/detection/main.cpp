#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "detector.hpp"
#include "geometry.hpp"
#include "tracker.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;
void onSignal(int) { g_running = 0; }

struct Options {
    std::string config, device, video, image;
    int  camera = -1;           // -1 = not given on the command line
    bool show = false, noEvidence = false;
};

struct AppConfig {
    DetectorConfig det;
    TrackerConfig  trk;
    geom::Intrinsics k;
    cv::Mat camMatrix, distCoeffs;
    int calibW = 0, calibH = 0;
    geom::Mount mount;
    double maxFps = 10.0;
    int cameraW = 0, cameraH = 0;
    int cameraIndex = -1;             // from config; -1 = not set
    std::string cameraDevice;         
    std::string evidenceDir = "evidence";
    int saveEvidence = 1;
};

template <typename T>
void readOpt(const cv::FileStorage& fs, const char* key, T& out) {
    const cv::FileNode n = fs[key];
    if (!n.empty()) n >> out;
}

bool loadConfig(const std::string& path, AppConfig& c) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    readOpt(fs, "work_width", c.det.work_width);
    readOpt(fs, "blur_k", c.det.blur_k);
    readOpt(fs, "bg_sigma_frac", c.det.bg_sigma_frac);
    readOpt(fs, "tex_sigma_frac", c.det.tex_sigma_frac);
    readOpt(fs, "roi_top_frac", c.det.roi_top_frac);
    readOpt(fs, "roi_bottom_frac", c.det.roi_bottom_frac);
    readOpt(fs, "w_color", c.det.w_color);
    readOpt(fs, "w_texture", c.det.w_texture);
    readOpt(fs, "z_thresh", c.det.z_thresh);
    readOpt(fs, "min_scale", c.det.min_scale);
    readOpt(fs, "morph_open", c.det.morph_open);
    readOpt(fs, "morph_close", c.det.morph_close);
    readOpt(fs, "min_area_frac", c.det.min_area_frac);
    readOpt(fs, "max_area_frac", c.det.max_area_frac);
    readOpt(fs, "confirm_frames", c.trk.confirm_frames);
    readOpt(fs, "lost_frames", c.trk.lost_frames);
    readOpt(fs, "gate_px", c.trk.gate_px);
    readOpt(fs, "ema_alpha", c.trk.ema_alpha);
    readOpt(fs, "max_fps", c.maxFps);
    readOpt(fs, "camera_width", c.cameraW);
    readOpt(fs, "camera_height", c.cameraH);
    readOpt(fs, "camera_index", c.cameraIndex);
    readOpt(fs, "camera_device", c.cameraDevice);
    readOpt(fs, "evidence_dir", c.evidenceDir);
    readOpt(fs, "save_evidence", c.saveEvidence);

    cv::Mat K, D;
    readOpt(fs, "camera_matrix", K);
    readOpt(fs, "dist_coeffs", D);
    readOpt(fs, "calib_width", c.calibW);
    readOpt(fs, "calib_height", c.calibH);
    if (K.rows == 3 && K.cols == 3) {
        K.convertTo(K, CV_64F);
        c.camMatrix = K;
        c.k.fx = K.at<double>(0, 0); c.k.fy = K.at<double>(1, 1);
        c.k.cx = K.at<double>(0, 2); c.k.cy = K.at<double>(1, 2);
        c.k.valid = c.k.fx > 0 && c.k.fy > 0;
        if (!D.empty()) { D.convertTo(D, CV_64F); c.distCoeffs = D; }
    }
    if (!fs["camera_height_m"].empty() && !fs["camera_pitch_deg"].empty()) {
        readOpt(fs, "camera_height_m", c.mount.height_m);
        readOpt(fs, "camera_pitch_deg", c.mount.pitch_deg);
        c.mount.valid = c.mount.height_m > 0.0;
    }
    return true;
}

struct Pose {
    int calib = 0;
    double xNorm = 0, bearing = 0, dist = 0;
    bool bearingOk = false, distOk = false;
};

Pose computePose(const AppConfig& c, double u, double v, const cv::Size& frame) {
    Pose p;
    p.xNorm = (u - frame.width / 2.0) / (frame.width / 2.0);   // needs no calibration
    if (!c.k.valid) return p;
    p.calib = 1;
    geom::Intrinsics k = c.k;
    if (c.calibW > 0 && c.calibH > 0)
        k = geom::scale_intrinsics(k, static_cast<double>(frame.width) / c.calibW,
                                   static_cast<double>(frame.height) / c.calibH);
    const cv::Mat K = (cv::Mat_<double>(3, 3) << k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1);
    std::vector<cv::Point2d> src{cv::Point2d(u, v)}, dst;
    cv::undistortPoints(src, dst, K, c.distCoeffs);              // -> normalised coordinates
    const double xn = dst[0].x, yn = dst[0].y;
    p.bearing = geom::bearing_deg(xn, yn, c.mount.valid ? c.mount.pitch_deg : 0.0);
    p.bearingOk = true;
    if (c.mount.valid) {
        p.calib = 2;
        const geom::GroundPoint g = geom::ground_from_normalized(xn, yn, c.mount);
        if (g.on_ground) { p.dist = g.distance_m; p.distOk = true; }
    }
    return p;
}

std::string num(bool ok, double v, const char* fmt) {
    if (!ok) return "na";
    char buf[32];
    std::snprintf(buf, sizeof(buf), fmt, v);
    return buf;
}

const char* stateName(TrackStatus s) {
    switch (s) {
        case TrackStatus::Searching: return "SEARCHING";
        case TrackStatus::Tracking:  return "TRACKING";
        case TrackStatus::Confirmed: return "CONFIRMED";
        case TrackStatus::Coasting:  return "COASTING";
    }
    return "SEARCHING";
}

bool parseArgs(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](std::string& dst) { if (i + 1 >= argc) return false; dst = argv[++i]; return true; };
        std::string tmp;
        if (a == "--config")       { if (!next(o.config)) return false; }
        else if (a == "--device")  { if (!next(o.device)) return false; }
        else if (a == "--video")   { if (!next(o.video)) return false; }
        else if (a == "--image")   { if (!next(o.image)) return false; }
        else if (a == "--camera")  { if (!next(tmp)) return false; o.camera = std::atoi(tmp.c_str()); }
        else if (a == "--show")    o.show = true;
        else if (a == "--no-evidence") o.noEvidence = true;
        else return false;
    }
    return true;
}

void usage() {
    std::fprintf(stderr,
        "usage: site_detector [--config file.yaml] [--camera N | --device /dev/videoN |\n"
        "                      --video file | --image file] [--show] [--no-evidence]\n");
}

bool openCamera(cv::VideoCapture& cap, const Options& o, const AppConfig& c) {
    cap.release();
    std::string dev;
    int idx = 0;
    if (!o.device.empty())              dev = o.device;
    else if (o.camera >= 0)             idx = o.camera;
    else if (!c.cameraDevice.empty())   dev = c.cameraDevice;
    else if (c.cameraIndex >= 0)        idx = c.cameraIndex;
    bool ok = dev.empty() ? cap.open(idx, cv::CAP_V4L2) : cap.open(dev, cv::CAP_V4L2);
    if (!ok) ok = dev.empty() ? cap.open(idx) : cap.open(dev);
    if (!ok) return false;
    if (c.cameraW > 0) cap.set(cv::CAP_PROP_FRAME_WIDTH, c.cameraW);
    if (c.cameraH > 0) cap.set(cv::CAP_PROP_FRAME_HEIGHT, c.cameraH);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);          
    return true;
}

std::string saveEvidence(const AppConfig& c, const cv::Mat& frame, double u, double v,
                         const Pose& pose, uint64_t seq, long long tMs) {
    cv::Mat img = frame.clone();
    cv::circle(img, cv::Point(static_cast<int>(u), static_cast<int>(v)), 25, cv::Scalar(0, 255, 0), 3);
    char txt[96];
    std::snprintf(txt, sizeof(txt), "seq %llu  bearing %s deg  dist %s m",
                  static_cast<unsigned long long>(seq),
                  num(pose.bearingOk, pose.bearing, "%.1f").c_str(),
                  num(pose.distOk, pose.dist, "%.2f").c_str());
    cv::putText(img, txt, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    std::error_code ec;
    std::filesystem::create_directories(c.evidenceDir, ec);
    char name[64];
    std::snprintf(name, sizeof(name), "site_%06llu_%lldms.jpg",
                  static_cast<unsigned long long>(seq), tMs);
    const std::string path = (std::filesystem::path(c.evidenceDir) / name).string();
    return cv::imwrite(path, img) ? path : std::string();
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) { usage(); return 2; }
    AppConfig cfg;
    if (!opt.config.empty() && !loadConfig(opt.config, cfg)) {
        std::fprintf(stderr, "cannot read config '%s'\n", opt.config.c_str());
        return 2;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    const AnomalyDetector detector(cfg.det);
    SiteTracker tracker(cfg.trk);
    const bool imageMode = !opt.image.empty();
    const bool videoMode = !opt.video.empty();

    cv::VideoCapture cap;
    cv::Mat still;
    if (imageMode) {
        still = cv::imread(opt.image, cv::IMREAD_COLOR);
        if (still.empty()) { std::fprintf(stderr, "cannot read image '%s'\n", opt.image.c_str()); return 2; }
    } else if (videoMode) {
        if (!cap.open(opt.video)) { std::fprintf(stderr, "cannot open video '%s'\n", opt.video.c_str()); return 2; }
    } else if (!openCamera(cap, opt, cfg)) {
        std::fprintf(stderr, "camera not available yet; will keep retrying\n");
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, cfg.maxFps));
    auto nextTick = std::chrono::steady_clock::now();
    uint64_t seq = 0;
    int failures = 0;
    int stillRepeats = cfg.trk.confirm_frames + 2;      

    while (g_running) {
        const long long tMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - t0).count();
        cv::Mat frame;
        if (imageMode) {
            if (stillRepeats-- <= 0) break;
            frame = still;
        } else if (!cap.isOpened() || !cap.read(frame) || frame.empty()) {
            if (videoMode) break;                        // end of file
            std::printf("SITE v1 seq=%llu t_ms=%lld state=CAMERA_ERROR found=0 calib=0 "
                        "x_norm=na bearing_deg=na dist_m=na conf=0.00 area_frac=0.0000\n",
                        static_cast<unsigned long long>(seq++), tMs);
            if (++failures >= 3) {                       
                std::this_thread::sleep_for(std::chrono::seconds(1));
                openCamera(cap, opt, cfg);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            continue;
        }
        failures = 0;
        if (frame.channels() != 3) { cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR); }

        cv::Mat dbg;
        const DetectResult det = detector.detect(frame, opt.show ? &dbg : nullptr);
        
        // Feed primary (highest-ranked) candidate to tracker
        std::optional<Candidate> top_candidate = det.candidates.empty() ? std::nullopt : std::optional<Candidate>(det.candidates[0].candidate);
        const TrackOutput tr = tracker.update(top_candidate);

        // Print all detected candidates (up to 3)
        if (!det.candidates.empty()) {
            for (size_t i = 0; i < det.candidates.size(); ++i) {
                const auto& sc = det.candidates[i];
                const double u = sc.candidate.centroid.x / det.scale;
                const double v = sc.candidate.centroid.y / det.scale;
                const bool confirmedNow = (tr.status == TrackStatus::Confirmed && i == 0);
                Pose pose = computePose(cfg, u, v, frame.size());

                double conf = 0.0;
                if (confirmedNow) {
                    const double base = 1.0 - std::exp(-(tr.mean_z - cfg.det.z_thresh) / cfg.det.z_thresh);
                    conf = std::min(1.0, std::max(0.0, base)) *
                           std::min(1.0, tr.hits / (2.0 * std::max(1, cfg.trk.confirm_frames)));
                }

                std::printf("SITE v1 seq=%llu idx=%zu t_ms=%lld state=%s found=%d calib=%d x_norm=%s bearing_deg=%s "
                            "dist_m=%s conf=%.2f area_frac=%.4f\n",
                            static_cast<unsigned long long>(seq), i, tMs, stateName(tr.status),
                            confirmedNow ? 1 : 0, pose.calib,
                            num(confirmedNow, pose.xNorm, "%.3f").c_str(),
                            num(pose.bearingOk, pose.bearing, "%.1f").c_str(),
                            num(pose.distOk, pose.dist, "%.2f").c_str(), conf,
                            static_cast<double>(sc.candidate.area_frac));

                if (i == 0 && tr.newly_confirmed && cfg.saveEvidence && !opt.noEvidence) {
                    const std::string f = saveEvidence(cfg, frame, u, v, pose, seq, tMs);
                    if (!f.empty()) std::printf("EVIDENCE v1 seq=%llu file=%s\n",
                                                static_cast<unsigned long long>(seq), f.c_str());
                    else std::fprintf(stderr, "could not write evidence image\n");
                }
            }
        } else {
            std::printf("SITE v1 seq=%llu t_ms=%lld state=%s found=0 calib=0 x_norm=na bearing_deg=na dist_m=na conf=0.00 area_frac=0.0000\n",
                        static_cast<unsigned long long>(seq), tMs, stateName(tr.status));
        }

        if (opt.show) {
            cv::Mat view = frame.clone();
            for (const auto& sc : det.candidates) {
                cv::rectangle(view, cv::Rect(static_cast<int>(sc.box.x / det.scale),
                                           static_cast<int>(sc.box.y / det.scale),
                                           static_cast<int>(sc.box.width / det.scale),
                                           static_cast<int>(sc.box.height / det.scale)),
                              cv::Scalar(0, 255, 255), 2);
            }
            if (tr.status != TrackStatus::Searching && !det.candidates.empty()) {
                const auto& top_sc = det.candidates[0];
                cv::circle(view, cv::Point(static_cast<int>(top_sc.candidate.centroid.x / det.scale), 
                                           static_cast<int>(top_sc.candidate.centroid.y / det.scale)), 20,
                           (tr.status == TrackStatus::Confirmed) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
            }
            cv::imshow("site_detector", view);
            if (!dbg.empty()) cv::imshow("anomaly score", dbg);
            if ((cv::waitKey(1) & 0xFF) == 'q') break;
        }
        ++seq;

        if (!imageMode && !videoMode) {                  // rate limit live camera only
            nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(nextTick);
            if (nextTick < std::chrono::steady_clock::now()) nextTick = std::chrono::steady_clock::now();
        }
    }
    cap.release();                                       // release the camera on shutdown
    return 0;
}
