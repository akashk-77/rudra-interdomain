// bubble_detect.cpp
// Standalone bubble detector for peroxide tests on soil samples in clear test tubes.
// Works on a SINGLE image. Printed text / scale ticks on the tube are detected and ignored.
//
// Build:
//   g++ -O2 -std=c++17 bubble_detect.cpp -o bubble_detect $(pkg-config --cflags --libs opencv4)
//
// Usage:
//   ./bubble_detect image.jpg [more.jpg ...] [options]
//
// Options:
//   --roi x,y,w,h     Region of interest in original image pixels. Put it INSIDE the tube walls,
//                     from the liquid surface down to just above the soil.
//   --select          Choose the ROI with the mouse on the first image (needs a display).
//   --out dir         Folder for overlay images (default: bubble_out)
//   --zoom F          Overlay magnification (default 1.0)
//   --contrast F      Minimum bubble contrast (default 1.2). Lower = more bubbles, more false hits.
//   --min-r N         Smallest bubble radius in px (default 8)
//   --max-r N         Largest bubble radius in px (default 65)
//   --mark-dark N     Gray level below which thin strokes count as printed marks (default 85).
//                     0 disables mark removal.
//   --mark-k N        Stroke-width kernel for marks (default 17). Raise if your camera resolution
//                     is higher and the tick marks are thicker than ~15 px.
//   --json            One JSON object per image instead of a text line
//   --show            Show the overlay window (needs a display)
//
// How it works:
//   1. Thin dark strokes (text, ticks) are found with a black-hat filter and inpainted away.
//   2. A "bubble-ness" map is built: bubble interiors are darker and bluer (less yellow) than the
//      surrounding film. Local lighting trends are subtracted.
//   3. A multi-scale Laplacian-of-Gaussian finds round blobs of any size, including overlapping ones.
//   4. Each candidate is validated with 36 rays: a real bubble has a closed boundary at a consistent
//      radius all around it. Tube walls, smudges and inpainted marks fail this test.
//   5. Overlaps are resolved, coverage and a fizz score are computed.
//
// Overlay: green circles = bubbles, blue tint = printed marks that were ignored.

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace cv;
using namespace std;

// =============================================================================
//                           DETECTOR (reusable part)
// =============================================================================

struct Config {
    int    maxWorkH     = 700;    // ROI taller than this is shrunk to it (never enlarged)
    double minR         = 8.0;    // bubble radius limits, px in working image
    double maxR         = 65.0;
    double minContrast  = 1.2;    // inside-vs-surroundings contrast of the bubble-ness map
    double minSupport   = 0.70;   // fraction of rays that must find the bubble boundary
    double maxSpread    = 0.35;   // max relative spread of boundary radius (roundness)
    double maxMarkFrac  = 0.10;   // max fraction of a bubble allowed to overlap removed marks
    double logThresh    = 0.30;   // LoG response threshold for candidates

    int    markDark     = 85;     // printed marks: gray below this ...
    int    markBh       = 30;     // ... and black-hat response above this ...
    int    markKernel   = 17;     // ... with stroke width below about this
    int    markDilate   = 9;

    double coverageFull = 0.25;   // fraction of ROI covered that counts as "full"
    double countFull    = 15.0;   // bubble count that counts as "full"

    double zoom         = 1.0;    // overlay magnification
};

struct Bubble {
    float  x, y, r;               // in working-image pixels
    double contrast;
};

struct BubbleResult {
    vector<Bubble> bubbles;
    int    count    = 0;
    double coverage = 0.0;        // fraction of ROI covered by bubbles (0..1)
    double score    = 0.0;        // 0..1
    string label    = "NONE";     // NONE / WEAK / MODERATE / STRONG
    Mat    overlay;               // annotated ROI (BGR)
};

static double clamp01(double v) { return max(0.0, min(1.0, v)); }

static string labelFor(double score) {
    if (score < 0.15) return "NONE";
    if (score < 0.40) return "WEAK";
    if (score < 0.70) return "MODERATE";
    return "STRONG";
}

static inline float sampleBilinear(const Mat& m, float x, float y) {
    x = min(max(x, 0.f), (float)(m.cols - 1));
    y = min(max(y, 0.f), (float)(m.rows - 1));
    int x0 = (int)floor(x), y0 = (int)floor(y);
    int x1 = min(x0 + 1, m.cols - 1), y1 = min(y0 + 1, m.rows - 1);
    float fx = x - x0, fy = y - y0;
    float a = m.at<float>(y0, x0), b = m.at<float>(y0, x1);
    float c = m.at<float>(y1, x0), d = m.at<float>(y1, x1);
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
}

// Step 1: find thin dark strokes (text, scale ticks), paint them out.
static Mat removeMarks(const Mat& bgr, const Config& cfg, Mat& marks) {
    marks = Mat::zeros(bgr.size(), CV_8UC1);
    if (cfg.markDark <= 0) return bgr.clone();

    Mat gray;
    cvtColor(bgr, gray, COLOR_BGR2GRAY);

    int k = cfg.markKernel | 1;
    Mat bh;
    morphologyEx(gray, bh, MORPH_BLACKHAT, getStructuringElement(MORPH_ELLIPSE, Size(k, k)));

    Mat dark = gray < cfg.markDark;
    Mat thin = bh > cfg.markBh;
    bitwise_and(dark, thin, marks);

    int d = cfg.markDilate | 1;
    dilate(marks, marks, getStructuringElement(MORPH_ELLIPSE, Size(d, d)));

    if (countNonZero(marks) == 0) return bgr.clone();
    Mat clean;
    inpaint(bgr, marks, clean, 5, INPAINT_TELEA);
    return clean;
}

// Step 2: "bubble-ness" map. High where the image is darker and less yellow than its surroundings.
static Mat buildFeature(const Mat& clean) {
    Mat lab;
    cvtColor(clean, lab, COLOR_BGR2Lab);
    vector<Mat> ch;
    split(lab, ch);

    Mat L, B;
    ch[0].convertTo(L, CV_32F);
    ch[2].convertTo(B, CV_32F);
    GaussianBlur(L, L, Size(0, 0), 1.5);
    GaussianBlur(B, B, Size(0, 0), 3.0);

    auto zscore = [](Mat& m) {
        Scalar mu, sd;
        meanStdDev(m, mu, sd);
        m = (m - mu[0]) / (sd[0] + 1e-6);
    };
    zscore(L);
    zscore(B);

    Mat F = -0.5 * L - B;
    Mat base;
    GaussianBlur(F, base, Size(0, 0), 40.0);   // remove slow lighting / colour gradients
    F = F - base;
    return F;
}

struct Analysis {
    double contrast = 0, support = 0, spread = 9, radius = 0;
};

// Step 4: ray-based validation of one candidate circle.
static bool analyseCandidate(const Mat& F, float cx, float cy, float r, Analysis& out) {
    const int NA = 36;
    vector<float> ts;
    for (float t = 0.3f * r; t < 1.9f * r; t += 1.0f) ts.push_back(t);
    const int T = (int)ts.size();
    if (T < 4) return false;

    vector<float> P((size_t)NA * T);
    double finSum = 0, ringSum = 0;
    int finN = 0, ringN = 0;

    for (int a = 0; a < NA; ++a) {
        double ang = 2.0 * CV_PI * a / NA;
        float ca = (float)cos(ang), sa = (float)sin(ang);
        for (int i = 0; i < T; ++i) {
            float v = sampleBilinear(F, cx + ts[i] * ca, cy + ts[i] * sa);
            P[(size_t)a * T + i] = v;
            if (ts[i] <= 0.6f * r) { finSum += v; ++finN; }
            if (ts[i] >= 1.25f * r && ts[i] <= 1.7f * r) { ringSum += v; ++ringN; }
        }
    }
    if (finN == 0 || ringN == 0) return false;

    double fin = finSum / finN, ring = ringSum / ringN;
    out.contrast = fin - ring;
    double mid = 0.5 * (fin + ring);

    vector<double> found;
    for (int a = 0; a < NA; ++a) {
        for (int i = 0; i < T; ++i) {
            if (ts[i] < 0.5f * r) continue;
            if (P[(size_t)a * T + i] < mid) { found.push_back(ts[i]); break; }
        }
    }
    out.support = (double)found.size() / NA;
    if (found.size() < 8) { out.spread = 9; out.radius = r; return true; }

    vector<double> sorted = found;
    sort(sorted.begin(), sorted.end());
    size_t n = sorted.size();
    double med = (n % 2) ? sorted[n / 2] : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);

    double mean = 0;
    for (double v : found) mean += v;
    mean /= n;
    double var = 0;
    for (double v : found) var += (v - mean) * (v - mean);
    var /= n;

    out.radius = med;
    out.spread = sqrt(var) / med;
    return true;
}

struct Cand { float resp; int x, y; float r; };

// Main entry point.
//   bgr : full image (BGR)
//   roi : region in original pixels (empty Rect = whole image)
BubbleResult detectBubbles(const Mat& bgr, Rect roi, const Config& cfg = Config()) {
    BubbleResult res;
    CV_Assert(!bgr.empty());

    Rect full(0, 0, bgr.cols, bgr.rows);
    if (roi.area() <= 0) roi = full;
    roi &= full;
    CV_Assert(roi.width > 20 && roi.height > 20);

    // Crop; shrink only if very tall, never enlarge
    Mat raw = bgr(roi).clone();
    if (cfg.maxWorkH > 0 && raw.rows > cfg.maxWorkH) {
        double s = (double)cfg.maxWorkH / raw.rows;
        resize(raw, raw, Size(), s, s, INTER_AREA);
    }
    const int H = raw.rows, W = raw.cols;

    Mat marks;
    Mat clean = removeMarks(raw, cfg, marks);
    Mat F = buildFeature(clean);

    // ---- Step 3: multi-scale LoG candidates ----
    const vector<double> sigmas = {5, 6.5, 8, 10, 12.5, 15, 18, 22, 27, 33};
    const int NS = (int)sigmas.size();
    vector<Mat> stack(NS), mx(NS);
    for (int i = 0; i < NS; ++i) {
        Mat G, lap;
        GaussianBlur(F, G, Size(0, 0), sigmas[i]);
        Laplacian(G, lap, CV_32F, 3);
        stack[i] = lap * (-(float)(sigmas[i] * sigmas[i]));
        dilate(stack[i], mx[i], Mat::ones(9, 9, CV_8U));
    }

    vector<Cand> cands;
    for (int si = 0; si < NS; ++si) {
        int s0 = max(0, si - 1), s1 = min(NS - 1, si + 1);
        for (int y = 0; y < H; ++y) {
            const float* cur = stack[si].ptr<float>(y);
            const float* cmx = mx[si].ptr<float>(y);
            for (int x = 0; x < W; ++x) {
                float v = cur[x];
                if (v <= (float)cfg.logThresh) continue;
                if (v < cmx[x] - 1e-6f) continue;
                bool isMax = true;
                for (int sj = s0; sj <= s1; ++sj)
                    if (stack[sj].at<float>(y, x) > v + 1e-6f) { isMax = false; break; }
                if (!isMax) continue;
                cands.push_back({v, x, y, (float)(sigmas[si] * sqrt(2.0))});
            }
        }
    }

    // ---- Step 4: validate ----
    struct Good { double contrast, radius; int x, y; };
    vector<Good> good;
    for (const Cand& c : cands) {
        if (c.r < (float)cfg.minR) continue;

        Analysis an;
        if (!analyseCandidate(F, (float)c.x, (float)c.y, c.r, an)) continue;
        double rr = an.radius;

        if (an.contrast < cfg.minContrast) continue;
        if (an.support < cfg.minSupport) continue;
        if (an.spread > cfg.maxSpread) continue;
        if (rr < cfg.minR || rr > cfg.maxR) continue;

        // The circle must lie (mostly) inside the ROI
        if (c.x - 0.5 * rr < 0 || c.x + 0.5 * rr > W || c.y - 0.5 * rr < 0 || c.y + 0.5 * rr > H) continue;

        // Reject anything sitting on removed printed marks (inpainting leaves artefacts there)
        int x0 = max(0, (int)floor(c.x - rr)), x1 = min(W - 1, (int)ceil(c.x + rr));
        int y0 = max(0, (int)floor(c.y - rr)), y1 = min(H - 1, (int)ceil(c.y + rr));
        long tot = 0, onMark = 0;
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                double dx = x - c.x, dy = y - c.y;
                if (dx * dx + dy * dy <= rr * rr) {
                    ++tot;
                    if (marks.at<uchar>(y, x)) ++onMark;
                }
            }
        if (tot == 0 || (double)onMark / tot >= cfg.maxMarkFrac) continue;

        good.push_back({an.contrast, rr, c.x, c.y});
    }

    // ---- Step 5: overlap resolution (prefer high contrast, favour larger bubbles) ----
    sort(good.begin(), good.end(), [](const Good& a, const Good& b) {
        return a.contrast * sqrt(a.radius) > b.contrast * sqrt(b.radius);
    });
    for (const Good& g : good) {
        bool dup = false;
        for (const Bubble& k : res.bubbles)
            if (hypot(g.x - k.x, g.y - k.y) < 0.8 * max(g.radius, (double)k.r)) { dup = true; break; }
        if (!dup) res.bubbles.push_back({(float)g.x, (float)g.y, (float)g.radius, g.contrast});
    }
    res.count = (int)res.bubbles.size();

    Mat cov = Mat::zeros(H, W, CV_8UC1);
    for (const Bubble& b : res.bubbles) circle(cov, Point(cvRound(b.x), cvRound(b.y)), cvRound(b.r), Scalar(255), FILLED);
    res.coverage = (double)countNonZero(cov) / ((double)H * W);

    res.score = 0.6 * clamp01(res.coverage / cfg.coverageFull) + 0.4 * clamp01(res.count / cfg.countFull);
    res.label = labelFor(res.score);

    // ---- Overlay on the ORIGINAL (not inpainted) crop ----
    const double Z = cfg.zoom > 0 ? cfg.zoom : 1.0;
    Mat ov;
    resize(raw, ov, Size(), Z, Z, Z >= 1.0 ? INTER_LINEAR : INTER_AREA);

    if (countNonZero(marks) > 0) {
        Mat mBig;
        resize(marks, mBig, ov.size(), 0, 0, INTER_NEAREST);
        Mat tint = ov.clone();
        tint.setTo(Scalar(255, 120, 0), mBig);
        addWeighted(tint, 0.35, ov, 0.65, 0, ov);
    }
    int idx = 1;
    for (const Bubble& b : res.bubbles) {
        Point c(cvRound(b.x * Z), cvRound(b.y * Z));
        circle(ov, c, cvRound(b.r * Z), Scalar(0, 255, 0), 2, LINE_AA);
        putText(ov, to_string(idx++), Point(c.x - 6, c.y + 5), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(0, 0, 0), 3, LINE_AA);
        putText(ov, to_string(idx - 1), Point(c.x - 6, c.y + 5), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255, 255, 255), 1, LINE_AA);
    }
    string txt = res.label + " " + format("%.2f", res.score) + "  bubbles=" + to_string(res.count);
    putText(ov, txt, Point(6, 20), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0, 0, 0), 4, LINE_AA);
    putText(ov, txt, Point(6, 20), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(255, 255, 255), 1, LINE_AA);
    res.overlay = ov;

    return res;
}

// =============================================================================
//                              TEST HARNESS
// =============================================================================

static Rect parseRect(const string& s) {
    int x, y, w, h;
    if (sscanf(s.c_str(), "%d,%d,%d,%d", &x, &y, &w, &h) != 4) {
        cerr << "Bad --roi, expected x,y,w,h\n";
        exit(2);
    }
    return Rect(x, y, w, h);
}

static Rect selectRoiInteractive(const Mat& img) {
    double s = min(1.0, 900.0 / img.rows);
    Mat preview;
    resize(img, preview, Size(), s, s, INTER_AREA);
    Rect r = selectROI("Select region INSIDE the tube (liquid surface down to soil), press ENTER", preview, true);
    destroyAllWindows();
    if (r.area() <= 0) return Rect();
    return Rect(cvRound(r.x / s), cvRound(r.y / s), cvRound(r.width / s), cvRound(r.height / s));
}

static string jsonFor(const string& file, const BubbleResult& r) {
    ostringstream o;
    o << "{\"file\":\"" << file << "\","
      << "\"label\":\"" << r.label << "\","
      << "\"score\":" << format("%.3f", r.score) << ","
      << "\"count\":" << r.count << ","
      << "\"coverage\":" << format("%.3f", r.coverage) << ","
      << "\"bubbles\":[";
    for (size_t i = 0; i < r.bubbles.size(); ++i) {
        if (i) o << ",";
        o << "{\"x\":" << format("%.1f", r.bubbles[i].x) << ",\"y\":" << format("%.1f", r.bubbles[i].y)
          << ",\"r\":" << format("%.1f", r.bubbles[i].r) << "}";
    }
    o << "]}";
    return o.str();
}

int main(int argc, char** argv) {
    vector<string> images;
    string outDir = "bubble_out";
    Rect roi;
    bool select = false, asJson = false, show = false;
    Config cfg;

    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        auto next = [&](const char* name) -> string {
            if (i + 1 >= argc) { cerr << "Missing value for " << name << "\n"; exit(2); }
            return argv[++i];
        };
        if      (a == "--roi")       roi = parseRect(next("--roi"));
        else if (a == "--select")    select = true;
        else if (a == "--out")       outDir = next("--out");
        else if (a == "--zoom")      cfg.zoom = stod(next("--zoom"));
        else if (a == "--contrast")  cfg.minContrast = stod(next("--contrast"));
        else if (a == "--min-r")     cfg.minR = stod(next("--min-r"));
        else if (a == "--max-r")     cfg.maxR = stod(next("--max-r"));
        else if (a == "--mark-dark") cfg.markDark = stoi(next("--mark-dark"));
        else if (a == "--mark-k")    cfg.markKernel = stoi(next("--mark-k"));
        else if (a == "--json")      asJson = true;
        else if (a == "--show")      show = true;
        else if (a.rfind("--", 0) == 0) { cerr << "Unknown option: " << a << "\n"; return 2; }
        else images.push_back(a);
    }

    if (images.empty()) {
        cerr << "Usage: " << argv[0] << " image.jpg [more.jpg ...] [--roi x,y,w,h] [--select] [--out dir] "
             << "[--zoom F] [--contrast F] [--min-r N] [--max-r N] [--mark-dark N] [--mark-k N] [--json] [--show]\n";
        return 2;
    }

    error_code ec;
    fs::create_directories(outDir, ec);

    bool first = true;
    for (const string& path : images) {
        Mat img = imread(path);
        if (img.empty()) { cerr << "Cannot read image: " << path << "\n"; continue; }

        if (first && select) {
            Rect r = selectRoiInteractive(img);
            if (r.area() > 0) {
                roi = r;
                cerr << "Selected ROI: --roi " << roi.x << "," << roi.y << "," << roi.width << "," << roi.height << "\n";
            }
        }
        first = false;

        BubbleResult r;
        try {
            r = detectBubbles(img, roi, cfg);
        } catch (const cv::Exception& e) {
            cerr << "Error on " << path << ": " << e.what() << "\n";
            continue;
        }

        string outPath = (fs::path(outDir) / (fs::path(path).stem().string() + "_overlay.png")).string();
        imwrite(outPath, r.overlay);
        cerr << "Saved overlay: " << outPath << "\n";

        if (asJson) {
            cout << jsonFor(path, r) << endl;
        } else {
            printf("%-24s %-8s score=%.2f  bubbles=%d  coverage=%.1f%%\n",
                   path.c_str(), r.label.c_str(), r.score, r.count, r.coverage * 100.0);
        }

        if (show) {
            imshow("bubbles", r.overlay);
            waitKey(0);
            destroyAllWindows();
        }
    }
    return 0;
}
