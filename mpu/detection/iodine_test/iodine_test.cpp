// iodine_test.cpp
// Standalone starch detector for the iodine test on soil samples in clear test tubes.
// Works on a SINGLE image.
//
//   Starch + iodine  -> blue / violet / blue-black      => "Iodine test positive"
//   No starch        -> iodine keeps its yellow / amber => "Iodine test negative"
//
// Build:
//   g++ -O2 -std=c++17 iodine_test.cpp -o iodine_test $(pkg-config --cflags --libs opencv4)
//
// Usage:
//   ./iodine_test image.jpg [more.jpg ...] [options]
//
// Options:
//   --roi x,y,w,h      Region of interest in original image pixels: the LIQUID in the tube.
//                      Keep the white background, label and finger out of it.
//   --select           Choose the ROI with the mouse on the first image (needs a display).
//   --white x,y,w,h    Optional white/grey reference patch (original pixels, e.g. a white card in
//                      view). Colours are normalised so the patch becomes neutral. Strongly
//                      recommended on the rover to cancel warm/cool lighting.
//   --out dir          Folder for overlay images (default: iodine_out)
//   --share F          Min share of violet/blue among coloured pixels to call POSITIVE (default 0.15)
//   --min-area F       Min violet/blue area as a fraction of the ROI (default 0.01)
//   --purple-lo N / --purple-hi N   Hue range (OpenCV 0..179) counted as starch colour (100..179)
//   --amber-lo N  / --amber-hi N    Hue range counted as unreacted iodine (5..40)
//   --min-sat F        Min colour strength (0..1, relative to brightness) for violet/blue pixels
//                      (default 0.20). Neutral soil/tube/background is below this.
//   --json             One JSON object per image instead of a text line
//   --show             Show the overlay window (needs a display)
//
// How it works:
//   1. Optional white balance from a reference patch, then light smoothing.
//   2. Every pixel is classified by hue: violet/blue (starch), yellow/amber (free iodine), other.
//      Colour strength is measured relative to brightness, so dark violet still counts while
//      neutral pixels (soil, tube, background, glare) fall in "other".
//   3. Small speckles are removed. The verdict uses the SHARE of violet/blue among all coloured
//      pixels, so it does not depend on how much liquid is in view.
//   4. The program prints one clear message:
//        Iodine test positive   (violet/blue found -> starch present)
//        Iodine test negative   (only yellow/amber iodine -> no starch)
//        Iodine test inconclusive (no iodine colour visible in the ROI)
//
// Overlay: magenta = starch colour, orange = free iodine colour.

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
    int    purpleLo        = 100;    // OpenCV hue range for blue / violet / magenta-black
    int    purpleHi        = 179;
    double purpleMinSat    = 0.20;   // colour strength relative to brightness (chroma / max channel)
    int    purpleMinV      = 15;     // ignore pixels too dark for a reliable hue
    int    amberLo         = 5;      // OpenCV hue range for yellow / orange / amber iodine
    int    amberHi         = 40;
    double amberMinSat     = 0.25;
    int    amberMinV       = 60;     // very dark brown is soil, not iodine

    double minShare        = 0.15;   // violet / (violet + amber) needed for POSITIVE
    double minAreaFrac     = 0.01;   // violet area as a fraction of the ROI needed for POSITIVE
    double minColoredFrac  = 0.01;   // below this much coloured area -> INCONCLUSIVE
    int    minBlobPx       = 80;     // speckle size removed from the masks

    bool   hasWhite        = false;  // optional white-reference patch (original image pixels)
    Rect   white;
};

struct IodineResult {
    string verdict  = "INCONCLUSIVE";  // POSITIVE / NEGATIVE / INCONCLUSIVE
    string message  = "Iodine test inconclusive";   // human-readable result
    double purpleFrac  = 0;            // violet/blue pixels / ROI pixels
    double amberFrac   = 0;            // yellow/amber pixels / ROI pixels
    double share       = 0;            // violet / (violet + amber)
    double purpleValue = 0;            // median brightness (V) of violet pixels, 0..255
    double purpleHue   = 0;            // median hue of violet pixels
    int    purplePx = 0, amberPx = 0, roiPx = 0;
    Mat    overlay;
};

// Scales the channels of 'img' so that the reference patch becomes neutral grey.
static void whiteBalance(Mat& img, const Mat& patch) {
    Scalar m = mean(patch);
    double g = (m[0] + m[1] + m[2]) / 3.0;
    if (m[0] < 1 || m[1] < 1 || m[2] < 1) return;
    vector<Mat> ch;
    split(img, ch);
    for (int i = 0; i < 3; ++i) ch[i].convertTo(ch[i], CV_8U, g / m[i]);
    merge(ch, img);
}

static Mat cleanMask(const Mat& m, int minBlobPx) {
    Mat out;
    morphologyEx(m, out, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
    morphologyEx(out, out, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));

    Mat labels, stats, cents;
    int n = connectedComponentsWithStats(out, labels, stats, cents, 8);
    Mat keep = Mat::zeros(out.size(), CV_8UC1);
    for (int i = 1; i < n; ++i)
        if (stats.at<int>(i, CC_STAT_AREA) >= minBlobPx) keep.setTo(255, labels == i);
    return keep;
}

// Fills holes inside blobs (e.g. near-black pixels in the middle of a blue-black layer).
static Mat fillHoles(const Mat& m) {
    vector<vector<Point>> cs;
    findContours(m.clone(), cs, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat out = Mat::zeros(m.size(), CV_8UC1);
    drawContours(out, cs, -1, Scalar(255), FILLED);
    return out;
}

// Main entry point.
//   bgr : full image (BGR)
//   roi : liquid region in original pixels (empty Rect = whole image)
IodineResult detectStarch(const Mat& bgr, Rect roi, const Config& cfg = Config()) {
    IodineResult res;
    CV_Assert(!bgr.empty());

    Rect full(0, 0, bgr.cols, bgr.rows);
    if (roi.area() <= 0) roi = full;
    roi &= full;
    CV_Assert(roi.width > 10 && roi.height > 10);

    Mat raw = bgr(roi).clone();
    Mat img = raw.clone();

    if (cfg.hasWhite) {
        Rect wr = cfg.white & full;
        if (wr.area() > 0) whiteBalance(img, bgr(wr));
    }

    // Light smoothing: suppresses JPEG chroma blocking and sensor noise
    Mat sm;
    medianBlur(img, sm, 5);
    GaussianBlur(sm, sm, Size(0, 0), 1.5);

    Mat hsv;
    cvtColor(sm, hsv, COLOR_BGR2HSV);

    const int H = sm.rows, W = sm.cols;
    Mat purple = Mat::zeros(H, W, CV_8UC1);
    Mat amber  = Mat::zeros(H, W, CV_8UC1);

    for (int y = 0; y < H; ++y) {
        const Vec3b* pb = sm.ptr<Vec3b>(y);
        const Vec3b* ph = hsv.ptr<Vec3b>(y);
        for (int x = 0; x < W; ++x) {
            int b = pb[x][0], g = pb[x][1], r = pb[x][2];
            int mx = max(b, max(g, r)), mn = min(b, min(g, r));
            double sat = (double)(mx - mn) / max(mx, 1);   // relative colour strength
            int hue = ph[x][0], val = ph[x][2];

            if (sat >= cfg.purpleMinSat && val >= cfg.purpleMinV &&
                hue >= cfg.purpleLo && hue <= cfg.purpleHi)
                purple.at<uchar>(y, x) = 255;
            else if (sat >= cfg.amberMinSat && val >= cfg.amberMinV &&
                     hue >= cfg.amberLo && hue <= cfg.amberHi)
                amber.at<uchar>(y, x) = 255;
        }
    }

    purple = fillHoles(cleanMask(purple, cfg.minBlobPx));
    amber  = cleanMask(amber,  cfg.minBlobPx);
    bitwise_and(amber, ~purple, amber);

    res.roiPx    = H * W;
    res.purplePx = countNonZero(purple);
    res.amberPx  = countNonZero(amber);
    res.purpleFrac = (double)res.purplePx / res.roiPx;
    res.amberFrac  = (double)res.amberPx / res.roiPx;
    double colored = res.purpleFrac + res.amberFrac;
    res.share = colored > 0 ? res.purpleFrac / colored : 0.0;

    if (res.purplePx > 0) {
        vector<uchar> vs, hs;
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x)
                if (purple.at<uchar>(y, x)) {
                    vs.push_back(hsv.at<Vec3b>(y, x)[2]);
                    hs.push_back(hsv.at<Vec3b>(y, x)[0]);
                }
        nth_element(vs.begin(), vs.begin() + vs.size() / 2, vs.end());
        nth_element(hs.begin(), hs.begin() + hs.size() / 2, hs.end());
        res.purpleValue = vs[vs.size() / 2];
        res.purpleHue   = hs[hs.size() / 2];
    }

    // ---- Verdict ----
    if (colored < cfg.minColoredFrac) {
        res.verdict = "INCONCLUSIVE";            // no iodine colour visible at all
        res.message = "Iodine test inconclusive";
    } else if (res.purpleFrac >= cfg.minAreaFrac && res.share >= cfg.minShare) {
        res.verdict = "POSITIVE";                // violet / blue found -> starch present
        res.message = "Iodine test positive";
    } else {
        res.verdict = "NEGATIVE";                // only yellow / amber iodine -> no starch
        res.message = "Iodine test negative";
    }

    // ---- Overlay ----
    Mat ov = raw.clone();
    Mat tint = ov.clone();
    tint.setTo(Scalar(255, 0, 255), purple);     // magenta = starch colour
    tint.setTo(Scalar(0, 150, 255), amber);      // orange  = free iodine colour
    addWeighted(tint, 0.55, ov, 0.45, 0, ov);

    vector<vector<Point>> cs;
    findContours(purple.clone(), cs, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(ov, cs, -1, Scalar(255, 255, 255), 2, LINE_AA);

    // Headline banner, font scaled so it always fits the ROI width
    string txt = res.message;
    int baseline = 0;
    Size ts = getTextSize(txt, FONT_HERSHEY_SIMPLEX, 1.0, 2, &baseline);
    double fs1 = min(0.8, (double)(W - 12) / ts.width);
    Size ts2 = getTextSize(txt, FONT_HERSHEY_SIMPLEX, fs1, 2, &baseline);
    Scalar bannerCol = res.verdict == "POSITIVE" ? Scalar(0, 140, 0)
                     : res.verdict == "NEGATIVE" ? Scalar(0, 0, 170) : Scalar(90, 90, 90);
    rectangle(ov, Rect(0, 0, W, ts2.height + 14), bannerCol, FILLED);
    putText(ov, txt, Point(6, ts2.height + 6), FONT_HERSHEY_SIMPLEX, fs1, Scalar(255, 255, 255), 2, LINE_AA);
    res.overlay = ov;

    return res;
}

// =============================================================================
//                              TEST HARNESS
// =============================================================================

static Rect parseRect(const string& s, const char* opt) {
    int x, y, w, h;
    if (sscanf(s.c_str(), "%d,%d,%d,%d", &x, &y, &w, &h) != 4) {
        cerr << "Bad " << opt << ", expected x,y,w,h\n";
        exit(2);
    }
    return Rect(x, y, w, h);
}

static Rect selectRoiInteractive(const Mat& img) {
    double s = min(1.0, 900.0 / img.rows);
    Mat preview;
    resize(img, preview, Size(), s, s, INTER_AREA);
    Rect r = selectROI("Select the LIQUID region of the tube, press ENTER", preview, true);
    destroyAllWindows();
    if (r.area() <= 0) return Rect();
    return Rect(cvRound(r.x / s), cvRound(r.y / s), cvRound(r.width / s), cvRound(r.height / s));
}

static string jsonFor(const string& file, const IodineResult& r) {
    ostringstream o;
    o << "{\"file\":\"" << file << "\","
      << "\"verdict\":\"" << r.verdict << "\","
      << "\"message\":\"" << r.message << "\","
      << "\"violet_fraction\":" << format("%.4f", r.purpleFrac) << ","
      << "\"amber_fraction\":" << format("%.4f", r.amberFrac) << ","
      << "\"share\":" << format("%.3f", r.share) << ","
      << "\"violet_value\":" << format("%.0f", r.purpleValue) << ","
      << "\"violet_hue\":" << format("%.0f", r.purpleHue) << "}";
    return o.str();
}

int main(int argc, char** argv) {
    vector<string> images;
    string outDir = "iodine_out";
    Rect roi;
    bool select = false, asJson = false, show = false;
    Config cfg;

    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        auto next = [&](const char* name) -> string {
            if (i + 1 >= argc) { cerr << "Missing value for " << name << "\n"; exit(2); }
            return argv[++i];
        };
        if      (a == "--roi")         roi = parseRect(next("--roi"), "--roi");
        else if (a == "--white")       { cfg.white = parseRect(next("--white"), "--white"); cfg.hasWhite = true; }
        else if (a == "--select")      select = true;
        else if (a == "--out")         outDir = next("--out");
        else if (a == "--share")       cfg.minShare = stod(next("--share"));
        else if (a == "--min-area")    cfg.minAreaFrac = stod(next("--min-area"));
        else if (a == "--purple-lo")   cfg.purpleLo = stoi(next("--purple-lo"));
        else if (a == "--purple-hi")   cfg.purpleHi = stoi(next("--purple-hi"));
        else if (a == "--amber-lo")    cfg.amberLo = stoi(next("--amber-lo"));
        else if (a == "--amber-hi")    cfg.amberHi = stoi(next("--amber-hi"));
        else if (a == "--min-sat")     cfg.purpleMinSat = stod(next("--min-sat"));
        else if (a == "--json")        asJson = true;
        else if (a == "--show")        show = true;
        else if (a.rfind("--", 0) == 0) { cerr << "Unknown option: " << a << "\n"; return 2; }
        else images.push_back(a);
    }

    if (images.empty()) {
        cerr << "Usage: " << argv[0] << " image.jpg [more.jpg ...] [--roi x,y,w,h] [--select] [--white x,y,w,h] "
             << "[--out dir] [--share F] [--min-area F] [--purple-lo N] [--purple-hi N] [--amber-lo N] "
             << "[--amber-hi N] [--min-sat F] [--json] [--show]\n";
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

        IodineResult r;
        try {
            r = detectStarch(img, roi, cfg);
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
            printf("%s: %s   (violet=%.1f%%, amber=%.1f%%, share=%.2f)\n",
                   path.c_str(), r.message.c_str(),
                   r.purpleFrac * 100.0, r.amberFrac * 100.0, r.share);
        }

        if (show) {
            imshow("iodine", r.overlay);
            waitKey(0);
            destroyAllWindows();
        }
    }
    return 0;
}
