#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

/*
  C++ CLI to generate raceline.csv from a centerline polyline:
  - Resample by arc length (linear interpolation)
  - Estimate heading (psi) and curvature (kappa)
  - Compute feasible v_ref from curvature limit and forward/backward longitudinal limits

  Input (tracks/centerline.csv):
    header: x,y
    rows  : closed-loop polyline in meters

  Output (data/raceline.csv):
    s,x,y,psi,kappa,v_ref

  Run:
    ros2 run planning_pkg generate_raceline --centerline_csv tracks/centerline.csv \
      --out_csv data/raceline.csv --ds 0.5 --mu 1.0 --v_max 20.0 --ax_max 4.0 --ax_min -6.0
*/

namespace {

struct Pt { double x, y; };

constexpr double G = 9.80665;

bool read_centerline(const std::string& path, std::vector<Pt>& pts) {
  std::ifstream in(path);
  if (!in.is_open()) return false;
  std::string line;
  // header
  if (!std::getline(in, line)) return false;
  // rows
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    Pt p{};
    char comma;
    if ((ss >> p.x >> comma >> p.y) || (ss.clear(), ss.str(line), ss >> p.x >> p.y)) {
      pts.push_back(p);
    }
  }
  // ensure closed loop
  if (!pts.empty()) {
    const Pt& a = pts.front();
    const Pt& b = pts.back();
    const double dx = a.x - b.x, dy = a.y - b.y;
    if (std::hypot(dx, dy) > 1e-6) pts.push_back(a);
  }
  return !pts.empty();
}

void cumulative_s(const std::vector<Pt>& pts, std::vector<double>& s) {
  s.resize(pts.size());
  s[0] = 0.0;
  for (size_t i=1; i<pts.size(); ++i) {
    const double dx = pts[i].x - pts[i-1].x;
    const double dy = pts[i].y - pts[i-1].y;
    s[i] = s[i-1] + std::hypot(dx, dy);
  }
}

void lininterp(const std::vector<double>& s, const std::vector<Pt>& pts,
               const std::vector<double>& s_new, std::vector<Pt>& pts_new) {
  pts_new.resize(s_new.size());
  size_t j = 0;
  for (size_t i=0; i<s_new.size(); ++i) {
    const double sn = s_new[i];
    while (j+1 < s.size() && s[j+1] < sn) ++j;
    if (j+1 >= s.size()) j = s.size()-2;
    const double t = (sn - s[j]) / std::max(1e-12, s[j+1] - s[j]);
    pts_new[i].x = (1.0 - t) * pts[j].x + t * pts[j+1].x;
    pts_new[i].y = (1.0 - t) * pts[j].y + t * pts[j+1].y;
  }
}

void gradient(const std::vector<double>& y, double ds, std::vector<double>& dy) {
  const size_t n = y.size();
  dy.assign(n, 0.0);
  if (n < 2) return;
  dy[0] = (y[1] - y[0]) / ds;
  for (size_t i=1; i<n-1; ++i) dy[i] = (y[i+1] - y[i-1]) / (2.0 * ds);
  dy[n-1] = (y[n-1] - y[n-2]) / ds;
}

void unwrap(std::vector<double>& angle) {
  double prev = angle[0];
  for (size_t i=1; i<angle.size(); ++i) {
    double a = angle[i];
    double diff = a - prev;
    while (diff > M_PI) { a -= 2*M_PI; diff = a - prev; }
    while (diff < -M_PI) { a += 2*M_PI; diff = a - prev; }
    angle[i] = a;
    prev = a;
  }
}

} // namespace

int main(int argc, char** argv) {
  // Simple argument parsing (no rclcpp init needed)
  std::string in_csv = "tracks/centerline.csv";
  std::string out_csv = "data/raceline.csv";
  double ds = 0.5, mu = 1.0, v_max = 20.0, ax_max = 4.0, ax_min = -6.0;

  for (int i=1; i<argc; ++i) {
    std::string a = argv[i];
    auto next = [&](double& v){ if (i+1<argc) v = std::stod(argv[++i]); };
    auto nexts = [&](std::string& v){ if (i+1<argc) v = argv[++i]; };
    if (a == "--centerline_csv") nexts(in_csv);
    else if (a == "--out_csv") nexts(out_csv);
    else if (a == "--ds") next(ds);
    else if (a == "--mu") next(mu);
    else if (a == "--v_max") next(v_max);
    else if (a == "--ax_max") next(ax_max);
    else if (a == "--ax_min") next(ax_min);
  }

  std::vector<Pt> cl;
  if (!read_centerline(in_csv, cl)) {
    fprintf(stderr, "Failed to read centerline: %s\n", in_csv.c_str());
    return 1;
  }

  // Original cumulative arc-length
  std::vector<double> s_orig;
  cumulative_s(cl, s_orig);
  const double L = s_orig.back();

  // New arc-length samples
  std::vector<double> s_new;
  for (double s = 0.0; s < L; s += ds) s_new.push_back(s);

  // Resample points
  std::vector<Pt> pts;
  lininterp(s_orig, cl, s_new, pts);

  // Heading psi
  std::vector<double> dx(pts.size()), dy(pts.size());
  for (size_t i=0; i<pts.size(); ++i) {
    if (i == 0) { dx[i] = (pts[1].x - pts[0].x) / ds; dy[i] = (pts[1].y - pts[0].y) / ds; }
    else if (i+1 == pts.size()) { dx[i] = (pts[i].x - pts[i-1].x) / ds; dy[i] = (pts[i].y - pts[i-1].y) / ds; }
    else { dx[i] = (pts[i+1].x - pts[i-1].x) / (2.0*ds); dy[i] = (pts[i+1].y - pts[i-1].y) / (2.0*ds); }
  }
  std::vector<double> psi(pts.size());
  for (size_t i=0; i<pts.size(); ++i) psi[i] = std::atan2(dy[i], dx[i] + 1e-12);
  unwrap(psi);

  // Curvature kappa
  std::vector<double> d1x, d1y, d2x, d2y;
  std::vector<double> px(pts.size()), py(pts.size());
  for (size_t i=0; i<pts.size(); ++i) { px[i] = pts[i].x; py[i] = pts[i].y; }
  gradient(px, ds, d1x); gradient(py, ds, d1y);
  gradient(d1x, ds, d2x); gradient(d1y, ds, d2y);

  std::vector<double> kappa(pts.size());
  for (size_t i=0; i<pts.size(); ++i) {
    const double num = d1x[i]*d2y[i] - d1y[i]*d2x[i];
    const double den = std::pow(d1x[i]*d1x[i] + d1y[i]*d1y[i], 1.5) + 1e-12;
    kappa[i] = num / den;
  }

  // Speed profile from curvature
  std::vector<double> v(pts.size());
  for (size_t i=0; i<pts.size(); ++i) {
    const double v_kappa = std::sqrt(std::max(0.0, mu * G / (std::abs(kappa[i]) + 1e-6)));
    v[i] = std::min(v_kappa, v_max);
  }

  // Forward pass (accel limit)
  for (size_t i=1; i<v.size(); ++i) {
    const double v_allow = std::sqrt(std::max(0.0, v[i-1]*v[i-1] + 2.0*ax_max*ds));
    v[i] = std::min(v[i], v_allow);
  }
  // Backward pass (decel limit; ax_min negative)
  for (int i=(int)v.size()-2; i>=0; --i) {
    const double v_allow = std::sqrt(std::max(0.0, v[i+1]*v[i+1] + 2.0*ax_min*ds));
    v[i] = std::min(v[i], v_allow);
  }

  // Write CSV
  std::ofstream out(out_csv);
  if (!out.is_open()) {
    fprintf(stderr, "Failed to write: %s\n", out_csv.c_str());
    return 1;
  }
  out << "s,x,y,psi,kappa,v_ref\n";
  for (size_t i=0; i<pts.size(); ++i) {
    out << s_new[i] << "," << pts[i].x << "," << pts[i].y << ","
        << psi[i] << "," << kappa[i] << "," << v[i] << "\n";
  }
  out.close();

  printf("Wrote raceline to %s (N=%zu, L≈%.1f m)\n", out_csv.c_str(), pts.size(), L);
  return 0;
}
