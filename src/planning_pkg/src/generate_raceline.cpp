#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include "planning_pkg/gg_diagram.hpp"

struct Pt { double x,y; };
static constexpr double G=9.80665;

static void smooth_vec(std::vector<double>&v,int win=5){
  if(win<3 || (int)v.size()<=win) return;
  std::vector<double> out(v.size(),0.0); int h=win/2;
  for(size_t i=0;i<v.size();++i){
    int i0=std::max<int>(0,(int)i-h);
    int i1=std::min<int>((int)v.size()-1,(int)i+h);
    double s=0; int n=0;
    for(int k=i0;k<=i1;++k){ s+=v[k]; ++n; }
    out[i]=s/std::max(1,n);
  }
  v.swap(out);
}

int main(int argc,char** argv){
  std::string in_csv="tracks/centerline.csv";
  std::string out_csv="data/raceline.csv";
  double ds=0.5, mu=1.0, v_max=20.0, ax_max=4.0, ax_min=-6.0;
  double offset=0.0;  // Offset distance to move raceline outward (positive = left/outside)
  double start_x=0.0, start_y=0.0, start_psi=0.0;  // Vehicle start pose (0,0,0 = use first centerline point)
  bool use_start_pose=false;  // Whether to use start pose

  for(int i=1;i<argc;++i){
    std::string a=argv[i];
    auto nexts=[&](std::string&s){ if(i+1<argc)s=argv[++i]; };
    auto nextd=[&](double&d){ if(i+1<argc)d=std::stod(argv[++i]); };
    if(a=="--centerline_csv") nexts(in_csv);
    else if(a=="--out_csv") nexts(out_csv);
    else if(a=="--ds") nextd(ds);
    else if(a=="--mu") nextd(mu);
    else if(a=="--v_max") nextd(v_max);
    else if(a=="--ax_max") nextd(ax_max);
    else if(a=="--ax_min") nextd(ax_min);
    else if(a=="--offset") nextd(offset);  // Offset to move raceline outward
    else if(a=="--start_x") { nextd(start_x); use_start_pose=true; }
    else if(a=="--start_y") { nextd(start_y); use_start_pose=true; }
    else if(a=="--start_psi") { nextd(start_psi); use_start_pose=true; }
  }

  std::ifstream in(in_csv);
  if(!in.is_open()){ fprintf(stderr,"Cannot open %s\n", in_csv.c_str()); return 1; }
  std::string line;
  if(!std::getline(in,line)){ fprintf(stderr,"Empty %s\n", in_csv.c_str()); return 1; }

  std::vector<Pt> cl;
  while(std::getline(in,line)){
    if(line.empty()) continue;
    std::stringstream ss(line);
    Pt p{}; char c;
    if((ss>>p.x>>c>>p.y) || (ss.clear(), ss.str(line), ss>>p.x>>p.y))
      cl.push_back(p);
  }
  if(cl.size()<2){ fprintf(stderr,"Too few points\n"); return 1; }
  if(std::hypot(cl.front().x-cl.back().x, cl.front().y-cl.back().y) > 1e-6)
    cl.push_back(cl.front());

  std::vector<double> s(cl.size(),0.0);
  for(size_t i=1;i<cl.size();++i){
    double dx=cl[i].x-cl[i-1].x, dy=cl[i].y-cl[i-1].y;
    s[i]=s[i-1]+std::hypot(dx,dy);
  }
  double L=s.back();

  std::vector<double> s_new;
  for(double v=0.0; v<L; v+=ds) s_new.push_back(v);

  std::vector<Pt> pts(s_new.size());
  size_t j=0;
  for(size_t i=0;i<s_new.size();++i){
    while(j+1<s.size() && s[j+1]<s_new[i]) ++j;
    if(j+1>=s.size()) j=s.size()-2;
    double t=(s_new[i]-s[j])/std::max(1e-12, s[j+1]-s[j]);
    pts[i].x=(1-t)*cl[j].x + t*cl[j+1].x;
    pts[i].y=(1-t)*cl[j].y + t*cl[j+1].y;
  }

  auto gradient=[&](const std::vector<double>&a,double h){
    std::vector<double>d(a.size(),0.0);
    if(a.size()<2) return d;
    d[0]=(a[1]-a[0])/h;
    for(size_t i=1;i+1<a.size();++i)
      d[i]=(a[i+1]-a[i-1])/(2*h);
    d.back()=(a.back()-a[a.size()-2])/h;
    return d;
  };

  std::vector<double> px(pts.size()), py(pts.size());
  for(size_t i=0;i<pts.size();++i){ px[i]=pts[i].x; py[i]=pts[i].y; }
  auto d1x=gradient(px,ds); auto d1y=gradient(py,ds);
  auto d2x=gradient(d1x,ds); auto d2y=gradient(d1y,ds);

  std::vector<double> psi(px.size());
  for(size_t i=0;i<psi.size();++i)
    psi[i]=std::atan2(d1y[i], d1x[i]+1e-12);
  for(size_t i=1;i<psi.size();++i){
    double d=psi[i]-psi[i-1];
    while(d> M_PI){ psi[i]-=2*M_PI; d-=2*M_PI; }
    while(d<-M_PI){ psi[i]+=2*M_PI; d+=2*M_PI; }
  }

  std::vector<double> kappa(px.size());
  for(size_t i=0;i<kappa.size();++i){
    double num=d1x[i]*d2y[i] - d1y[i]*d2x[i];
    double den=std::pow(d1x[i]*d1x[i]+d1y[i]*d1y[i],1.5)+1e-12;
    kappa[i]=num/den;
  }

  smooth_vec(psi,7);
  smooth_vec(kappa,7);

  // Apply offset to move raceline outward (left side of path)
  // Positive offset moves path to the left (outside of turns)
  if(std::abs(offset) > 1e-6){
    for(size_t i=0;i<pts.size();++i){
      // Left normal vector: rotate tangent by +90 degrees
      double nx = -std::sin(psi[i]);  // Left normal x
      double ny = std::cos(psi[i]);   // Left normal y
      pts[i].x += offset * nx;
      pts[i].y += offset * ny;
    }
    // Recalculate derivatives after offset
    for(size_t i=0;i<pts.size();++i){ px[i]=pts[i].x; py[i]=pts[i].y; }
    d1x=gradient(px,ds); d1y=gradient(py,ds);
    d2x=gradient(d1x,ds); d2y=gradient(d1y,ds);
    // Recalculate psi and kappa after offset
    for(size_t i=0;i<psi.size();++i)
      psi[i]=std::atan2(d1y[i], d1x[i]+1e-12);
    for(size_t i=1;i<psi.size();++i){
      double d=psi[i]-psi[i-1];
      while(d> M_PI){ psi[i]-=2*M_PI; d-=2*M_PI; }
      while(d<-M_PI){ psi[i]+=2*M_PI; d+=2*M_PI; }
    }
    for(size_t i=0;i<kappa.size();++i){
      double num=d1x[i]*d2y[i] - d1y[i]*d2x[i];
      double den=std::pow(d1x[i]*d1x[i]+d1y[i]*d1y[i],1.5)+1e-12;
      kappa[i]=num/den;
    }
    smooth_vec(psi,7);
    smooth_vec(kappa,7);
  }

  // Initialize G-G Diagram constraint
  planning_pkg::GGDiagram gg_diagram(mu, G);
  
  std::vector<double> vref(pts.size());
  
  // Step 1: Calculate initial speed limits based on curvature and G-G diagram
  for(size_t i=0;i<vref.size();++i){
    // Original curvature-based limit
    double v_kappa=std::sqrt(std::max(0.0, mu*G/(std::abs(kappa[i])+1e-6)));
    
    // G-G diagram constraint: consider zero longitudinal acceleration first
    double v_gg = gg_diagram.max_speed_for_curvature(kappa[i], 0.0);
    
    // Take minimum of both constraints
    vref[i] = std::min({v_kappa, v_gg, v_max});
  }
  
  // Step 2: Forward pass - apply acceleration limits with G-G diagram
  for(size_t i=1;i<vref.size();++i){
    // Calculate required acceleration to reach next speed
    double v_prev = vref[i-1];
    double v_desired = vref[i];
    
    // Required acceleration: a = (v^2 - v_prev^2) / (2*ds)
    double a_req = (v_desired * v_desired - v_prev * v_prev) / (2.0 * ds);
    
    // Limit by maximum acceleration
    a_req = std::min(a_req, ax_max);
    
    // Check G-G diagram constraint
    double a_lat = v_prev * v_prev * std::abs(kappa[i-1]);  // Lateral acceleration
    double a_lon_max = gg_diagram.max_longitudinal_accel(a_lat);
    a_req = std::min(a_req, a_lon_max);
    
    // Calculate new speed
    double v_new = std::sqrt(std::max(0.0, v_prev * v_prev + 2.0 * a_req * ds));
    vref[i] = std::min(v_new, vref[i]);  // Don't exceed curvature limit
  }
  
  // Step 3: Backward pass - apply deceleration limits with G-G diagram
  for(int i=(int)vref.size()-2;i>=0;--i){
    double v_next = vref[i+1];
    double v_current = vref[i];
    
    // Required deceleration: a = (v_next^2 - v_current^2) / (2*ds)
    double a_req = (v_next * v_next - v_current * v_current) / (2.0 * ds);
    
    // Limit by maximum deceleration (negative)
    a_req = std::max(a_req, ax_min);
    
    // Check G-G diagram constraint
    double a_lat = v_current * v_current * std::abs(kappa[i]);  // Lateral acceleration
    double a_lon_min = -gg_diagram.max_longitudinal_accel(a_lat);  // Negative for braking
    a_req = std::max(a_req, a_lon_min);
    
    // Calculate new speed
    double v_new = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * a_req * ds));
    vref[i] = std::min(v_new, vref[i]);  // Don't exceed curvature limit
  }

  // Step 4: Set first point to vehicle start pose if specified
  if(use_start_pose && pts.size() > 0){
    pts[0].x = start_x;
    pts[0].y = start_y;
    psi[0] = start_psi;
    // Recalculate derivatives for first point after changing position
    if(pts.size() > 1){
      double dx = pts[1].x - pts[0].x;
      double dy = pts[1].y - pts[0].y;
      double dist = std::hypot(dx, dy);
      if(dist > 1e-6){
        // Update psi to point towards next point
        psi[0] = std::atan2(dy, dx);
        // Normalize psi
        while(psi[0] > M_PI) psi[0] -= 2*M_PI;
        while(psi[0] < -M_PI) psi[0] += 2*M_PI;
      }
    }
    // Recalculate kappa for first point (will be updated in gradient calculation)
    // Recompute gradients after position change
    for(size_t i=0;i<pts.size();++i){ px[i]=pts[i].x; py[i]=pts[i].y; }
    auto d1x_new=gradient(px,ds); auto d1y_new=gradient(py,ds);
    auto d2x_new=gradient(d1x_new,ds); auto d2y_new=gradient(d1y_new,ds);
    // Update kappa for first few points
    for(size_t i=0;i<std::min<size_t>(3, kappa.size());++i){
      double num=d1x_new[i]*d2y_new[i] - d1y_new[i]*d2x_new[i];
      double den=std::pow(d1x_new[i]*d1x_new[i]+d1y_new[i]*d1y_new[i],1.5)+1e-12;
      kappa[i]=num/den;
    }
  }

  std::ofstream out(out_csv);
  if(!out.is_open()){ fprintf(stderr,"Cannot write %s\n", out_csv.c_str()); return 1; }
  out<<"s,x,y,psi,kappa,v_ref\n";
  for(size_t i=0;i<pts.size();++i){
    out<<s_new[i]<<","<<pts[i].x<<","<<pts[i].y<<","<<psi[i]<<","<<kappa[i]<<","<<vref[i]<<"\n";
  }
  out.close();

  double vmax=*std::max_element(vref.begin(), vref.end());
  double kmax=0.0;
  for(auto &kk:kappa) kmax=std::max(kmax,std::abs(kk));

  fprintf(stdout,"Wrote %s (N=%zu, L≈%.1f m, v_ref_max=%.2f, |kappa|_max=%.3f)\n",
          out_csv.c_str(), pts.size(), L, vmax, kmax);
  return 0;
}
