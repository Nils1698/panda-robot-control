#pragma once

#include "fingers.h"

#include <vector>
#include <numeric> // accumulate
#include<algorithm> // for_each

namespace TactilePerception {

struct contact_info_t {
  float total_force, area, cm_x, cm_y;
  void printTo(Print &p, bool endline=true){
    p <<"F: "<<total_force<<", A: "<<area<<", CM: ("<<cm_x<<", "<<cm_y<<")";
    if(endline) p << "\n";
  }
};

template<size_t N_TAX>
inline contact_info_t compute_contact_info(std::array<float, N_TAX> forces,  const float thres, const float eta)
{    
  contact_info_t res = {0,0,0,0};

  float sumx=0,sumy=0, data_max=0;
  for(size_t i=0; i<forces.size(); ++i){
    if(forces[i]>thres){
      sumx += hw::Fingers::taxel_xy[i][0]*forces[i];
      sumy += hw::Fingers::taxel_xy[i][1]*forces[i];
      res.total_force += forces[i];
      data_max = max(data_max, forces[i]);
    }
  }
  
  if(res.total_force==0){
    res.area = 0;
    return res;
  }
  
  for(size_t i=0; i<forces.size(); ++i){
      res.area += (forces[i] > eta*data_max) ? 1.0 : forces[i]/(eta*data_max);
  }
  res.area /= forces.size();
  res.cm_x = sumx / res.total_force;
  res.cm_y = sumy / res.total_force;

  return res;
}

struct wrench_info_t {
  float tau_x, tau_z, f_y;
  void printTo(Print &p, bool endline=true){
    p <<"Fy: "<<f_y<<", tau: ("<<tau_x<<", "<<tau_z<<")";
    if(endline) p << "\n";
  }
};

inline wrench_info_t compute_wrench(const contact_info_t s1, const contact_info_t s2)
{    
  wrench_info_t res = {0};
  res.tau_x = (s1.cm_y - s2.cm_y) * (s1.total_force + s2.total_force) / 2;
  res.tau_z = (s1.cm_x + s2.cm_x) * (s1.total_force + s2.total_force) / 2;
  res.f_y = s1.total_force - s2.total_force;
  return res;
}

inline void compute_moment_info(){
  float tx_sum = 0, tz_sum=0;
  for(size_t i=0; i<hw::Fingers::N_TAXELS; ++i){
      float rz = 30.0/2 - hw::Fingers::taxel_xy[i][1];
      float rx = hw::Fingers::taxel_xy[i][0];

      float fy = hw::Fingers::F[0]->TactileSensor.force(i);

      // Tau = r x F
      tx_sum += -rz*fy;
      tz_sum +=  rx*fy;
  }

  for(size_t i=0; i<hw::Fingers::N_TAXELS; ++i){
      float rz = 30.0/2 - hw::Fingers::taxel_xy[i][1];
      float rx = -hw::Fingers::taxel_xy[i][0];

      float fy = -hw::Fingers::F[1]->TactileSensor.force(i);

      // Tau = r x F
      tx_sum += -rz*fy;
      tz_sum +=  rx*fy;
  }

  debug_stream << "(" << tx_sum << ", 0, " << tz_sum << ")\n";

}

inline float stdDev(std::vector<float> &v){
  float sum = std::accumulate(std::begin(v), std::end(v), 0.0);
  float m =  sum / v.size();
  float accum = 0.0;
  std::for_each (std::begin(v), std::end(v), [&](const float d) {
      accum += (d - m) * (d - m);
  });

  return sqrt(accum / v.size());
}

template<size_t N_TAX>
inline float smoothness(std::array<float,N_TAX> T){
  float fmax = 0;
  for(float f : T){
    fmax = max(fmax, f);
  }  
  return fmax==0 ? 0 : (
    fabs(T[1]-T[0])
  + fabs(T[2]-T[0])
  + fabs(T[3]-T[1])
  + fabs(T[3]-T[2])
  + fabs(T[4]-T[2])
  + fabs(T[5]-T[3])
  + fabs(T[5]-T[4])
  + fabs(T[6]-T[4])
  + fabs(T[6]-T[5])
  ) / fmax;
}

template<size_t N_TAX>
inline float smoothness2(std::array<float,N_TAX> T){
  float fmax = 0;
  for(float f : T){
    fmax = max(fmax, f);
  }  
  float thres = 0.1*fmax;

  std::vector<float> vy; //, vx;
  for (size_t i = 0; i < N_TAX; i++)
  {
    if(T[i]>thres){
      //vx.push_back(hw::Fingers::taxel_xy[i][0]);
      vy.push_back(hw::Fingers::taxel_xy[i][1]);
    }
  }
  return stdDev(vy);
}

inline float sensor_var(tactile_sensor::RowSensor<7> &sensor){
  float mu=0, P=0;
  float p[4], x[4];
  for (size_t i = 0; i < 3; i++)
  {
      p[i] = (sensor.force(2*i) + sensor.force(2*i+1)) / 2;
      P += p[i];
      x[i] = hw::Fingers::taxel_xy[2*i][1];
  }
  p[3] = sensor.force(6);
  P += p[3];
  x[4] = hw::Fingers::taxel_xy[6][1];

  if(P<1e-3) return 0;

  for (size_t i = 0; i < 4; i++){
      p[i] /= P;
      mu += p[i]*x[i];
  }

  float var = 0;
  for (size_t i = 0; i < 4; i++){
      var += p[i] * (x[i]-mu)*(x[i]-mu);
  }
  return var;

}

} // TactilePerception