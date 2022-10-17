
#pragma once 

#include <Arduino.h>
#include <TactileSensorInterface.h>
#include <array>
#include <Streaming.h>

#define CALTYPE_POLYSPLINE 0
#define CALTYPE_LINSPLINE 1
#define CALTYPE_POLYSPLINE_ACT 2

// #define CALIB_TYPE CALTYPE_POLYSPLINE
#define CALIB_TYPE CALTYPE_POLYSPLINE_ACT

namespace tactile_sensor {
#if CALIB_TYPE==CALTYPE_POLYSPLINE || CALIB_TYPE==CALTYPE_LINSPLINE
constexpr size_t N_CALIB_PARAMS = 4;
#elif CALIB_TYPE==CALTYPE_POLYSPLINE_ACT
constexpr size_t N_CALIB_PARAMS = 5;
#endif

typedef std::array<float, N_CALIB_PARAMS> calib_type;

template<size_t N_TAX>
class RowSensor : public TactileSensorInterface{

public:
    RowSensor(SamplingCircuit  &circuit);
    RowSensor(SamplingCircuit  &circuit, std::array<size_t, N_TAX> taxel_order);
    void measure() override;

    const size_t n_taxels() const override {return N_TAX;}
    float total_force() const override;
    float avg_force() const override;
    float peak_force() const override;
    location_t peak_location() const override;
    int peaks() const override;
    std::array<float,N_TAX> forces() const;
    float force(size_t i) const;
    void printTo(Print &p, bool endline=true);
    void printTo(Print &p, size_t i, bool endline=true);

    // debug
    float raw_signal(size_t i) const;
    float raw_force(size_t i) const;
    float force0(size_t i) const;
    float cusum(size_t i) const;
    float is_activated(size_t i) const;
    
    void set_calibration(size_t i, float offset, float scale);

    void set_model_params(size_t i, std::array<double, 4> num, std::array<double, 3> den);
    void set_calib_params(size_t i, calib_type cor);
    void set_threshold(size_t i, float threshold);
    float get_threshold(size_t i);
    void set_cusum_w(size_t i, float w);
    calib_type get_calib_params(size_t i);

    template<size_t N>
    friend Print& operator<<(Print& out, const RowSensor<N>& obj);

    inline void calib_zero(){
      for(size_t i=0; i<N_TAX; ++i){
         taxels[i].force0 = taxels[i].force_est;
         taxels[i].cusum = 0;
      }
    }
    inline void reset_zero_calib(){
      for(size_t i=0; i<N_TAX; ++i) taxels[i].force0 = 0;
    }
    inline void reset_calib_params(size_t i){
#if CALIB_TYPE==CALTYPE_POLYSPLINE
      set_calib_params(i, {0,1,0,0});
#elif CALIB_TYPE==CALTYPE_POLYSPLINE_ACT
      set_calib_params(i, {0,1,0,0,0});
#elif CALIB_TYPE==CALTYPE_LINSPLINE
      set_calib_params(i, {0,1,0,0});
#endif
    }

private:
    // typedef std::array<float, 3> calib_type;
    struct taxel_t {
      std::array<double, 4> cal_num{{1,0,0,0}};
      std::array<double, 3> cal_den{{0,0,0}};
      // std::array<float, 3> cal_cor{{0,1,0}};
#if CALIB_TYPE==CALTYPE_POLYSPLINE
      calib_type cal_cor{{0,1,0,0}};
#elif CALIB_TYPE==CALTYPE_POLYSPLINE_ACT
      calib_type cal_cor{{0,1,0,0,0}};
#elif CALIB_TYPE==CALTYPE_LINSPLINE
      calib_type cal_cor{{0,1,0,0}};
#endif
      double u_prev[3]={0}, y_prev[3]={0};
      double calib[2] = {0,1000000};
      float force_est=-1, force=0, force0=0, thres=0, cusum=0, cusum_w=0;
      float conductance=0;
      bool is_active=false;
      int active_count = 0;

      void update_force_estimate(float G){

        // double y = cal_num[0]*G + cal_num[1]*u_prev[0] + cal_num[2]*u_prev[1] + cal_num[3]*u_prev[2]
        //           - cal_den[0]*y_prev[0] - cal_den[1]*y_prev[1] - cal_den[2]*y_prev[2];

        // u_prev[2] = u_prev[1];
        // u_prev[1] = u_prev[0];
        // u_prev[0] = G;
        // y_prev[2] = y_prev[1];
        // y_prev[1] = y_prev[0];
        // y_prev[0] = y;
        const double y = 1000*G; // y = [mS]
        conductance = G; // debug

        // force = cal_cor[0]*y*y + cal_cor[1]*y + cal_cor[2]; // 2 deg poly
        
#if CALIB_TYPE==CALTYPE_POLYSPLINE
        // Poly spline:
        float x1 = min(y,cal_cor[0]);
        float x2 = max(y-cal_cor[0],0);
        const float d = 2*cal_cor[0]*cal_cor[2] + cal_cor[1];
        force_est = cal_cor[1]*x1 + cal_cor[2]*x1*x1 + d*x2 + cal_cor[3]*x2*x2;
#elif CALIB_TYPE==CALTYPE_POLYSPLINE_ACT // CURRENT IMPLEMENTATION!!!!!
        float x1 = min(y,cal_cor[0]);
        float x2 = max(y-cal_cor[0],0);
        const float d = 2*cal_cor[0]*cal_cor[2] + cal_cor[1];
        force_est = cal_cor[1]*x1 + cal_cor[2]*x1*x1 + d*x2 + cal_cor[3]*x2*x2 + cal_cor[4];
#elif CALIB_TYPE==CALTYPE_LINSPLINE
        force_est = min(y,cal_cor[0])*cal_cor[1] + max(y-cal_cor[0],0)*cal_cor[2] + cal_cor[3];
#endif

        // active_count = (force_est-force0 >= thres) ? active_count+1 : 0;
        // is_active = cusum > 0.02;
        is_active = force_est-force0 >= thres;
        if(!is_active){
          constexpr float Ts = 0.02;
          constexpr float fc = 0.2;
          constexpr float tau = 1/(2*3.1415*fc);
          constexpr float a0 =         Ts/(2*tau+Ts);
          constexpr float a1 =         a0;
          constexpr float b0 = (Ts-2*tau)/(2*tau+Ts);
          force0 = -b0*force0 + a0*force_est + a1*u_prev[0]; // lowpass filter
          u_prev[0] = force_est;

          //cusum = max(0, cusum + force_est-force0 - cusum_w);
          // if(cusum > 0.02){
          //   Serial << "detect\n";
          //   is_active=true;
          // }

          force = 0;
          // force = force_est; //-force0;
        }else{
          force = force_est - force0;
          if(force_est-force0 < thres){
            is_active = false;
            cusum = 0;
          }
        }

      }
    };
    taxel_t taxels[N_TAX];
    std::array<size_t, N_TAX> taxel_order;

}; // RowSensor

template<size_t N_TAX>
inline Print& operator<<(Print& out, const RowSensor<N_TAX>& obj) {
    for(size_t i=0; i<N_TAX-1; ++i){
      out.print(obj.taxels[i].force);
      out.print(", ");
    }
    out.print(obj.taxels[N_TAX-1].force);
    return out;
}

} // tactile_sensor

#include "RowSensor_impl.h"