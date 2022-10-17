#include "UserStream.h"

namespace tactile_sensor {


template<size_t N_TAX>
RowSensor<N_TAX>::RowSensor(SamplingCircuit  &circuit) : TactileSensorInterface(circuit){
    for(size_t i=0; i<N_TAX; ++i)
        taxel_order[i]=i;
}

template<size_t N_TAX>
RowSensor<N_TAX>::RowSensor(SamplingCircuit  &circuit, std::array<size_t, N_TAX> taxel_order) : TactileSensorInterface(circuit){
    this->taxel_order = taxel_order;
}

template<size_t N_TAX>
void RowSensor<N_TAX>::measure(){
    for(size_t i=0; i<N_TAX; ++i){
        taxels[i].update_force_estimate( circuit.measure_conductance(taxel_order[i]) );
    }
}

template<size_t N_TAX>
float RowSensor<N_TAX>::total_force() const{
    float sum = 0;
    for(size_t i=0; i<N_TAX; ++i) sum += taxels[i].force;
    return sum;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::avg_force() const{
    return total_force()/N_TAX;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::peak_force() const{
    float max_f = 0;
    for(size_t i=0; i<N_TAX; ++i) max_f = max(taxels[i].force, max_f);
    return max_f;
}

template<size_t N_TAX>
int RowSensor<N_TAX>::peaks() const{
    // TODO
    return -1;
}

template<size_t N_TAX>
typename RowSensor<N_TAX>::location_t RowSensor<N_TAX>::peak_location() const{
    float max_f = 0, x=-1;
    for(size_t i=0; i<N_TAX; ++i){
        if(taxels[i].force > max_f){
            x = i;
            max_f = taxels[i].force;
        }

    }
    location_t loc={x,0};
    return loc;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::raw_signal(size_t i) const {
    return taxels[i].conductance;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::raw_force(size_t i) const {
    return taxels[i].force_est;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::force0(size_t i) const {
    return taxels[i].force0;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::cusum(size_t i) const {
    return taxels[i].cusum;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::is_activated(size_t i) const {
    return taxels[i].is_active;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::force(size_t i) const {
    return taxels[i].force;
}

template<size_t N_TAX>
std::array<float,N_TAX> RowSensor<N_TAX>::forces() const {
    std::array<float,N_TAX> F;
    for(size_t i=0; i<N_TAX; ++i)
        F[i] = taxels[i].force;
    return F;
}

// TODO delete
template<size_t N_TAX>
void RowSensor<N_TAX>::set_calibration(size_t i, float offset, float scale){
    taxels[i].calib[0] = offset;
    taxels[i].calib[1] = scale;
}

template<size_t N_TAX>
void RowSensor<N_TAX>::set_model_params(size_t i, std::array<double, 4> num, std::array<double, 3> den){
    taxels[i].cal_num = num;
    taxels[i].cal_den = den;
    debug_stream << taxels[i].cal_num[0] << " " << taxels[i].cal_num[1] << " " << taxels[i].cal_num[2] << " " << taxels[i].cal_num[3] << "\n";
    debug_stream << taxels[i].cal_den[0] << " " << taxels[i].cal_den[1] << " " << taxels[i].cal_den[2] << "\n";
}

template<size_t N_TAX>
void RowSensor<N_TAX>::set_calib_params(size_t i, calib_type cor){
    taxels[i].cal_cor = cor;
}

template<size_t N_TAX>
void RowSensor<N_TAX>::set_threshold(size_t i, float threshold){
    taxels[i].thres = threshold;
}

template<size_t N_TAX>
float RowSensor<N_TAX>::get_threshold(size_t i){
    return taxels[i].thres;
}

template<size_t N_TAX>
void RowSensor<N_TAX>::set_cusum_w(size_t i, float w){
    taxels[i].cusum_w = w;
}

template<size_t N_TAX>
calib_type RowSensor<N_TAX>::get_calib_params(size_t i){
    return taxels[i].cal_cor;
}


template<size_t N_TAX>
void RowSensor<N_TAX>::printTo(Print &p, bool endline){
    for (size_t i = 0; i < N_TAX; i++)
    {
        p << _FLOAT(taxels[i].force, 3) << " ";
        // Serial << _FLOAT(taxels[i].force0, 3) << "\t"; // debug
        // Serial << (taxels[i].is_active ? 1 : 0) << "\t"; // debug
    }
    if(endline) p << "\n";
}

template<size_t N_TAX>
void RowSensor<N_TAX>::printTo(Print &p, size_t i, bool endline){
    p << _FLOAT(taxels[i].force, 3);
    if(endline) p << "\n";
}
    
};
