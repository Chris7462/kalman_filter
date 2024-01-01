#pragma once

#include "kalman_filter/matrix.hpp"
#include "kalman_filter/state.hpp"
#include "kalman_filter/standard_base.hpp"


namespace kalman
{

class ExtendedKalmanFilter;

class VelMeasurement : public kalman::Vector<1>
{
public:
  KALMAN_VECTOR(VelMeasurement, 1)
  enum : uint8_t
  {
    NU
  };

  inline double nu() const {return (*this)[NU];}

  inline double & nu() {return (*this)[NU];}
};

class VelMeasurementModel : public StandardBase<VelMeasurement>
{
  friend class kalman::ExtendedKalmanFilter;

public:
  VelMeasurementModel();
  ~VelMeasurementModel() = default;

  VelMeasurement h(const State & x) const;
  void updateJacobians(const State & x);

private:
  double threshold_;
  Jacobian<VelMeasurement, State> H_;
  Jacobian<VelMeasurement, VelMeasurement> V_;
};

} // namespace kalman
