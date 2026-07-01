#pragma once

#include <limits>

#include "kalman_filter/measurement_model.hpp"


namespace kalman
{

template<class StateType>
class ExtendedKalmanFilter;

template<typename StateType, typename MeasurementType,
  template<class> typename CovarianceBase = StandardBase>
class LinearizedMeasurementModel
  : public MeasurementModel<StateType, MeasurementType, CovarianceBase>
{
  friend class ExtendedKalmanFilter<StateType>;

public:
  using Base = MeasurementModel<StateType, MeasurementType, CovarianceBase>;
  using typename Base::State;
  using typename Base::Measurement;

protected:
  LinearizedMeasurementModel()
  : chisq_threshold_(std::numeric_limits<double>::infinity())
  {
    H_.setZero();
    V_.setIdentity();
  }
  ~LinearizedMeasurementModel() override = default;

  virtual void updateJacobians(const State & x)
  {
    (void)x;
  }

  double chisq_threshold_;
  Jacobian<Measurement, State> H_;
  Jacobian<Measurement, Measurement> V_;
};

} // namespace kalman
