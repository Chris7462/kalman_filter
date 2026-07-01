#include <gtest/gtest.h>
#include <limits>

#define protected public
#include "kalman_filter/linearized_measurement_model.hpp"
#undef protected


using namespace kalman;

namespace
{

// Minimal concrete model so we can instantiate LinearizedMeasurementModel,
// which is otherwise abstract (h() is pure virtual in MeasurementModel).
class TestMeasurementModel : public LinearizedMeasurementModel<Vector<3>, Vector<2>>
{
public:
  Vector<2> h(const Vector<3> & x) const override
  {
    (void)x;
    Vector<2> z;
    z.setZero();
    return z;
  }
};

} // namespace

class LinearizedMeasurementModelTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    model = std::make_unique<TestMeasurementModel>();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<TestMeasurementModel> model;
};

TEST_F(LinearizedMeasurementModelTest, Constructor_ChiSqThreshold_TC1)
{
  EXPECT_EQ(model->chisq_threshold_, std::numeric_limits<double>::infinity());
}

TEST_F(LinearizedMeasurementModelTest, Constructor_HJacobian_TC1)
{
  EXPECT_EQ(model->H_.rows(), 2);
  EXPECT_EQ(model->H_.cols(), 3);
  EXPECT_TRUE(model->H_.isZero());
}

TEST_F(LinearizedMeasurementModelTest, Constructor_VJacobian_TC1)
{
  EXPECT_EQ(model->V_.rows(), 2);
  EXPECT_EQ(model->V_.cols(), 2);
  EXPECT_TRUE(model->V_.isIdentity());
}

TEST_F(LinearizedMeasurementModelTest, updateJacobians_DefaultNoOp_TC1)
{
  Vector<3> x;
  x.setZero();
  model->updateJacobians(x);

  // Default updateJacobians() is a no-op; H_/V_ stay at constructor values.
  EXPECT_TRUE(model->H_.isZero());
  EXPECT_TRUE(model->V_.isIdentity());
}
