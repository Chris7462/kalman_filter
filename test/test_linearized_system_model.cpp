#include <gtest/gtest.h>

#define protected public
#include "kalman_filter/linearized_system_model.hpp"
#undef protected


using namespace kalman;

namespace
{

// Minimal concrete model so we can instantiate LinearizedSystemModel,
// which is otherwise abstract (f() is pure virtual in SystemModel).
class TestSystemModel : public LinearizedSystemModel<Vector<3>>
{
public:
  Vector<3> f(const Vector<3> & x, const Vector<0> & u) const override
  {
    (void)u;
    return x;
  }
};

} // namespace

class LinearizedSystemModelTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    model = std::make_unique<TestSystemModel>();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<TestSystemModel> model;
};

TEST_F(LinearizedSystemModelTest, Constructor_FJacobian_TC1)
{
  EXPECT_EQ(model->F_.rows(), 3);
  EXPECT_EQ(model->F_.cols(), 3);
  EXPECT_TRUE(model->F_.isIdentity());
}

TEST_F(LinearizedSystemModelTest, Constructor_WJacobian_TC1)
{
  EXPECT_EQ(model->W_.rows(), 3);
  EXPECT_EQ(model->W_.cols(), 3);
  EXPECT_TRUE(model->W_.isZero());
}

TEST_F(LinearizedSystemModelTest, updateJacobians_DefaultNoOp_TC1)
{
  Vector<3> x;
  x.setZero();
  Vector<0> u;

  model->updateJacobians(x, u);

  // Default updateJacobians() is a no-op; F_/W_ stay at constructor values.
  EXPECT_TRUE(model->F_.isIdentity());
  EXPECT_TRUE(model->W_.isZero());
}
