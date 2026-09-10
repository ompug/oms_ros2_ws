#include <gtest/gtest.h>

#include "lego_loam/projection_math.h"

TEST(ProjectionMath, MapsVlp16AnglesToExpectedRows) {
  constexpr float resolution = 2.0F * lego_loam::kDegreesToRadians;
  EXPECT_EQ(lego_loam::verticalRow(-15.0F * lego_loam::kDegreesToRadians,
                                   -15.0F, resolution),
            0);
  EXPECT_EQ(lego_loam::verticalRow(15.0F * lego_loam::kDegreesToRadians,
                                   -15.0F, resolution),
            15);
}

TEST(ProjectionMath, AcceptsPositiveAndNegativeGroundSlopesWithinThreshold) {
  EXPECT_TRUE(lego_loam::isGroundPair(1.0F, 0.0F, 0.1F, 0.0F));
  EXPECT_TRUE(lego_loam::isGroundPair(1.0F, 0.0F, -0.1F, 0.0F));
}

TEST(ProjectionMath, RejectsPositiveAndNegativeSlopesBeyondThreshold) {
  EXPECT_FALSE(lego_loam::isGroundPair(1.0F, 0.0F, 0.25F, 0.0F));
  EXPECT_FALSE(lego_loam::isGroundPair(1.0F, 0.0F, -0.25F, 0.0F));
}

TEST(ProjectionMath, AppliesMountingAngleSymmetrically) {
  constexpr float mount = 5.0F * lego_loam::kDegreesToRadians;
  EXPECT_TRUE(lego_loam::isGroundPair(1.0F, 0.0F,
                                      std::tan(14.9F * lego_loam::kDegreesToRadians),
                                      mount));
  EXPECT_FALSE(lego_loam::isGroundPair(1.0F, 0.0F,
                                       std::tan(-5.1F * lego_loam::kDegreesToRadians),
                                       mount));
}
