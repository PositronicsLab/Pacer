# Standing -- Regression Test

## Objective

The robot should remain absolutely stationary for this test. 

Inverse dynamics control combined with a static base of support should result in no movement over the test interval.

## Test failure conditions

If the `norm_inf` (maximum magnitude element) of the generalized coordinate error vector exceeds `1e-5` the test is considered a failure.
