# note of ideas

## Nov 20

* Since the constraint function is one dimension, the orthogonal complement of a chart is one dimension. So the projection of Atlas is a one-dimensional optimization, where theconfig move along the orthogonal vector of a chart. 
* The projection now stop at a local optimal point, but it is curious that why such a point exist. The **jacobian** is almost **the same** on both side of this point, but the calculated **direction** are **reversed** after passing this point.

## Nov 21

Check the performance of different parameters.

| $\epsilon$ | $\rho$ | $\alpha$ | average least norm |
| :--------: | :----: | :------: | :----------------: |
|   0.002    |  0.1   | $\pi/8$  |     0.0534156      |
|   0.005    |  0.1   | $\pi/8$  |     0.0579916      |
|    0.01    |  0.1   | $\pi/8$  |     0.0563779      |
|    0.02    |  0.1   | $\pi/8$  |     0.0535655      |
|    0.05    |  0.1   | $\pi/8$  |     0.0574297      |
|            |        |          |                    |
|   0.005    |  0.01  | $\pi/8$  |     0.00613032     |
|   0.005    |  0.02  | $\pi/8$  |      0.011139      |
|   0.005    |  0.05  | $\pi/8$  |     0.0270211      |
|   0.005    |  0.1   | $\pi/8$  |     0.0579916      |
|   0.005    |  0.2   | $\pi/8$  |      0.104793      |
|   0.005    |  0.5   | $\pi/8$  |      0.221473      |
|            |        |          |                    |
|   0.005    |  0.1   | $\pi/16$ |     0.0594703      |
|   0.005    |  0.1   | $\pi/12$ |     0.0533916      |
|   0.005    |  0.1   | $\pi/8$  |     0.0579916      |
|   0.005    |  0.1   | $\pi/6$  |     0.0545765      |
|   0.005    |  0.1   | $\pi/4$  |     0.0641769      |
|   0.005    |  0.1   | $\pi/3$  |     0.0497173      |

## Nov 26

* Use redundent constraint instead

## Dec 15

* If we use the redundant constraint, we will need to take sigular point into consideration, because the dimension of tangent space on that point is different.
* Also, we do not need to follow the format of T0_w, Tw_e and Bw anymore, because we can build the virtual robot in advance and just pass the virtual robot to the algorithm.
* The constraint function can be defined as the diff of (x, y, z, r, p, y) so that we can calculate Jacobian easily.

## Dec 16

* Change the constraint function to redundant, need to check if every part works properly

## Dec 18

* Check projection success rate: satisfactory
* Change Jacobian to analytical
* Change ambient space wrapping: finished
* Need to consider singular points
* Try to determine the dimension of manifold from redundant constraint
* Test different parameters