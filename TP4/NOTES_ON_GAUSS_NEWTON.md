# Important Note on Gauss-Newton Optimizer

## Assignment Requirement vs Reality

The assignment (Task 2.2.B and 3.2.B) specifically asks to use **Gauss-Newton optimizer**. However, when testing with the Intel dataset, Gauss-Newton **diverges**.

### What Happens with Gauss-Newton

```
Initial error: 2,574,860
Final error:   1,546,783,319  (600x WORSE!)
Status: DIVERGED - Error increased
```

The optimizer stops after 1 iteration with the warning:

```
"stopping nonlinear iterations because error increased"
```

### Why Does Gauss-Newton Fail?

1. **Poor Initial Estimates**: The G2O file contains initial estimates that are far from the optimum
2. **Linear Approximation Invalid**: Gauss-Newton assumes the problem is nearly linear, which isn't true here
3. **No Step Size Control**: Gauss-Newton takes full steps, which can overshoot the minimum

### What to Include in Your Report

#### Option 1: Show the Failure (More Honest)

Include the Gauss-Newton results showing divergence and explain:

_"As requested, we implemented batch optimization using the Gauss-Newton optimizer. However, the optimizer diverged (error increased from 2.57M to 1.55B) due to poor initial estimates causing the linear approximation to be invalid. This is a known limitation of Gauss-Newton when the starting point is far from the optimum. In the graph, we can see the trajectory became worse after 'optimization'."_

Then show the ISAM2 incremental results which work perfectly.

#### Option 2: Use Levenberg-Marquardt (More Practical)

If you want working results for Task 2.2.B, I can change it to Levenberg-Marquardt and you can note in the report:

_"While the assignment suggested Gauss-Newton, we used Levenberg-Marquardt optimizer for the batch solution as it is more robust to poor initialization. This is a standard practice in SLAM implementations."_

### Current Code Status

The code currently uses **Gauss-Newton** as requested in the assignment, but it diverges.

### How to Switch Between Optimizers

If your professor is okay with using a more robust optimizer, I can easily switch between:

1. **Gauss-Newton** (current - diverges, but follows assignment literally)
2. **Levenberg-Marquardt** (better results, but not exactly what assignment asked)

### Comparison

| Optimizer           | Task 2.2.B Result               | Notes                        |
| ------------------- | ------------------------------- | ---------------------------- |
| Gauss-Newton        | DIVERGED (error ×600)           | Follows assignment literally |
| Levenberg-Marquardt | Converged (error reduced 85%)   | More practical               |
| ISAM2 (Task 2.3.C)  | Converged (error reduced 99.5%) | Best results                 |

### Recommendation

**For your report**, I suggest:

1. Acknowledge that the assignment asked for Gauss-Newton
2. Show that it diverged and explain why
3. Mention that in practice, Levenberg-Marquardt or ISAM2 would be used
4. Focus on the excellent ISAM2 results (Task 2.3.C) which work perfectly

This shows understanding of the theory AND practical considerations.

### Which Version Do You Want?

Let me know if you want me to:

- **A) Keep Gauss-Newton** (current) - show the divergence in your report as a learning point
- **B) Switch to Levenberg-Marquardt** - get working results for 2.2.B
- **C) Implement both** - run both and compare in the report (best for learning!)
