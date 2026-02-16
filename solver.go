package trilateration

import (
	"fmt"
	"math"

	"gonum.org/v1/gonum/optimize"
)

// Solver encapsulates the position solving logic for Time-of-Flight (ToF) calculations.
// It uses a least-squares optimization approach to determine the position of a tag
// based on distances from known anchor points.
type Solver struct{}

// New creates and returns a new Solver instance
func New() *Solver {
	return &Solver{}
}

// solveProblem defines the optimization problem for the solver.
// It implements the optimize.Problem interface (via the Func method).
type solveProblem struct {
	anchors   []Point
	distances []float64
}

// Func calculates the Sum of Squared Residuals (SSR) for a given position x.
// x is a slice of [x, y, z] coordinates.
func (p *solveProblem) Func(x []float64) float64 {
	var rss float64
	for i, anchor := range p.anchors {
		dx := x[0] - anchor.X
		dy := x[1] - anchor.Y
		dz := x[2] - anchor.Z

		// Euclidean distance from candidate position to anchor
		estimatedDist := math.Sqrt(dx*dx + dy*dy + dz*dz)

		// Residual is the difference between estimated and measured distance
		residual := estimatedDist - p.distances[i]
		rss += residual * residual
	}
	return rss
}

// Solve calculates the 3D position of a tag based on anchor coordinates and measured distances.
//
// Parameters:
//   - anchors: A slice of Point representing the known positions of the anchors.
//   - distances: A slice of float64 representing the measured distances from each anchor to the tag.
//
// Returns:
//   - Point: The calculated 3D position of the tag.
//   - error: An error if the inputs are invalid (e.g., mismatched lengths or insufficient anchors).
//
// The solver uses the Nelder-Mead simplex method to minimize the sum of squared residuals.
// It uses the centroid of the anchors as the initial guess to speed up convergence.
func (s *Solver) Solve(anchors []Point, distances []float64) (Point, error) {
	if len(anchors) != len(distances) {
		return Point{}, fmt.Errorf("length mismatch: %d anchors vs %d distances", len(anchors), len(distances))
	}

	// We require at least 4 points to resolve the 3D position uniquely without mirror ambiguity
	// With 3 points, there are two valid solutions (mirror images), and while the solver might
	// converge to the correct one, it is not guaranteed
	if len(anchors) < 4 {
		return Point{}, fmt.Errorf("insufficient anchor data: need at least 4, got %d", len(anchors))
	}

	// A reasonable starting point speeds up convergence and helps avoid local minima
	var sumX, sumY, sumZ float64
	for _, c := range anchors {
		sumX += c.X
		sumY += c.Y
		sumZ += c.Z
	}
	n := float64(len(anchors))
	initialGuess := []float64{sumX / n, sumY / n, sumZ / n}

	// Minimize sum of squared residuals
	p := &solveProblem{
		anchors:   anchors,
		distances: distances,
	}
	problem := optimize.Problem{
		Func: p.Func,
	}

	// We don't provide a gradient, so gonum defaults to Nelder-Mead
	result, err := optimize.Minimize(problem, initialGuess, nil, nil)
	if err != nil {
		return Point{}, fmt.Errorf("optimization failed: %w", err)
	}

	return Point{X: result.X[0], Y: result.X[1], Z: result.X[2]}, nil
}
