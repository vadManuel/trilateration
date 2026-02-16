package trilateration

import (
	"math"
	"testing"
)

func TestSolver_Solve(t *testing.T) {
	tests := []struct {
		name      string
		anchors   []Point
		distances []float64
		want      Point
		wantErr   bool
		tolerance float64
	}{
		{
			name: "Exact 3D Position",
			anchors: []Point{
				{X: 0, Y: 0, Z: 0},
				{X: 10, Y: 0, Z: 0},
				{X: 0, Y: 10, Z: 0},
				{X: 0, Y: 0, Z: 10},
			},
			// Target at (2, 2, 2)
			distances: []float64{
				math.Sqrt(4 + 4 + 4),  // from (0,0,0) -> sqrt(12)
				math.Sqrt(64 + 4 + 4), // from (10,0,0) -> sqrt(72)
				math.Sqrt(4 + 64 + 4), // from (0,10,0) -> sqrt(72)
				math.Sqrt(4 + 4 + 64), // from (0,0,10) -> sqrt(72)
			},
			want:      Point{X: 2, Y: 2, Z: 2},
			wantErr:   false,
			tolerance: 0.001,
		},
		{
			name: "Basic Triangle (Insufficient)",
			anchors: []Point{
				{X: 0, Y: 0, Z: 0},
				{X: 10, Y: 0, Z: 0},
				{X: 0, Y: 10, Z: 0},
			},
			distances: []float64{
				math.Sqrt(5*5 + 5*5 + 5*5),
				math.Sqrt(5*5 + 5*5 + 5*5),
				math.Sqrt(5*5 + 5*5 + 5*5),
			},
			want:      Point{},
			wantErr:   true,
			tolerance: 0.1,
		},
		{
			name: "Mismatch Lengths",
			anchors: []Point{
				{X: 0, Y: 0, Z: 0},
				{X: 10, Y: 0, Z: 0},
				{X: 0, Y: 10, Z: 0},
			},
			distances: []float64{10, 10},
			want:      Point{},
			wantErr:   true,
			tolerance: 0,
		},
		{
			name: "Optimization Failure (Inf)",
			anchors: []Point{
				{X: 0, Y: 0, Z: 0},
				{X: 10, Y: 0, Z: 0},
				{X: 0, Y: 10, Z: 0},
				{X: 0, Y: 0, Z: 10},
			},
			distances: []float64{math.Inf(1), 10, 10, 10},
			want:      Point{},
			wantErr:   true,
			tolerance: 0,
		},
	}

	s := New()

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got, err := s.Solve(tt.anchors, tt.distances)
			if (err != nil) != tt.wantErr {
				t.Errorf("Solver.Solve() error = %v, wantErr %v", err, tt.wantErr)
				return
			}

			if math.Abs(got.X-tt.want.X) > tt.tolerance {
				t.Errorf("Solver.Solve() X = %v, want %v (diff %v)", got.X, tt.want.X, math.Abs(got.X-tt.want.X))
			}
			if math.Abs(got.Y-tt.want.Y) > tt.tolerance {
				t.Errorf("Solver.Solve() Y = %v, want %v (diff %v)", got.Y, tt.want.Y, math.Abs(got.Y-tt.want.Y))
			}
			if math.Abs(got.Z-tt.want.Z) > tt.tolerance {
				t.Errorf("Solver.Solve() Z = %v, want %v (diff %v)", got.Z, tt.want.Z, math.Abs(got.Z-tt.want.Z))
			}
		})
	}
}
