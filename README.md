# Trilateration Library

A Go library for solving 3D trilateration problems. It calculates a position (x, y, z) based on distance measurements from known anchor points.

While commonly used for Time-of-Flight (ToF) positioning (e.g., UWB), this library is hardware-agnostic and works with any ranging technology (e.g., RSSI, Ultrasonic, Lidar).

## Installation

```bash
go get github.com/vadmanuel/trilateration
```

## Usage

```go
package main

import (
	"fmt"
	"log"

	"github.com/vadmanuel/trilateration"
)

func main() {
	// 1. Define known anchor positions
	anchors := []trilateration.Point{
		{X: 0, Y: 0, Z: 0},  // Anchor 0
		{X: 10, Y: 0, Z: 0}, // Anchor 1
		{X: 0, Y: 10, Z: 0}, // Anchor 2
		{X: 0, Y: 0, Z: 10}, // Anchor 3
	}

	// 2. Provide measured distances from each anchor to the tag
	// The order of the distances must match the order of the anchors array
	distances := []float64{
		3.46, // Distance from anchor 0
		8.66, // Distance from anchor 1
		8.66, // Distance from anchor 2
		8.66, // Distance from anchor 3
	}

	// 3. Create a solver instance
	solver := trilateration.New()

	// 4. Calculate the position
	position, err := solver.Solve(anchors, distances)
	if err != nil {
		log.Fatalf("Failed to solve position: %v", err)
	}

	fmt.Printf("Calculated Position: X=%.2f, Y=%.2f, Z=%.2f\n", position.X, position.Y, position.Z)
}
```

## How It Works

This library solves the 3D trilateration problem using a non-linear least squares optimization approach.

1.  It first calculates the centroid of the provided anchor points to use as a starting estimate.
2.  It uses the Nelder-Mead simplex method (via `gonum.org/v1/gonum/optimize`) to find the coordinate $(x, y, z)$ that minimizes the sum of squared residuals:

    $$\sum\_{i=1}^{n} (\sqrt{(x-x_i)^2 + (y-y_i)^2 + (z-z_i)^2} - d_i)^2$$

    Where $(x_i, y_i, z_i)$ is the position of anchor $i$ and $d_i$ is the measured distance.

    > The solver requires at least 4 anchors to uniquely resolve the 3D position and avoid mirror ambiguity.

## Requirements

- Go 1.18+
- `gonum.org/v1/gonum`
