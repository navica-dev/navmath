package vector

import (
	"fmt"
	"math"

	"golang.org/x/exp/constraints"
)

type Numeric interface {
	constraints.Integer | constraints.Float
}

type Vector2[T Numeric] struct {
	X, Y T
}

func NewVector2[T Numeric](x, y T) Vector2[T] {
	return Vector2[T]{X: x, Y: y}
}

func (v Vector2[T]) Add(other Vector2[T]) Vector2[T] {
	return Vector2[T]{
		X: v.X + other.X,
		Y: v.Y + other.Y,
	}
}

func (v Vector2[T]) Subtract(other Vector2[T]) Vector2[T] {
	return Vector2[T]{
		X: v.X - other.X,
		Y: v.Y - other.Y,
	}
}

// Multiply multiplies the vector by a scalar
func (v Vector2[T]) Multiply(scalar T) Vector2[T] {
	return Vector2[T]{
		X: v.X * scalar,
		Y: v.Y * scalar,
	}
}

// Dot returns the dot product of two vectors
func (v Vector2[T]) Dot(other Vector2[T]) T {
	return v.X*other.X + v.Y*other.Y
}

// MagnitudeSquared returns the squared length of the vector
func (v Vector2[T]) MagnitudeSquared() T {
	return v.X*v.X + v.Y*v.Y
}

// Magnitude returns the length of the vector as a float64
func (v Vector2[T]) Magnitude() float64 {
	return math.Sqrt(float64(v.MagnitudeSquared()))
}

// Normalize returns a unit vector in the same direction (as float64)
// For integer vectors, this will convert to Vector2[float64]
func (v Vector2[T]) Normalize() Vector2[float64] {
	mag := v.Magnitude()
	if mag < 1e-10 {
		return Vector2[float64]{0, 0}
	}
	return Vector2[float64]{
		X: float64(v.X) / mag,
		Y: float64(v.Y) / mag,
	}
}

// Distance returns the Euclidean distance between two vectors
func Distance[T Numeric](a, b Vector2[T]) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Sqrt(float64(dx*dx + dy*dy))
}

// ManhattanDistance returns the Manhattan (L1) distance between two vectors
func ManhattanDistance[T Numeric](a, b Vector2[T]) T {
	dx := a.X - b.X
	dy := a.Y - b.Y

	// Need to handle absolute value differently for generic types
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}

	return dx + dy
}

// Equal checks if two vectors are exactly equal
func (v Vector2[T]) Equal(other Vector2[T]) bool {
	return v.X == other.X && v.Y == other.Y
}

// Convert converts a vector from one numeric type to another
func Convert[T, U Numeric](v Vector2[T]) Vector2[U] {
	return Vector2[U]{
		X: U(v.X),
		Y: U(v.Y),
	}
}

// String implements the Stringer interface for pretty printing
func (v Vector2[T]) String() string {
	return fmt.Sprintf("Vector2(%v, %v)", v.X, v.Y)
}

type Vector3[T Numeric] struct {
	X, Y, Z T
}

func NewVector3[T Numeric](x, y, z T) Vector3[T] {
	return Vector3[T]{X: x, Y: y, Z: z}
}

func (v Vector3[T]) Add(other Vector3[T]) Vector3[T] {
	return Vector3[T]{
		X: v.X + other.X,
		Y: v.Y + other.Y,
		Z: v.Z + other.Z,
	}
}

func (v Vector3[T]) Subtract(other Vector3[T]) Vector3[T] {
	return Vector3[T]{
		X: v.X - other.X,
		Y: v.Y - other.Y,
		Z: v.Z - other.Z,
	}
}

// Multiply multiplies the vector by a scalar
func (v Vector3[T]) Multiply(scalar T) Vector3[T] {
	return Vector3[T]{
		X: v.X * scalar,
		Y: v.Y * scalar,
		Z: v.Z * scalar,
	}
}

// Dot returns the dot product of two vectors
func (v Vector3[T]) Dot(other Vector3[T]) T {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

// Cross returns the cross product of two vectors
func (v Vector3[T]) Cross(other Vector3[T]) Vector3[T] {
	return Vector3[T]{
		X: v.Y*other.Z - v.Z*other.Y,
		Y: v.Z*other.X - v.X*other.Z,
		Z: v.X*other.Y - v.Y*other.X,
	}
}

// MagnitudeSquared returns the squared length of the vector
func (v Vector3[T]) MagnitudeSquared() T {
	return v.X*v.X + v.Y*v.Y + v.Z*v.Z
}

// Magnitude returns the length of the vector as a float64
func (v Vector3[T]) Magnitude() float64 {
	return math.Sqrt(float64(v.MagnitudeSquared()))
}

// Normalize returns a unit vector in the same direction
func (v Vector3[T]) Normalize() Vector3[float64] {
	mag := v.Magnitude()
	if mag < 1e-10 {
		return Vector3[float64]{0, 0, 0}
	}
	return Vector3[float64]{
		X: float64(v.X) / mag,
		Y: float64(v.Y) / mag,
		Z: float64(v.Z) / mag,
	}
}

// Distance3D returns the Euclidean distance between two 3D vectors
func Distance3D[T Numeric](a, b Vector3[T]) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	dz := a.Z - b.Z
	return math.Sqrt(float64(dx*dx + dy*dy + dz*dz))
}

// ManhattanDistance3D returns the Manhattan (L1) distance between two 3D vectors
func ManhattanDistance3D[T Numeric](a, b Vector3[T]) T {
	dx := a.X - b.X
	dy := a.Y - b.Y
	dz := a.Z - b.Z
	
	// Need to handle absolute value differently for generic types
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	if dz < 0 {
		dz = -dz
	}
	
	return dx + dy + dz
}

// Equal checks if two vectors are exactly equal
func (v Vector3[T]) Equal(other Vector3[T]) bool {
	return v.X == other.X && v.Y == other.Y && v.Z == other.Z
}

// Convert3D converts a 3D vector from one numeric type to another
func Convert3D[T, U Numeric](v Vector3[T]) Vector3[U] {
	return Vector3[U]{
		X: U(v.X),
		Y: U(v.Y),
		Z: U(v.Z),
	}
}

// String implements the Stringer interface for pretty printing
func (v Vector3[T]) String() string {
	return fmt.Sprintf("Vector3(%v, %v, %v)", v.X, v.Y, v.Z)
}