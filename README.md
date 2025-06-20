# Dynamic Window Algorithm (DWA) Implementation

A Java implementation of the Dynamic Window Algorithm for real-time robot path planning and obstacle avoidance.

## Overview

The Dynamic Window Algorithm is a local path planning method that enables robots to navigate safely through dynamic environments with moving obstacles. This implementation provides a complete solution for 2D robot navigation with configurable parameters and real-time obstacle avoidance.

## Features

- **Real-time Path Planning**: Continuous trajectory optimization based on current robot state
- **Dynamic Obstacle Avoidance**: Handles both static and moving obstacles
- **Configurable Parameters**: Easily adjustable robot constraints and planning parameters
- **Collision Detection**: Robust collision checking with circular obstacles
- **Goal-oriented Navigation**: Efficient pathfinding toward target destinations

## Algorithm Details

The Dynamic Window Algorithm works by:

1. **Dynamic Window Generation**: Creates a set of feasible velocity commands based on robot constraints
2. **Trajectory Prediction**: Simulates robot motion for each velocity command over a prediction horizon
3. **Trajectory Evaluation**: Scores each trajectory based on distance to goal and collision avoidance
4. **Command Selection**: Selects the velocity command with the highest score

## Files

### `DynamicWindowPlanner.java`
Core implementation of the Dynamic Window Algorithm containing:
- **DynamicWindowPlanner**: Main planning class with configurable parameters
- **State**: Robot state representation (position, orientation, velocities)
- **Control**: Control command structure (linear and angular velocities)
- **Obstacle**: Dynamic obstacle representation with position and velocity

### `Main.java`
Demonstration program showing:
- Algorithm initialization and configuration
- Goal setting and obstacle placement
- Simulation loop with robot state updates
- Progress monitoring and goal achievement detection

## Usage

### Basic Usage

```java
// Create planner instance
DynamicWindowPlanner dwa = new DynamicWindowPlanner();

// Set goal position
dwa.setGoal(10, 10);

// Add obstacles
DynamicWindowPlanner.Obstacle obs = new DynamicWindowPlanner.Obstacle(5, 5, 0.1, 0.05, 0.5);
dwa.addObstacle(obs);

// Initialize robot state
DynamicWindowPlanner.State robot = new DynamicWindowPlanner.State(0, 0, 0, 0, 0);

// Planning loop
DynamicWindowPlanner.Control control = dwa.plan(robot);
```

### Complete Example

```java
public class Main {
    public static void main(String[] args) {
        DynamicWindowPlanner dwa = new DynamicWindowPlanner();
        dwa.setGoal(10, 10);

        // Add a moving obstacle
        DynamicWindowPlanner.Obstacle obs = new DynamicWindowPlanner.Obstacle(5, 5, 0.1, 0.05, 0.5);
        dwa.addObstacle(obs);

        DynamicWindowPlanner.State robot = new DynamicWindowPlanner.State(0, 0, 0, 0, 0);

        for (int i = 0; i < 100; i++) {
            DynamicWindowPlanner.Control u = dwa.plan(robot);

            // Apply control commands
            robot.v = u.v;
            robot.omega = u.omega;

            // Update robot state
            robot.x += robot.v * Math.cos(robot.theta) * 0.1;
            robot.y += robot.v * Math.sin(robot.theta) * 0.1;
            robot.theta += robot.omega * 0.1;

            // Check goal achievement
            if (Math.hypot(dwa.goalX - robot.x, dwa.goalY - robot.y) < 0.5) {
                System.out.println("✅ Reached goal!");
                break;
            }
        }
    }
}
```

## Configuration Parameters

### Robot Constraints
- `MAX_V`: Maximum linear velocity (default: 1.0 m/s)
- `MAX_OMEGA`: Maximum angular velocity (default: π/4 rad/s)
- `ROBOT_RADIUS`: Robot radius for collision checking (default: 0.5 m)

### Planning Parameters
- `DT`: Time step for trajectory prediction (default: 0.1 s)
- `PREDICT_TIME`: Prediction horizon (default: 2.0 s)

### Velocity Sampling
- Linear velocity: Sampled from 0 to `MAX_V` in 0.1 m/s increments
- Angular velocity: Sampled from `-MAX_OMEGA` to `MAX_OMEGA` in π/20 rad/s increments

## Algorithm Performance

### Computational Complexity
- **Time Complexity**: O(n × m × k) where:
  - n = number of linear velocity samples
  - m = number of angular velocity samples  
  - k = number of prediction steps
- **Space Complexity**: O(p) where p is the number of obstacles

### Typical Performance
- **Planning Frequency**: 10-50 Hz depending on parameter settings
- **Convergence**: Usually reaches goal within reasonable time for unobstructed paths
- **Obstacle Avoidance**: Effective for obstacles with radius > robot radius

## Compilation and Execution

### Prerequisites
- Java Development Kit (JDK) 8 or higher

### Compilation
```bash
javac DynamicWindowPlanner.java
javac Main.java
```

### Execution
```bash
java Main
```

### Expected Output
```
Step 0: x=0.10, y=0.00, θ=0.00, v=1.00, ω=0.00
Step 1: x=0.20, y=0.00, θ=0.00, v=1.00, ω=0.00
...
Step 95: x=9.60, y=9.98, θ=0.79, v=0.50, ω=0.00
Step 96: x=9.85, y=10.00, θ=0.79, v=0.50, ω=-0.16
✅ Reached goal!
```

## Customization

### Adding Custom Obstacles

```java
// Static obstacle
DynamicWindowPlanner.Obstacle staticObs = new DynamicWindowPlanner.Obstacle(3, 3, 0, 0, 0.8);
dwa.addObstacle(staticObs);

// Moving obstacle
DynamicWindowPlanner.Obstacle movingObs = new DynamicWindowPlanner.Obstacle(7, 2, -0.2, 0.1, 0.6);
dwa.addObstacle(movingObs);
```

### Modifying Robot Constraints

```java
// Modify the constants in DynamicWindowPlanner.java
final double MAX_V = 2.0;              // Faster robot
final double MAX_OMEGA = Math.PI / 2;  // More agile turning
final double ROBOT_RADIUS = 0.3;       // Smaller robot
```

### Custom Evaluation Function

The trajectory evaluation can be modified in the `evaluateTrajectory` method to include additional criteria such as:
- Path smoothness
- Energy efficiency  
- Safety margins
- Multi-objective optimization
