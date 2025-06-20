import java.util.*;

public class DynamicWindowPlanner {
    // --- Robot and planning parameters ---
    final double MAX_V = 1.0;              // max linear velocity (m/s)
    final double MAX_OMEGA = Math.PI / 4;  // max angular velocity (rad/s)
    final double DT = 0.1;                 // time step (s)
    final double PREDICT_TIME = 2.0;       // prediction time (s)
    final double ROBOT_RADIUS = 0.5;       // robot radius (m)

    // --- Environment ---
    List<Obstacle> obstacles;
    double goalX, goalY;

    public DynamicWindowPlanner() {
        obstacles = new ArrayList<>();
    }

    public void setGoal(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public void addObstacle(Obstacle o) {
        obstacles.add(o);
    }

    // Main planning method: returns best control (v, omega)
    public Control plan(State state) {
        double bestScore = -Double.MAX_VALUE;
        Control bestControl = new Control(0, 0);

        for (double v = 0; v <= MAX_V; v += 0.1) {
            for (double omega = -MAX_OMEGA; omega <= MAX_OMEGA; omega += Math.PI / 20) {
                double score = evaluateTrajectory(state, v, omega);
                if (score > bestScore) {
                    bestScore = score;
                    bestControl = new Control(v, omega);
                }
            }
        }

        return bestControl;
    }

    // Predict trajectory and evaluate its cost
    private double evaluateTrajectory(State state, double v, double omega) {
        double x = state.x;
        double y = state.y;
        double theta = state.theta;

        for (double t = 0; t < PREDICT_TIME; t += DT) {
            // Predict robot position
            x += v * Math.cos(theta) * DT;
            y += v * Math.sin(theta) * DT;
            theta += omega * DT;

            // Check collision with each obstacle
            for (Obstacle obs : obstacles) {
                double ox = obs.x + obs.velocityX * t;
                double oy = obs.y + obs.velocityY * t;
                double dist = Math.hypot(ox - x, oy - y);
                if (dist < ROBOT_RADIUS + obs.radius) {
                    return -1000; // collision penalty
                }
            }
        }

        // Final position's distance to goal (the closer, the better)
        double goalDist = Math.hypot(goalX - x, goalY - y);
        return -goalDist; // lower distance = higher score
    }

    // --- Support Classes ---

    // Robot state at any time
    public static class State {
        public double x, y, theta, v, omega;

        public State(double x, double y, double theta, double v, double omega) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.v = v;
            this.omega = omega;
        }
    }

    // Command to apply
    public static class Control {
        public double v, omega;

        public Control(double v, double omega) {
            this.v = v;
            this.omega = omega;
        }
    }

    // Dynamic obstacle class
    public static class Obstacle {
        public double x, y;
        public double velocityX, velocityY;
        public double radius;

        public Obstacle() {}

        public Obstacle(double x, double y, double velocityX, double velocityY, double radius) {
            this.x = x;
            this.y = y;
            this.velocityX = velocityX;
            this.velocityY = velocityY;
            this.radius = radius;
        }
    }
}
