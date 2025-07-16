public class Main {
    public static void main(String[] args) {
        DynamicWindowPlanner dwa = new DynamicWindowPlanner();
        dwa.setGoal(10, 10);

        // Add a moving obstacle
       DynamicWindowPlanner.Obstacle obs1 = new DynamicWindowPlanner.Obstacle(3, 3, 0.1, 0.05, 0.5);
dwa.addObstacle(obs1);

DynamicWindowPlanner.Obstacle obs2 = new DynamicWindowPlanner.Obstacle(7, 8, -0.05, 0.1, 0.4);
dwa.addObstacle(obs2);

// Far obstacles (should be filtered out initially)
DynamicWindowPlanner.Obstacle obs3 = new DynamicWindowPlanner.Obstacle(20, 20, 0.01, 0.01, 0.4);
dwa.addObstacle(obs3);

DynamicWindowPlanner.Obstacle obs4 = new DynamicWindowPlanner.Obstacle(25, 5, -0.02, 0.03, 0.3);
dwa.addObstacle(obs4);

DynamicWindowPlanner.Obstacle obs5 = new DynamicWindowPlanner.Obstacle(-5, -5, 0.01, 0.01, 0.2);
dwa.addObstacle(obs5);

System.out.println("Added 5 test obstacles at various distances");

        DynamicWindowPlanner.State robot = new DynamicWindowPlanner.State(0, 0, 0, 0, 0);

       // Add timing variables before the loop
long totalPlanningTime = 0;
int planningCalls = 0;

// Modify the for loop
for (int i = 0; i < 100; i++) {
    // Measure planning time
    long startTime = System.nanoTime();
    DynamicWindowPlanner.Control u = dwa.plan(robot);
    long endTime = System.nanoTime();
    
    totalPlanningTime += (endTime - startTime);
    planningCalls++;

    robot.v = u.v;
    robot.omega = u.omega;

    robot.x += robot.v * Math.cos(robot.theta) * 0.1;
    robot.y += robot.v * Math.sin(robot.theta) * 0.1;
    robot.theta += robot.omega * 0.1;

    // Print progress and stats every 10 steps
    if (i % 10 == 0) {
        System.out.printf("Step %d: x=%.2f, y=%.2f, θ=%.2f, v=%.2f, ω=%.2f%n",
                i, robot.x, robot.y, robot.theta, robot.v, robot.omega);
        
        // Show filtering statistics
        dwa.printFilteringStats(robot);
        System.out.println();
    }

    if (Math.hypot(dwa.goalX - robot.x, dwa.goalY - robot.y) < 0.5) {
        System.out.println("✅ Reached goal!");
        break;
    }
}

// Add performance summary after the loop
System.out.println("=== Performance Statistics ===");
double avgPlanningTime = (totalPlanningTime / planningCalls) / 1_000_000.0; // Convert to milliseconds
System.out.printf("Average planning time: %.2f ms%n", avgPlanningTime);
System.out.printf("Total planning calls: %d%n", planningCalls);
    }
}
