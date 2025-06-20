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

            robot.v = u.v;
            robot.omega = u.omega;

            robot.x += robot.v * Math.cos(robot.theta) * 0.1;
            robot.y += robot.v * Math.sin(robot.theta) * 0.1;
            robot.theta += robot.omega * 0.1;

            System.out.printf("Step %d: x=%.2f, y=%.2f, θ=%.2f, v=%.2f, ω=%.2f%n",
                    i, robot.x, robot.y, robot.theta, robot.v, robot.omega);

            if (Math.hypot(dwa.goalX - robot.x, dwa.goalY - robot.y) < 0.5) {
                System.out.println("✅ Reached goal!");
                break;
            }
        }
    }
}
