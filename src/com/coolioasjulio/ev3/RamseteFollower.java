package com.coolioasjulio.ev3;

import trclib.TrcDriveBase;
import trclib.TrcEvent;
import trclib.TrcPath;
import trclib.TrcRobot;
import trclib.TrcStopwatch;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;
import trclib.TrcWarpSpace;
import trclib.TrcWaypoint;

import java.util.Arrays;

public class RamseteFollower {

    public static void main(String[] args) {
        Ev3Motor leftMotor = new Ev3Motor("Left", Ev3Motor.Port.B);
        Ev3Motor rightMotor = new Ev3Motor("Right", Ev3Motor.Port.C);

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        DifferentialDriveBase driveBase = new DifferentialDriveBase(leftMotor, rightMotor, RobotInfo.WHEELBASE_WIDTH, RobotInfo.INCHES_PER_TICK);
        driveBase.resetOdometry(true, true);

        RamseteFollower follower = new RamseteFollower("Ramsete", driveBase, RobotInfo.WHEELBASE_WIDTH, 0.5, 0.2, RobotInfo.TOP_SPEED_INCHES, 1.0);

        TrcPath path = new TrcPath(true,
                new TrcWaypoint(0.1, 0, 0, 0.0, 0.0, 0, 0, 0),
                new TrcWaypoint(0.1, 0, 5, 0.0, 10, 0, 0, 0),
                new TrcWaypoint(0.1, 0, 25, 0.0, 10, 0, 0, 0),
                new TrcWaypoint(0.1, 0, 30, 0.0, 0.0, 0, 0, 0));
        path.inferTimeSteps();
        System.out.println("Path duration: " + path.getPathDuration());
        TrcEvent event = new TrcEvent("event");
        follower.start(path, event);
        double start = TrcUtil.getCurrentTime();
        while(!event.isSignaled()) {
            Arrays.stream(TrcTaskMgr.TaskType.values()).skip(2).limit(4).forEach(e -> TrcTaskMgr.getInstance().executeTaskType(e, TrcRobot.RunMode.TELEOP_MODE));
            //System.out.printf("%.3f - %.2f\n", TrcUtil.getCurrentTime() - start, driveBase.getYPosition());
        }
        driveBase.stop();
    }

    private TrcDriveBase driveBase;
    private TrcTaskMgr.TaskObject driveTask;
    private double b, zeta;
    private double trackWidth;
    private double topSpeed;
    private TrcStopwatch stopwatch;
    private TrcPath path;
    private TrcEvent onFinishedEvent;
    private TrcWarpSpace warpSpace;
    private double tolerance;

    public RamseteFollower(String instanceName, TrcDriveBase driveBase, double trackWidth, double b, double zeta, double topSpeed, double tolerance) {
        this.driveBase = driveBase;
        this.trackWidth = trackWidth;
        this.b = b;
        this.zeta = zeta;
        this.topSpeed = topSpeed;
        this.tolerance = tolerance;
        driveTask = TrcTaskMgr.getInstance().createTask(instanceName + ".driveTask", this::driveTask);
        stopwatch = new TrcStopwatch();
        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
    }

    public void start(TrcPath path, TrcEvent onFinishedEvent) {
        this.path = path.toRadians();
        setEnabled(true);
        if (onFinishedEvent != null) {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;
        stopwatch.start();
    }

    public void cancel() {
        stop();
        if (onFinishedEvent != null) {
            onFinishedEvent.cancel();
        }
    }

    private void setEnabled(boolean enabled) {
        if (enabled) {
            driveTask.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        } else {
            driveTask.unregisterTask();
        }
    }

    private double sinc(double x) {
        return x == 0 ? 1.0 : Math.sin(x) / x;
    }

    private void stop() {
        setEnabled(false);
        driveBase.stop();
    }

    private void driveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode robotMode) {
        RamseteState state = getTargetState();
        transformState(state);

        double headingErr = state.targetHeading - state.heading;
        double xErr = state.targetX - state.x;
        double yErr = state.targetY - state.y;

        if (stopwatch.getElapsedTime() >= path.getPathDuration() && TrcUtil.magnitude(xErr, yErr) <= tolerance) {
            stop();
            if (onFinishedEvent != null) {
                onFinishedEvent.set(true);
            }
        } else {
            double vTerm1 = state.targetVel * Math.cos(headingErr);
            double vTerm2 = k(state.targetVel, state.targetRotVel) * (xErr * Math.cos(state.heading) + yErr * Math.sin(state.heading));
            double v = vTerm1 + vTerm2;

            double wTerm1 = state.targetRotVel;
            double wTerm2 = b * state.targetVel * sinc(headingErr) * (yErr * Math.cos(state.heading) - xErr * Math.sin(state.heading));
            double wTerm3 = k(state.targetVel, state.targetRotVel) * headingErr;
            double w = wTerm1 + wTerm2 + wTerm3; // rad/sec

            double leftSpeed = v - w * trackWidth / 2;
            double rightSpeed = v + w * trackWidth / 2;
            System.out.printf("%.3f - x=%.2f,y=%.2f,rot=%.2f - v=%.2f, w=%.2f\n", stopwatch.getElapsedTime(), state.x, state.y, state.heading, v, w);
            driveBase.tankDrive(leftSpeed / topSpeed, rightSpeed / topSpeed);
        }
    }

    /**
     * Transform the state into the coordinate system used by ramsete.
     *
     * @param state
     */
    private void transformState(RamseteState state) {
        double temp = state.y;
        state.y = -state.x;
        state.x = temp;

        temp = state.targetY;
        state.targetY = -state.targetX;
        state.targetX = temp;

        state.targetHeading = -state.targetHeading;
        state.heading = -state.heading;
        state.targetRotVel = -state.targetRotVel;
        state.rotVel = -state.rotVel;
    }

    private double k(double v, double w) {
        return 2 * zeta * Math.sqrt(Math.pow(w, 2) + b * Math.pow(v, 2));
    }

    private RamseteState getTargetState() {
        double t = stopwatch.getElapsedTime();
        TrcWaypoint point = null, prev = null;
        int size = path.getSize();
        double weight = 1;
        if (t >= path.getPathDuration() - path.getWaypoint(size - 1).timeStep) {
            point = path.getWaypoint(size - 1);
            prev = path.getWaypoint(size - 2);
        } else {
            for (int i = 0; i < size - 1; i++) {
                double dt = path.getWaypoint(i).timeStep;
                if (t > dt) {
                    t -= dt;
                } else {
                    weight = t / dt;
                    prev = path.getWaypoint(i);
                    point = path.getWaypoint(i + 1);
                }
            }
            if (prev == null || point == null) {
                throw new IllegalStateException("Something just blew up!");
            }
        }

        double rotVel = (point.heading - prev.heading) / prev.timeStep;
        TrcWaypoint interpolated = interpolate(prev, point, weight);
        RamseteState state = waypointToState(interpolated);
        state.targetRotVel = rotVel;
        return state;
    }

    private RamseteState waypointToState(TrcWaypoint waypoint) {
        RamseteState state = new RamseteState();
        state.x = driveBase.getXPosition();
        state.targetX = waypoint.x;

        state.y = driveBase.getYPosition();
        state.targetY = waypoint.y;

        state.vel = TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());
        state.targetVel = waypoint.velocity;

        state.heading = Math.toRadians(driveBase.getHeading());
        state.targetHeading = waypoint.heading;

        state.rotVel = Math.toRadians(driveBase.getGyroTurnRate());
        return state;
    }

    private TrcWaypoint interpolate(TrcWaypoint point1, TrcWaypoint point2, double weight) {
        double timestep = interpolate(point1.timeStep, point2.timeStep, weight);
        double x = interpolate(point1.x, point2.x, weight);
        double y = interpolate(point1.y, point2.y, weight);
        double position = interpolate(point1.encoderPosition, point2.encoderPosition, weight);
        double velocity = interpolate(point1.velocity, point2.velocity, weight);
        double acceleration = interpolate(point1.acceleration, point2.acceleration, weight);
        double jerk = interpolate(point1.jerk, point2.jerk, weight);
        double heading = interpolate(point1.heading, warpSpace.getOptimizedTarget(point2.heading, point1.heading),
                weight);
        return new TrcWaypoint(timestep, x, y, position, velocity, acceleration, jerk, heading);
    }

    private double interpolate(double start, double end, double weight) {
        return (1.0 - weight) * start + weight * end;
    }

    private class RamseteState {
        private double targetVel, vel;
        private double targetRotVel, rotVel;
        private double targetX, x;
        private double targetY, y;
        private double targetHeading, heading;
    }
}
