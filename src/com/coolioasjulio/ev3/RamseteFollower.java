package com.coolioasjulio.ev3;

import trclib.TrcDriveBase;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;

public class RamseteFollower {
    private TrcDriveBase driveBase;
    private TrcTaskMgr.TaskObject driveTask;
    private double b, zeta;
    private double trackWidth;

    public RamseteFollower(String instanceName, TrcDriveBase driveBase, double trackWidth, double b, double zeta) {
        this.driveBase = driveBase;
        this.trackWidth = trackWidth;
        this.b = b;
        this.zeta = zeta;
        driveTask = TrcTaskMgr.getInstance().createTask(instanceName + ".driveTask", this::driveTask);
    }

    private void driveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode robotMode) {
        RamseteState state = getTargetState();
        transformState(state);

        double headingErr = state.targetHeading - state.heading;
        double xErr = state.targetX - state.x;
        double yErr = state.targetY - state.y;

        double vTerm1 = state.targetVel * Math.cos(headingErr);
        double vTerm2 = k(state.targetVel, state.targetRotVel) * (xErr * Math.cos(state.heading) + yErr * Math.sin(state.heading));
        double v = vTerm1 + vTerm2;

        double wTerm1 = state.targetRotVel;
        double wTerm2 = b * state.targetVel * (Math.sin(headingErr) / (headingErr)) * (yErr * Math.cos(state.heading) - xErr * Math.sin(state.heading));
        double wTerm3 = k(state.targetVel, state.targetRotVel) * headingErr;
        double w = wTerm1 + wTerm2 + wTerm3; // rad/sec

        double leftPower = v - w * trackWidth / 2;
        double rightPower = v + w * trackWidth / 2;
        driveBase.tankDrive(leftPower, rightPower);
    }

    /**
     * Transform the state into the coordinate system used by ramsete.
     * @param state
     */
    private void transformState(RamseteState state) {
        double temp = state.x;
        state.x = -state.y;
        state.y = temp;

        temp = state.targetX;
        state.targetX = -state.targetY;
        state.targetY = temp;

        state.targetHeading = -Math.toRadians(state.targetHeading);
        state.heading = -Math.toRadians(state.heading);
        state.targetRotVel = -Math.toRadians(state.targetRotVel);
        state.rotVel = -Math.toRadians(state.rotVel);
    }

    private double k(double v, double w) {
        return 2 * zeta * Math.sqrt(Math.pow(w, 2) + b * Math.pow(v, 2));
    }

    private RamseteState getTargetState() {
        throw new UnsupportedOperationException("Not implemented!");
    }

    private class RamseteState {
        private double targetVel, vel;
        private double targetRotVel, rotVel;
        private double targetX, x;
        private double targetY, y;
        private double targetHeading, heading;
    }
}
