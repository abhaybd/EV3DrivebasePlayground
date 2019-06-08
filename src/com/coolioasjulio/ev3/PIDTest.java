package com.coolioasjulio.ev3;

import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcStopwatch;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

import java.util.Arrays;

public class PIDTest {
    public static void main(String[] args) throws Exception{
        Ev3Motor leftMotor = new Ev3Motor("Left", Ev3Motor.Port.B);
        Ev3Motor rightMotor = new Ev3Motor("Right", Ev3Motor.Port.C);

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        DifferentialDriveBase driveBase = new DifferentialDriveBase(leftMotor, rightMotor, RobotInfo.WHEELBASE_WIDTH, RobotInfo.INCHES_PER_TICK);
        driveBase.resetOdometry(true, true);
        driveBase.setOdometryEnabled(true);

        TrcPidController turnPid = new TrcPidController("turnpid", new TrcPidController.PidCoefficients(0.004), 2, driveBase::getHeading);
        turnPid.setAbsoluteSetPoint(true);
        turnPid.setTarget(0.0);


        Thread t = new Thread(() -> {
            double start = TrcUtil.getCurrentTime();
            while (!Thread.interrupted()) {
                Arrays.stream(TrcTaskMgr.TaskType.values()).skip(2).limit(4).forEach(e -> TrcTaskMgr.getInstance().executeTaskType(e, TrcRobot.RunMode.TELEOP_MODE));
                System.out.printf("%.3f - %.0f, %.0f - x=%.1f,y=%.1f,rot=%.1f\n", TrcUtil.getCurrentTime() - start,
                        leftMotor.getMotorPosition(), rightMotor.getMotorPosition(),
                        driveBase.getRawXPosition(), driveBase.getRawYPosition(), driveBase.getHeading());
                driveBase.arcadeDrive(0.4, turnPid.getOutput());
            }
        });
        t.start();
        System.in.read();
        t.interrupt();
        t.join();
        driveBase.stop();
        System.out.println("Done!");
        System.out.println(driveBase.getRawYPosition());
        System.out.println(TrcUtil.average(leftMotor.getMotorPosition(), rightMotor.getMotorPosition()));
    }
}
