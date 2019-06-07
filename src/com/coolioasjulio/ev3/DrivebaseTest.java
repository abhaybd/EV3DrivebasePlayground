package com.coolioasjulio.ev3;

import trclib.TrcUtil;

public class DrivebaseTest {
    public static void main(String[] args) throws Exception {
        Ev3Motor leftMotor = new Ev3Motor("Left", Ev3Motor.Port.B);
        Ev3Motor rightMotor = new Ev3Motor("Right", Ev3Motor.Port.C);

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        DifferentialDriveBase driveBase = new DifferentialDriveBase(leftMotor, rightMotor, 1.0, 1.0);
        driveBase.setOdometryEnabled(true);
        driveBase.arcadeDrive(0.4, 0.0);
        Thread t = new Thread(() -> {
            double start = TrcUtil.getCurrentTime();
            while(!Thread.interrupted()) {
                System.out.printf("%.3f, %.0f\n", TrcUtil.getCurrentTime() - start, driveBase.getRawYPosition());
            }
        });
        t.start();
        System.in.read();
        t.interrupt();
        t.join();
        driveBase.stop();
        Ev3Brick.getDefaultBrick().close();
    }
}
