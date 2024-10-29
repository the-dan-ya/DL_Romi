// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;

public class RomiDrivetrain extends SubsystemBase
{
    private static final double COUNTS_PER_REVOLUTION = 1440.0;
    private static final double WHEEL_DIAMETER_INCH = 2.75591; // 70 mm

    // The Romi has the left and right motors set to
    // PWM channel 0 and 1 respectively
    private final Spark leftMotor = new Spark(0);
    private final Spark rightMotor = new Spark(1);

    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder leftEncoder = new Encoder(4, 5);
    private final Encoder rightEncoder = new Encoder(6, 7);
    private final RomiGyro gyro = new RomiGyro();

    // Set up the differential drive controller
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);


    /** Creates a new RomiDrivetrain. */
    public RomiDrivetrain()
    {
        // Use inches as unit for encoder distances
        leftEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION);
        rightEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION);
        resetEncoders();

        leftMotor.setSafetyEnabled(false);
        rightMotor.setSafetyEnabled(false);
        diffDrive.setSafetyEnabled(false);

        // Invert right side since motor is flipped
        rightMotor.setInverted(true);
    }
    
    
    public void arcadeDrive(double xAxisSpeed, double zAxisRotate) {diffDrive.arcadeDrive(xAxisSpeed, zAxisRotate);}

    public void tankDrive(double leftSpeed, double rightSpeed) {diffDrive.tankDrive(leftSpeed,rightSpeed);}

    public Command driveTankAndStop(double leftSpeed, double rightSpeed){
        return runEnd(()-> tankDrive(leftSpeed,rightSpeed), () -> tankDrive(0,0));
    }

    private Command resetDrivetrain() {
        return run(() ->new InstantCommand(this::resetGyro).andThen(new InstantCommand(this::resetEncoders)));
    }

    public Command startDriving(double speed) {
        return run(() ->resetDrivetrain().andThen(() -> arcadeDrive(speed,0)));
    }

    public Command startTurning(double speed) {
        return run(() ->resetDrivetrain().andThen(() ->arcadeDrive(0,speed)));
    }
    public Command driveAndStop(double speed) {
        return runEnd(() -> startDriving(speed), () -> arcadeDrive(0, 0));
    }

    double kP = 1;
    double heading;

    public Command driveStraight(){
        return runEnd(gyro::reset, ()-> tankDrive(0,0));
    }

    public Command turnAndStop(double speed){
        return runEnd(() -> startTurning(speed), () -> arcadeDrive(0,0));
    }

    public Command driveForDistance(double distance, double speed) {
        return driveAndStop(speed).until(() -> getAverageDistanceInch() >= distance);
    }

    double inchPerDegree = Math.PI * 5.551 / 360;
    // Compare distance travelled from start to distance based on degree turn
    public Command turnForDegrees(double degrees, double speed) {
        return turnAndStop(speed).until(() -> gyro.getAngle()>=degrees);
    }

    public Command driveSquare(double distance, double speed) {
        return driveForDistance(distance,speed)
                .andThen(turnForDegrees(90,speed)
                        .andThen(driveForDistance(distance,speed))
                        .andThen(turnForDegrees(90,speed))
                        .andThen(driveForDistance(distance,speed))
                        .andThen(turnForDegrees(90,speed))
                        .andThen(driveForDistance(distance,speed)));

    }

    public Command driveTriangle(double distance, double speed) {
        return driveForDistance(distance,speed)
                .andThen(turnForDegrees(120,speed)
                        .andThen(driveForDistance(distance,speed))
                        .andThen(turnForDegrees(120,speed))
                        .andThen(driveForDistance(distance,speed)));
    }

    public Command driveCircle(double radius, double speed) {
        return resetDrivetrain().andThen(driveTankAndStop(speed * (radius/(radius - 5.55118)),speed)
                .until(() -> getLeftDistanceInch()>=2 * (radius) * Math.PI));
    }

    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }


    public void resetGyro(){ gyro.reset(); }

    public double getAngle(){
        return gyro.getAngle();
    }

    public double getLeftDistanceInch()
    {
        return leftEncoder.getDistance();
    }

    public double getRightDistanceInch()
    {
        return rightEncoder.getDistance();
    }

    public double getAverageDistanceInch() { return (leftEncoder.getDistance() + rightEncoder.getDistance()) /2.0;}
    
    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
