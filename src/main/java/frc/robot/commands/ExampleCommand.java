// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;



/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RomiDrivetrain subsystem;
    private final RomiGyro gyro = new RomiGyro();

    final double kP = 25;
    final double kI = 12;
    final double kD = 0.8;

    double heading;
    double errorSum;
    double lastTimeStamp;
    double lastError;

    double outputDiff;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ExampleCommand(RomiDrivetrain subsystem)
    {
        this.subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);


    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        heading = gyro.getAngleZ();
        subsystem.resetEncoders();

        errorSum = 0;
        lastError = 0;
        lastTimeStamp = Timer.getFPGATimestamp();

    }
    
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double error = heading - gyro.getAngleZ();

        double dt = Timer.getFPGATimestamp() - lastTimeStamp;
        double errorRate = (error - lastError) / dt;
        errorSum += error * dt;



        outputDiff = kP * error + kI * errorSum + kD * errorRate;
        // Drives forward continuously at half speed, using the gyro to stabilize the heading
        subsystem.tankDrive(0.8 - outputDiff, 0.8 + outputDiff);
    }
    
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.tankDrive(0,0);
    }
    
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subsystem.getAverageDistanceInch() > 36;
    }
}
