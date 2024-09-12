// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {
  private final RomiDrivetrain drive;
  private final double distance;
  private final double speed;

  /**
   * Creates a new DriveDistance. This command will drive your  robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double inches, RomiDrivetrain drive) {
    distance = inches;
    this.speed = speed;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.arcadeDrive(0, 0);
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(drive.getAverageDistanceInch()) >= distance;
  }
}
