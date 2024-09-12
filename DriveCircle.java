// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveCircle extends Command {
  private final RomiDrivetrain drive;
  private final double leftDistance;
  private final double rightDistance;
  private final double speed;
  private final double outerSpeed;

  /**
   * Creates a new DriveDistance. This command will drive your  robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveCircle(double speed, double radius,RomiDrivetrain drive) {
    leftDistance = 2 * (radius) * Math.PI;
    rightDistance = 2 * (radius - 5.55118) * Math.PI;
    outerSpeed = speed * (radius/(radius - 5.55118));
    this.speed = speed;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.tankDrive(0, 0);
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.tankDrive(outerSpeed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(drive.getLeftDistanceInch())>=leftDistance;
    //Math.abs(drive.getLeftDistanceInch())>=leftDistance && Math.abs(drive.getRightDistanceInch())>=rightDistance
  }
}
