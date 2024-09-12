// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;


/** An example command that uses an example subsystem. */
public class DriveTriangle extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RomiDrivetrain subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    Command driveSquare = null;
    public DriveTriangle(RomiDrivetrain subsystem)
    {
        this.subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        driveSquare = new DriveDistance(0.6,6,subsystem).
                andThen(new TurnGyro(0.3, 120, subsystem))
                .andThen(new DriveDistance(0.6,6,subsystem))
                .andThen(new TurnGyro(0.3, 120, subsystem))
                .andThen(new DriveDistance(0.6,6,subsystem));

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveSquare.initialize();
    }
    
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveSquare.execute();
    }
    
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSquare.end(interrupted);
    }
    
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return driveSquare.isFinished();
    }
}
