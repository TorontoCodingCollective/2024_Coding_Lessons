// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.DriveSubsystem;

/**
 * An example command that uses the drive subsystem.
 */
public class DefaultDriveCommand extends LoggingCommand {

    private final OperatorInput  operatorInput;
    private final DriveSubsystem driveSubsystem;

    /**
     * Default Drive command. This command runs when nothing else is running that
     * uses the drive subsystem.
     *
     * @param operatorInput
     * @param driveSubsystem
     */
    public DefaultDriveCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem) {

        this.operatorInput  = operatorInput;
        this.driveSubsystem = driveSubsystem;

        // Add required subsystems
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double rightSpeed = operatorInput.getRightSpeed();
        double leftSpeed  = operatorInput.getLeftSpeed();

        driveSubsystem.setDriveSpeed(leftSpeed, rightSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
