// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorInputConstants.AutoPattern;
import frc.robot.commands.drive.DriveForTimeCommand;
import frc.robot.subsystems.DriveSubsystem;

public final class AutoCommand extends SequentialCommandGroup {

    public AutoCommand(AutoPattern autoPattern, DriveSubsystem driveSubsystem) {

        Alliance alliance = DriverStation.getAlliance().orElse(null);

        System.out.println();
        System.out.println("Auto Pattern :" + autoPattern);
        System.out.println("    Alliance :" + alliance);
        System.out.println();

        addCommands(new InstantCommand());

        switch (autoPattern) {

        case DO_NOTHING:
            return;

        case DRIVE_FORWARD_ONE_SECOND:
            addCommands(new DriveForTimeCommand(1, .2, driveSubsystem));
            return;
        }
    }
}
