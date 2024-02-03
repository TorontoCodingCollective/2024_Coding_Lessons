package frc.robot.operatorInput;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.Constants.OperatorInputConstants.AutoPattern;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.drive.DriveForTimeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightsSubsystem;

/**
 * Operator Input
 * <p>
 * This class is used to bind buttons to functions.
 * <p>
 * The operator input class should be passed into any command that requires operator input
 */
public class OperatorInput extends SubsystemBase {

    final XboxController               driverController = new XboxController(OperatorInputConstants.DRIVER_CONTROLLER_PORT);

    final SendableChooser<AutoPattern> autoChooser;

    public OperatorInput() {

        /*
         * Configure all selectors for the dashboard
         */
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        autoChooser.addOption("Drive Forward 1s", AutoPattern.DRIVE_FORWARD_ONE_SECOND);
    }

    /**
     * Use this method to define bindings of buttons to command
     */
    public void configureBindings(DriveSubsystem driveSubsystem, LightsSubsystem lightsSubsystem) {

        /*
         * Cancel Button
         */
        new Trigger(() -> getCancel())
            .onTrue(
                new CancelCommand(this, driveSubsystem));

        /*
         * Robot enabled - flash the RSL lights
         */
        new Trigger(() -> RobotController.isSysActive())
            .onTrue(
                new InstantCommand(() -> lightsSubsystem.setRobotEnabled()));
        /*
         * Reset Gyro
         */
        new Trigger(() -> driverController.getBackButton())
            .onTrue(
                new InstantCommand(() -> driveSubsystem.setHeading(0)));


        new Trigger(() -> driverController.getAButton())
            .onTrue(
                new DriveForTimeCommand(2, .2, driveSubsystem));
    }

    /*
     * Any command where operator input is required will need to get functional instructions from the controller
     */
    public double getLeftSpeed() {
        double speed = -driverController.getLeftY();
        return speed;
    }

    public double getRightSpeed() {
        double speed = -driverController.getRightY();
        return speed;
    }

    public boolean getCancel() {
        return driverController.getStartButton();
    }

    public AutoPattern getSelectedAuto() {
        return autoChooser.getSelected();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
