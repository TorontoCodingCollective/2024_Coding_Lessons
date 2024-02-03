// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    /*
     * Drive Motors
     */
    private final TalonSRX        leftMotor     = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT);
    private final TalonSRX        leftMotor2    = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT + 1);
    private final TalonSRX        rightMotor    = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT);
    private final TalonSRX        rightMotor2   = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT + 1);

    private double                leftSpeed     = 0;
    private double                rightSpeed    = 0;

    /*
     * Gyro
     */
    private AHRS                  navXGyro      = new AHRS();
    private double                headingOffset = 0;

    /*
     * Lights Subsystem
     */
    private final LightsSubsystem lightsSubsystem;

    /**
     * Place all initialization code in the constructor
     */
    public DriveSubsystem(LightsSubsystem lightsSubsystem) {

        this.lightsSubsystem = lightsSubsystem;

        /*
         * One of the sides always needs to be inverted
         */
        leftMotor.setInverted(true);
        leftMotor2.setInverted(true);
    }

    public void setDriveSpeed(double leftSpeed, double rightSpeed) {

        // Save the speeds
        this.leftSpeed  = leftSpeed;
        this.rightSpeed = rightSpeed;

        leftMotor.set(ControlMode.PercentOutput, leftSpeed);
        leftMotor2.set(ControlMode.PercentOutput, leftSpeed);

        rightMotor.set(ControlMode.PercentOutput, rightSpeed);
        rightMotor2.set(ControlMode.PercentOutput, rightSpeed);
    }

    /**
     * Safely stop the robot
     */
    public void stop() {
        setDriveSpeed(0, 0);
    }

    /**
     * Get the current heading from the Gyro
     */
    public double getHeading() {

        // The navX returns a yaw angle between -180 and 180 degrees
        // The heading should be zero to 360.
        // Adjust the yaw to the heading

        double yaw = navXGyro.getYaw();

        // Add the offset to the heading
        yaw = (yaw + headingOffset) % 360;

        if (yaw < 0) {
            yaw += 360;
        }

        // Round to one decimal place
        return Math.round(yaw * 10.0) / 10.0d;
    }

    public double getHeadingError(double desiredHeading) {

        // FIXME: calcuate the heading error based on the desired heading
        // by convention, when calculating the error for a PID feedback
        // loop the formula for error is:
        // error = desiredValue (setpoint) - actualValue (measurement)

        double error = desiredHeading - getHeading();

        // Modulo 360 makes sure the number is between -360 - +360

        error %= 360;

        // Return the error in the range -180 to +180 so the robot
        // knows the direction to turn.

        if (error > 180) {
            error -= 360;
        }

        if (error < -180) {
            error += 360;
        }

        return error;
    }

    /**
     * Set the heading to the supplied value
     * <p>
     * This routine allows the robot to be set up on the field facing a
     * known angle and to set the gyro to that angle in auto
     *
     * @param heading current heading of the robot
     */
    public void setHeading(double heading) {

        headingOffset = 0;
        headingOffset = heading - getHeading();
    }

    @Override
    public void periodic() {

        // Indicate the motor speeds using the lights
        lightsSubsystem.setDriveMotorSpeeds(leftSpeed, rightSpeed);

        SmartDashboard.putNumber("Gyro", getHeading());

        SmartDashboard.putNumber("Left Speed", leftSpeed);
        SmartDashboard.putNumber("Right Speed", rightSpeed);
    }

    /**
     * Returns the state of the subsystem in a human readable format
     */
    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        // Subsystem name
        sb.append(this.getClass().getSimpleName()).append(" : ");

        // Motor Speeds
        sb.append("(").append(leftSpeed).append(", ").append(rightSpeed).append(") ");

        // Heading
        sb.append("Heading ").append(getHeading());

        return sb.toString();
    }
}
