// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.
 */
public final class Constants {

    public static class OperatorInputConstants {

        public enum AutoPattern {
            DO_NOTHING, DRIVE_FORWARD_ONE_SECOND
        };

        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class DriveConstants {
        public static final int LEFT_MOTOR_PORT  = 10;
        public static final int RIGHT_MOTOR_PORT = 20;
    }

    public static class LightsConstants {
        public static final int LED_STRING_LENGTH = 60;
        public static final int LED_PWM_PORT      = 9;
    }

}
