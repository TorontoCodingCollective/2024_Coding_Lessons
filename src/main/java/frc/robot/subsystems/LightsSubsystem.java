package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {

    // Note: on Shifty, the lights are Y-ed, both of the strips run the same pattern.

    private final AddressableLED       ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBuffer RSL_ON;
    private final AddressableLEDBuffer RSL_OFF;


    private static final Color         TEST_COLOR    = new Color(30, 30, 30); // RGB
    private static final Color         RSL_COLOR     = new Color(255, 30, 0); // RGB

    // RSL tracker
    private boolean                    previousRSLOn = false;
    private int                        rslSyncCount  = -1;

    public LightsSubsystem() {

        ledStrip  = new AddressableLED(LightsConstants.LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);

        ledStrip.start();

        // Run the test pattern
        setColor(TEST_COLOR);

        // Set up the RSL buffers
        RSL_ON  = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);
        RSL_OFF = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

        for (int i = 0; i < LightsConstants.LED_STRING_LENGTH; i++) {
            RSL_ON.setLED(i, RSL_COLOR);
            RSL_OFF.setLED(i, Color.kBlack);
        }
    }

    public void setRobotEnabled() {
        // Flash the inverted color buffer 5 times when starting up.
        rslSyncCount = 4;
    }

    public void setDriveMotorSpeeds(double leftSpeed, double rightSpeed) {

        // If the robot is not enabled, then set the pattern to the test pattern

        if (!RobotController.isSysActive()) {
            for (int i = 0; i < 10; i++) {
                setPixel(i, TEST_COLOR);
            }
        }
        else {

            // Set the color of the first 5 leds to the left speed
            // green is positive, red is negative
            // intensity is based on speed.
            Color leftColor = null;

            if (leftSpeed > 0) {
                leftColor = new Color(0, leftSpeed, 0);
            }
            else {
                leftColor = new Color(leftSpeed, 0, 0);
            }
            for (int i = 0; i < 5; i++) {
                setPixel(i, leftColor);
            }

            Color rightColor = null;

            if (leftSpeed > 0) {
                rightColor = new Color(0, rightSpeed, 0);
            }
            else {
                rightColor = new Color(rightSpeed, 0, 0);
            }
            for (int i = 5; i < 10; i++) {
                setPixel(i, rightColor);
            }
        }
    }

    @Override
    public void periodic() {

        // Update the buffer every loop
        if (rslSyncCount < 0) {
            ledStrip.setData(ledBuffer);
        }
        else {
            flashRSL();
        }
    }

    private void setColor(Color color) {
        for (int pixel = 0; pixel < ledBuffer.getLength(); pixel++) {
            setPixel(pixel, color);
        }
    }

    private void setPixel(int pixel, Color color) {
        // Make sure the pixel is in the buffer to avoid an out of bounds exception
        if (pixel >= 0 && pixel < ledBuffer.getLength()) {

            ledBuffer.setLED(pixel, color);
        }
    }

    private void flashRSL() {

        // Sync with the first set of RSL flashes when the robot is first enabled
        boolean rslOn = RobotController.getRSLState();

        if (!rslOn && previousRSLOn) {
            rslSyncCount--;
        }
        previousRSLOn = rslOn;

        if (rslOn) {
            ledStrip.setData(RSL_ON);
        }
        else {
            ledStrip.setData(RSL_OFF);
        }
    }

}
