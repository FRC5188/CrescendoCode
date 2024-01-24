package shooter;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import frc.robot.hardware.shooter.SimShooterHardware;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class SetTargetPositionAsAngleTests {
    // Define the subsystem and hardware you're testing with
    Shooter _shooter;
    SimShooterHardware _hardware;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        // Create our sim hardware and subsystem
        _hardware = new SimShooterHardware();
        _shooter = new Shooter(_hardware);
    }

    void replayMocks() {
        // Put anything in here that is mocked
        _hardware.replayHardware();
    }

    void verifyMocks() {
        // Put anything in here that is mocked
        _hardware.verifyHardware();
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {

    }

    // A test case with hardware mocking
    @Test
    void testSetShooterPositionAsAngle_withMinShooterAngle_expectPIDSetToMinShooterAngle() {
        double angle = ShooterConstants.MIN_SHOOTER_ANGLE;

        // Here is where we tell EasyMock our expected behavior for our sim hardware
        // This is called recording
        _hardware.getAnglePidController().setSetpoint(angle);

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _shooter.setTargetPositionAsAngle(angle);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }

    @Test
    void testSetShooterPositionAsAngle_withMaxShooterAngle_expectPIDSetToMaxShooterAngle() {
        double angle = ShooterConstants.MAX_SHOOTER_ANGLE;

        // Here is where we tell EasyMock our expected behavior for our sim hardware
        // This is called recording
        _hardware.getAnglePidController().setSetpoint(angle);

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _shooter.setTargetPositionAsAngle(angle);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }

    @Test
    void testSetShooterPositionAsAngle_withInvalidMinShooterAngle_expectPIDNoSet() {
        double angle = ShooterConstants.MIN_SHOOTER_ANGLE - 0.1;

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _shooter.setTargetPositionAsAngle(angle);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }

    @Test
    void testSetShooterPositionAsAngle_withInvalidMaxShooterAngle_expectPIDNoSet() {
        double angle = ShooterConstants.MAX_SHOOTER_ANGLE + 0.1;

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _shooter.setTargetPositionAsAngle(angle);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }
}
