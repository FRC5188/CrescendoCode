package shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.easymock.EasyMock;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import frc.robot.hardware.shooter.SimShooterHardware;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter.ShooterPosition;

public class SetTargetPositionTests {
    // Define the subsystem and hardware you're testing with
    Shooter _shooter;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        // Create our sim hardware and subsystem
        _shooter = EasyMock.partialMockBuilder(Shooter.class)
                           .addMockedMethod("setTargetPositionAsAngle", Double.class)
                           .mock();
    }

    void replayMocks() {
        // Put anything in here that is mocked
        EasyMock.replay(_shooter);
    }

    void verifyMocks() {
        // Put anything in here that is mocked
        EasyMock.verify(_shooter);
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {

    }

    @Test
    void testSetShooterPosition_withSubwooferPosition_expectSetAngleSubwoofer() {
        ShooterPosition positon = ShooterPosition.Subwoofer;

        // Here is where we tell EasyMock our expected behavior for our sim hardware
        // This is called recording
        _shooter.setTargetPositionAsAngle(ShooterConstants.SUBWOOFER_ANGLE);

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _shooter.setTargetPosition(positon);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }

    @Test
    void testSetShooterPosition_withPodiumPosition_expectSetAnglePodium() {
        ShooterPosition positon = ShooterPosition.Podium;

        // Here is where we tell EasyMock our expected behavior for our sim hardware
        // This is called recording
        _shooter.setTargetPositionAsAngle(ShooterConstants.PODIUM_ANGLE);

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _shooter.setTargetPosition(positon);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }
}
