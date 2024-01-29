package intake;

import org.easymock.EasyMock;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.hardware.intake.IntakeIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class SetIntakePositionTests {
    // Define the subsystem and hardware you're testing with
    Intake _intake;
    IntakeIO _io;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        // Create our sim hardware and subsystem
        _io = EasyMock.mock(IntakeIO.class);
        _intake = new Intake(_io);
    }

    void replayMocks() {
        // Put anything in here that is mocked
        EasyMock.replay(_io);
    }

    void verifyMocks() {
        // Put anything in here that is mocked
        EasyMock.verify(_io);
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {

    }

    @Test 
    void testSetIntakePosition_withGroundPickup_expectGroundPickup() {
        _io.setTargetPositionAsDegrees(IntakeConstants.INTAKE_GROUND_PICKUP_ANGLE);

        replayMocks();

        _intake.setIntakePosition(IntakePosition.GroundPickup);

        verifyMocks();
    }

    @Test
    void testSetIntakePosition_withStowed_expectStowed() {
        _io.setTargetPositionAsDegrees(IntakeConstants.INTAKE_STOWED_ANGLE);

        replayMocks();

        _intake.setIntakePosition(IntakePosition.Stowed);

        replayMocks();
    }
}
