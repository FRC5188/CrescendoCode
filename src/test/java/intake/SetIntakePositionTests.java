package intake;

import org.easymock.EasyMock;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.hardware.intake.SimIntakeHardware;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class SetIntakePositionTests {
    // Define the subsystem and hardware you're testing with
    Intake _intake;
    SimIntakeHardware _hardware;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        // Create our sim hardware and subsystem
        _hardware = new SimIntakeHardware();
        _intake = EasyMock.partialMockBuilder(Intake.class)
                 .addMockedMethod("setIntakePositionWithAngle", Double.class)
                 .mock();
    }

    void replayMocks() {
        // Put anything in here that is mocked
        EasyMock.replay(_intake);
        _hardware.replayHardware();
    }

    void verifyMocks() {
        // Put anything in here that is mocked
        // Should probably always have _hardware.verifyHardware()
        // Only add EasyMock.verify(_subsystem) if you are mocking class methods
        EasyMock.verify(_intake);
        _hardware.verifyHardware();
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {

    }

    @Test
    void testSetIntakePosition_withGroundPickup_expectPIDSetpointGroundPickup() {
        IntakePosition inputPosition = IntakePosition.GroundPickup;

        // We expect this method to be called
        _intake.setIntakePositionWithAngle(IntakeConstants.INTAKE_GROUND_PICKUP_ANGLE);

        // Here we call a hardware method called replayMocks()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _intake.setIntakePosition(inputPosition);

        // Here is where we make assertions about behavior and call verifyMocks()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our code to call
        verifyMocks();
    }

    @Test
    void testSetIntakePosition_withSourcePickup_expectPIDSetpointSourcePickup() {
        IntakePosition inputPosition = IntakePosition.SourcePickup;

        // We expect this method to be called
        _intake.setIntakePositionWithAngle(IntakeConstants.INTAKE_SOURCE_PICKUP_ANGLE);

        // Here we call a hardware method called replayMocks()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _intake.setIntakePosition(inputPosition);

        // Here is where we make assertions about behavior and call verifyMocks()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our code to call
        verifyMocks();
    }

    @Test
    void testSetIntakePosition_withAmpScore_expectPIDSetpointAmpScore() {
        IntakePosition inputPosition = IntakePosition.AmpScore;

        // We expect this method to be called
        _intake.setIntakePositionWithAngle(IntakeConstants.INTAKE_AMP_SCORE_ANGLE);

        // Here we call a hardware method called replayMocks()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _intake.setIntakePosition(inputPosition);

        // Here is where we make assertions about behavior and call verifyMocks()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our code to call
        verifyMocks();
    }

    @Test
    void testSetIntakePosition_withSpeakerScore_expectPIDSetpointSpeakerScore() {
        IntakePosition inputPosition = IntakePosition.SpeakerScore;

        // We expect this method to be called
        _intake.setIntakePositionWithAngle(IntakeConstants.INTAKE_SPEAKER_SCORE_ANGLE);

        // Here we call a hardware method called replayMocks()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _intake.setIntakePosition(inputPosition);

        // Here is where we make assertions about behavior and call verifyMocks()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our code to call
        verifyMocks();
    }

    @Test
    void testSetIntakePosition_withStowed_expectPIDSetpointStowed() {
        IntakePosition inputPosition = IntakePosition.Stowed;

        // We expect this method to be called
        _intake.setIntakePositionWithAngle(IntakeConstants.INTAKE_STOWED_ANGLE);

        // Here we call a hardware method called replayMocks()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _intake.setIntakePosition(inputPosition);

        // Here is where we make assertions about behavior and call verifyMocks()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our code to call
        verifyMocks();
    }
}
