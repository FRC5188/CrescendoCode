package intake;

import org.easymock.EasyMock;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;

public class SetIntakePositionWithAngleTests {
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
    // TODO: Uncomment these tests after we move to onboard PIDs
    // @Test
    // void testSetIntakePositionWithAngle_withMinAngle_expectPIDSetpointSet() {
    //     double inputAngle = IntakeConstants.MIN_INTAKE_ANGLE;

    //     // We expect this method to set the PID's setpoint to this value
    //     _io.setTargetPositionAsDegrees(IntakeConstants.MIN_INTAKE_ANGLE);

    //     // Here we call a hardware method called replayHardware()
    //     // This causes EasyMock to run all of the stuff we just recorded
    //     replayMocks();

    //     // Here is where we call the method under test
    //     _intake.setIntakePositionWithAngle(inputAngle);

    //     // Here is where we make assertions about behavior and call verifyHardware()
    //     // In this test, our assertions are handled by EasyMock, since
    //     // we tell it what we expect our motors to output.
    //     // So we will only call verifyHardware()
    //     verifyMocks();
    // }

    // @Test
    // void testSetIntakePositionWithAngle_withMaxAngle_expectPIDSetpointSet() {
    //     double inputAngle = IntakeConstants.MAX_INTAKE_ANGLE;

    //     // We expect this method to set the PID's setpoint to this value
    //     _io.setTargetPositionAsDegrees(IntakeConstants.MAX_INTAKE_ANGLE);

    //     // Here we call a hardware method called replayHardware()
    //     // This causes EasyMock to run all of the stuff we just recorded
    //     replayMocks();

    //     // Here is where we call the method under test
    //     _intake.setIntakePositionWithAngle(inputAngle);

    //     // Here is where we make assertions about behavior and call verifyHardware()
    //     // In this test, our assertions are handled by EasyMock, since
    //     // we tell it what we expect our motors to output.
    //     // So we will only call verifyHardware()
    //     verifyMocks();
    // }

    @Test
    void testSetIntakePositionWithAngle_withInvalidMaxAngle_expectPIDSetpointToNotChange() {
        double inputAngle = IntakeConstants.MAX_INTAKE_ANGLE + 0.01;

        // We expect this method to not set the PID's setpoint

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _intake.setIntakePositionWithAngle(inputAngle);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }

    @Test
    void testSetIntakePositionWithAngle_withInvalidMinAngle_expectPIDSetpointToNotChange() {
        double inputAngle = IntakeConstants.MIN_INTAKE_ANGLE - 0.01;

        // We expect this method to not set the PID's setpoint

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _intake.setIntakePositionWithAngle(inputAngle);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }
}
