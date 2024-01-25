package shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.easymock.EasyMock;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.intake.SimIntakeHardware;
import frc.robot.hardware.shooter.SimShooterHardware;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class GetCurrentPositionInDegreesTests {
    Shooter _shooter;
    SimShooterHardware _simulatedHardware;


    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        // Create our sim hardware and subsystem
        _simulatedHardware = new SimShooterHardware();
        _shooter = new Shooter(_simulatedHardware);
    }

    void replayMocks() {
        // Put anything in here that is mocked
        _simulatedHardware.replayHardware();
    }

    void verifyMocks() {
        // Put anything in here that is mocked
        _simulatedHardware.verifyHardware();
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {

    }

    @Test
    void testGetCurrentPositionInDegrees_withTurnsOutput_expectDegrees() {
        final double SIMULATED_POSITION_TURNS = 0.34;
        // Sets the simulated hardware to the value that we need it to be. 
        EasyMock.expect(_simulatedHardware.getAngleEncoder().get()).andReturn(SIMULATED_POSITION_TURNS);

        replayMocks(); // Run everything for EasyMock.

        // Expected output in degrees.
        final double EXPECTED_DEGREES_RESULT = Rotation2d.fromRotations(SIMULATED_POSITION_TURNS).getDegrees();

        assertEquals(EXPECTED_DEGREES_RESULT, _shooter.getCurrentPositionInDegrees());
        verifyMocks();
    }

    @Test
    void testGetCurrentPositionInDegrees_withImpossibleHighTurnOutput_expectRuntimeException() {
        final double SIMULATED_POSITION_TURNS = ShooterConstants.MAXIMUM_ANGLE_ENCODER_TURNS + 0.1;
        // Sets the simulated hardware to the value that we need it to be. 
        EasyMock.expect(_simulatedHardware.getAngleEncoder().get()).andReturn(SIMULATED_POSITION_TURNS);

        replayMocks(); // Run everything for EasyMock.

        assertThrows(RuntimeException.class, () -> _shooter.getCurrentPositionInDegrees());
        verifyMocks();
    }

        @Test
    void testGetCurrentPositionInDegrees_withImpossibleLowTurnOutput_expectRuntimeException() {
        final double SIMULATED_POSITION_TURNS = ShooterConstants.MINIMUM_ANGLE_ENCODER_TURNS - 0.1;
        // Sets the simulated hardware to the value that we need it to be. 
        EasyMock.expect(_simulatedHardware.getAngleEncoder().get()).andReturn(SIMULATED_POSITION_TURNS);

        replayMocks(); // Run everything for EasyMock.

        assertThrows(RuntimeException.class, () -> _shooter.getCurrentPositionInDegrees());
        verifyMocks();
    }

}
