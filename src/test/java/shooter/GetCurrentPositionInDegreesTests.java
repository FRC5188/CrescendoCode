package shooter;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import edu.wpi.first.hal.HAL;
import frc.robot.hardware.shooter.SimShooterHardware;
import frc.robot.subsystems.shooter.Shooter;

public class GetCurrentPositionInDegreesTests {
    Shooter _shooter;
    SimShooterHardware _simulatedHardware;


    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        // Create our sim hardware and subsystem
        _simulatedHardware = new SimShooterHardware();
        //_shooter = new Shooter(_simulatedHardware);
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

    // @Test
    // void testGetCurrentPositionInDegrees_withTurnsOutput_expectDegrees() {
    //     final double simulatedPositionTurns = 0.34;
    //     // Sets the simulated hardware to the value that we need it to be. 
    //     EasyMock.expect(_simulatedHardware.getAngleEncoder().get()).andReturn(simulatedPositionTurns);

    //     replayMocks(); // Run everything for EasyMock.

    //     // Expected output in degrees.
    //     final double expectedDegreesResult = Rotation2d.fromRotations(simulatedPositionTurns).getDegrees();

    //     assertEquals(expectedDegreesResult, _shooter.getCurrentPositionInDegrees());
    //     verifyMocks();
    // }

    // @Test
    // void testGetCurrentPositionInDegrees_withImpossibleHighTurnOutput_expectRuntimeException() {
    //     final double simulatedPositionTurns = ShooterConstants.MAXIMUM_ANGLE_ENCODER_TURNS + Rotation2d.fromDegrees(10).getRotations();
    //     // Sets the simulated hardware to the value that we need it to be. 
    //     EasyMock.expect(_simulatedHardware.getAngleEncoder().get()).andReturn(simulatedPositionTurns);

    //     replayMocks(); // Run everything for EasyMock.

    //     assertThrows(RuntimeException.class, () -> _shooter.getCurrentPositionInDegrees());
    //     verifyMocks();
    // }

    // @Test
    // void testGetCurrentPositionInDegrees_withImpossibleLowTurnOutput_expectRuntimeException() {
    //     final double simulatedPositionTurns = ShooterConstants.MINIMUM_ANGLE_ENCODER_TURNS - Rotation2d.fromDegrees(10).getRotations();
    //     // Sets the simulated hardware to the value that we need it to be. 
    //     EasyMock.expect(_simulatedHardware.getAngleEncoder().get()).andReturn(simulatedPositionTurns);

    //     replayMocks(); // Run everything for EasyMock.

    //     assertThrows(RuntimeException.class, () -> _shooter.getCurrentPositionInDegrees());
    //     verifyMocks();
    // }

    // @Test
    // void testGetCurrentPositionInDegrees_withPossibleHighOutputOutsideBounds_expectDegrees() {
    //     final double simulatedPositionTurns = ShooterConstants.MAXIMUM_ANGLE_ENCODER_TURNS + Rotation2d.fromDegrees(5).getRotations();

    //     EasyMock.expect(_simulatedHardware.getAngleEncoder().get()).andReturn(simulatedPositionTurns);

    //     replayMocks(); // Run everything for EasyMock.

    //     // Expected output in degrees.
    //     final double expectedDegreesResult = Rotation2d.fromRotations(simulatedPositionTurns).getDegrees();

    //     assertEquals(expectedDegreesResult, _shooter.getCurrentPositionInDegrees());
    //     verifyMocks();
    // }

    // @Test
    // void testGetCurrentPositionInDegrees_withPossibleLowOutputOutsideBounds_expectDegrees() {
    //     final double simulatedPositionTurns = ShooterConstants.MINIMUM_ANGLE_ENCODER_TURNS - Rotation2d.fromDegrees(5).getRotations();

    //     EasyMock.expect(_simulatedHardware.getAngleEncoder().get()).andReturn(simulatedPositionTurns);

    //     replayMocks(); // Run everything for EasyMock.

    //     // Expected output in degrees.
    //     final double expectedDegreesResult = Rotation2d.fromRotations(simulatedPositionTurns).getDegrees();

    //     assertEquals(expectedDegreesResult, _shooter.getCurrentPositionInDegrees());
    //     verifyMocks();
    // }
}
