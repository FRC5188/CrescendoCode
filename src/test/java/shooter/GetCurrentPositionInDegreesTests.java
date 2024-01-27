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
}
