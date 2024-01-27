package shooter;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import frc.robot.hardware.shooter.SimShooterHardware;
import frc.robot.subsystems.shooter.Shooter;

public class SetTargetPositionAsAngleTests {
    // Define the subsystem and hardware you're testing with
    Shooter _shooter;
    SimShooterHardware _hardware;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        // Create our sim hardware and subsystem
        _hardware = new SimShooterHardware();
        //_shooter = new Shooter(_hardware);
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
}
