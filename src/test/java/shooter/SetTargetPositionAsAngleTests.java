import static org.junit.jupiter.api.Assertions.assertEquals;

import org.easymock.EasyMock;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.hardware.shooter.ShooterHardware;
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
        _shooter = new Shooter(_hardware);
    }

    void replayMocks() {
        // Put anything in here that is mocked
        EasyMock.replay(_subsystem, _hardware);
    }

    void verifyMocks() {
        // Put anything in here that is mocked
        EasyMock.verify(_subsystem, _hardware);
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {

    }

    // A typical test case with no exceptions or mocking
    @Test
    void testMethod_withInputs_expectOutputs() {
        // Set up test parameters
        double a = 0;
        int b = 1;

        // Set expected outputs
        boolean expected = false;

        // Run test
        assertEquals(expected, _subsystem.Foo(a, b));
    }

    // A test case with hardware mocking
    @Test
    void testMethod_withHardwareInputs_expectOutputs() {
        double throttle = -1;
        double rotate = -1;

        // Here is where we tell EasyMock our expected behavior for our sim hardware
        // This is called recording
        _hardware.getLeftPrimaryMotor().set(-1);
        _hardware.getRightPrimaryMotor().set(0);

        // Here we call a hardware method called replayHardware()
        // This causes EasyMock to run all of the stuff we just recorded
        replayMocks();

        // Here is where we call the method under test
        _subsystem.arcadeDrive(throttle, rotate);

        // Here is where we make assertions about behavior and call verifyHardware()
        // In this test, our assertions are handled by EasyMock, since
        // we tell it what we expect our motors to output.
        // So we will only call verifyHardware()
        verifyMocks();
    }

    @Test
    void testMethod_withMethodMocking_expectOutputs() {
        int c = 0;
        // this is a method we made already in the subsystem and mocked in the constructor for this class
        // We give it the expected parameters and tell it what to return
        // When we run the test, the system will expect this exact call to occur and will return what we told it to
        // If this exact call doesn't occur, the test will fail
        EasyMock.expect(_subsystem.foo(1, 3)).andReturn(true);

        int expected = 1;

        replayMocks();

        assertEquals(expected, _subsystem.foo3(c));
        verifyMocks();
    }
}
