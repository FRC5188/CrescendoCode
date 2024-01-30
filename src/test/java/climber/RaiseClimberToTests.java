package climber;

import static org.junit.jupiter.api.Assertions.assertThrows;

import java.security.InvalidParameterException;

import org.easymock.EasyMock;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.hardware.climber.ClimberIO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;

public class RaiseClimberToTests {
    // Define the subsystem and hardware you're testing with
    Climber _climber;
    ClimberIO _io;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        // Create our sim hardware and subsystem
        _io = EasyMock.mock(ClimberIO.class);
        _climber = new Climber(_io, 0.0);
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
    void testSetClimberPosition_withInvalidLowHeight_expectInvalidParametersException() {
        assertThrows(
            InvalidParameterException.class,
            () -> _climber.setClimberPosition(-1)
        );
    }

    @Test 
    void testSetClimberPosition_withInvalidHighHeight_expectInvalidParametersException() {
        assertThrows(
            InvalidParameterException.class,
            () -> _climber.setClimberPosition(ClimberConstants.MAXIMUM_HEIGHT_OF_CLIMBERS + 0.1)
        );
    }

    @Test 
    void testSetClimberPosition_withValidHeight_expectValidHeightSet() {
        final double expectedResult  = ClimberConstants.MAXIMUM_HEIGHT_OF_CLIMBERS - 0.1;

        _io.setRightClimberPosition(expectedResult * ClimberConstants.DELTA_METERS_INTO_ROTATIONS);
        _io.setLeftClimberPosition(expectedResult * ClimberConstants.DELTA_METERS_INTO_ROTATIONS);

        replayMocks();

        try {
        _climber.setClimberPosition(expectedResult);
        } catch (Exception exception) {
             System.out.println("[TEST]: CLIMBER POSITION SETTING FAILED"); 
        }

        verifyMocks();
    }
}
