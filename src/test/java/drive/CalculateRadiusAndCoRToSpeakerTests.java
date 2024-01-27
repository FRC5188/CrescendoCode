package drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class CalculateRadiusAndCoRToSpeakerTests {

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {

    }

    @Test
    void testGetRadiusToSpeakerInMeters_withBlueSpeakerYLessThan_expectCorrectMath() {
        Pose2d speakerPose = DriveConstants.BLUE_SPEAKER;
        Pose2d robotPose = new Pose2d(2.22, 4.35, new Rotation2d(0));
        double tolerance = 0.1;

        // Here is where we call the method under test
        assertEquals(2.556, Drive.getRadiusToSpeakerInMeters(robotPose, speakerPose), tolerance);
    }

    @Test
    void testGetRadiusToSpeakerInMeters_withBlueSpeakerYGreaterThan_expectCorrectMath() {
        Pose2d speakerPose = DriveConstants.BLUE_SPEAKER;
        Pose2d robotPose = new Pose2d(2.22, 6.9, new Rotation2d(0));
        double tolerance = 0.1;

        // Here is where we call the method under test
        assertEquals(2.632, Drive.getRadiusToSpeakerInMeters(robotPose, speakerPose), tolerance);
    }

    @Test
    void testGetRadiusToSpeakerInMeters_withRedSpeakerYLessThan_expectCorrectMath() {
        Pose2d speakerPose = DriveConstants.RED_SPEAKER;
        Pose2d robotPose = new Pose2d(14.4, 4.5, new Rotation2d(0));
        double tolerance = 0.1;

        // Here is where we call the method under test
        assertEquals(2.419, Drive.getRadiusToSpeakerInMeters(robotPose, speakerPose), tolerance);
    }

    @Test
    void testGetRadiusToSpeakerInMeters_withRedSpeakerYGreaterThan_expectCorrectMath() {
        Pose2d speakerPose = DriveConstants.RED_SPEAKER;
        Pose2d robotPose = new Pose2d(14.4, 6.5, new Rotation2d(0));
        double tolerance = 0.1;

        // Here is where we call the method under test
        assertEquals(2.379, Drive.getRadiusToSpeakerInMeters(robotPose, speakerPose), tolerance);
    }

    @Test
    void testCalcSpeakerCoR_withBlueSpeakerYLessThan_expectCorrectMath() {
        Pose2d speakerPose = DriveConstants.BLUE_SPEAKER;
        Pose2d robotPose = new Pose2d(2.22, 4.35, new Rotation2d(Math.PI));
        double expectedX = 2.252;
        double expectedY = 1.198;
        double tolerance = 0.1;

        // Here is where we call the method under test
        Translation2d actual = Drive.calcSpeakerCoRForBlue(robotPose, speakerPose);
        assertEquals(expectedX, actual.getX(), tolerance);
        assertEquals(expectedY, actual.getY(), tolerance);
    }

    @Test
    void testCalcSpeakerCoR_withBlueSpeakerYGreaterThan_expectCorrectMath() {
        Pose2d speakerPose = DriveConstants.BLUE_SPEAKER;
        Pose2d robotPose = new Pose2d(2.22, 6.9, new Rotation2d(Math.PI));
        double expectedX = 2.252;
        double expectedY = -1.352;
        double tolerance = 0.1;

        // Here is where we call the method under test
        Translation2d actual = Drive.calcSpeakerCoRForBlue(robotPose, speakerPose);
        assertEquals(expectedX, actual.getX(), tolerance);
        assertEquals(expectedY, actual.getY(), tolerance);
    }

    @Test
    void testCalcSpeakerCoR_withRedSpeakerYLessThan_expectCorrectMath() {
        Pose2d speakerPose = DriveConstants.RED_SPEAKER;
        Pose2d robotPose = new Pose2d(14.4, 4.5, new Rotation2d(0));
        double expectedX = 2.17;
        double expectedY = 1.048;
        double tolerance = 0.1;

        // Here is where we call the method under test
        Translation2d actual = Drive.calcSpeakerCoRForRed(robotPose, speakerPose);
        assertEquals(expectedX, actual.getX(), tolerance);
        assertEquals(expectedY, actual.getY(), tolerance);
    }

    @Test
    void testCalcSpeakerCoR_withRedSpeakerYGreaterThan_expectCorrectMath() {
        Pose2d speakerPose = DriveConstants.RED_SPEAKER;
        Pose2d robotPose = new Pose2d(14.4, 6.5, new Rotation2d(0));
        double expectedX = 2.17;
        double expectedY = -0.952;
        double tolerance = 0.1;

        // Here is where we call the method under test
        Translation2d actual = Drive.calcSpeakerCoRForRed(robotPose, speakerPose);
        assertEquals(expectedX, actual.getX(), tolerance);
        assertEquals(expectedY, actual.getY(), tolerance);
    }
}
