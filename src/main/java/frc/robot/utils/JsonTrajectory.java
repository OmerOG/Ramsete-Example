package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public enum JsonTrajectory {
    GET_ALL_TRENCH_BALLS("GetAllTrenchBalls"), 
    RETURN_TO_SHOOT("ReturnToShoot");

    private final String _filename;

    JsonTrajectory(String filename) {
        _filename = filename;
    }

    public Trajectory getTrajectoryFromJson() {
        Path filePath = getJsonFilePath();
        try {
            return TrajectoryUtil.fromPathweaverJson(filePath);
        } catch (IOException exception) {
            /**
             * In some rare cases, reading from a file can fail. We don't wanna let this
             * error crash the robot code when it happens, so we log the error and return an
             * empty trajectory.
             */
            DriverStation.reportError(String.format("Cannot get trajectory from json: %s\nException message: %s",
                    filePath.toString(), exception.getMessage()), false);
            return new Trajectory();
        }
    }

    private Path getJsonFilePath() {
        String relativePath = String.format("paths/%s.wpilib.json", _filename);
        Path jsonFilePath = Filesystem.getDeployDirectory().toPath().resolve(relativePath);
        return jsonFilePath;
    }
}
