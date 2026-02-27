package org.ironriders.drive;

import java.io.IOException;
import java.util.Optional;

import org.ironriders.lib.Utils;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This is not a real subsystem, but more of a utility class to organize the
 * pathfinding functions neatly.
 */
public class PathPlannerHelpers {
    private static Command pathfindingCommand = Commands.none();

    private static void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }

    // -- Pathfinding --
    /*
     * Function to pathfind to a given pose.
     */
    public static void pathfindToPose(Pose2d target) {
        pathfindingCommand = AutoBuilder.pathfindToPose(target, DriveConstants.PATHFIND_CONSTRAINTS);
        schedule(pathfindingCommand);
    }

    /*
     * Function to pathfind to the start of a given path then follow that path.
     */
    public static void pathfindThenFollowPath(PathPlannerPath path) {
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, DriveConstants.PATHFIND_CONSTRAINTS);
        schedule(pathfindingCommand);
    }

    /*
     * Function to pathfind to the start of a given path then follow the flipped
     * version of that path.
     */
    public static void pathfindThenFollowFlippedPath(PathPlannerPath path) {
        path = path.flipPath();
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, DriveConstants.PATHFIND_CONSTRAINTS);
        schedule(pathfindingCommand);
    }

    /*
     * Function to figure out if the distance to the start point of the flipped
     * version of the provided path is closer than the normal version, and if so
     * follow the flipped version.
     * 
     * TODO: !Uses distance as the crow flies, not path distance to start point!
     */
    public static void pathfindThenFlipPathIfBetterThenFollow(PathPlannerPath path) {
        Double normalDistance = distanceToPose2d(path.getPathPoses().get(0), DriveSubsystem.getPose());

        // Flip path modifies the path it is called on...
        Double flippedDistance = distanceToPose2d(path.flipPath().getPathPoses().get(0), DriveSubsystem.getPose());

        if (normalDistance < flippedDistance) {
            // so we need to call it again here which is odd.
            // TOOD: make this more intuitive.
            path = path.flipPath();
        }

        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, DriveConstants.PATHFIND_CONSTRAINTS);
    }

    public static void pathfindToPoseThenAimAt(Pose2d pose, Pose2d target) {
        pathfindingCommand = AutoBuilder.pathfindToPose(
                new Pose2d(pose.getTranslation(), new Rotation2d(Utils.getAngleToPointRadians(pose, target) + Math.PI)),
                DriveConstants.PATHFIND_CONSTRAINTS);
    
        schedule(pathfindingCommand);
    }

    /*
     * Cancel the current pathfinding operation.
     */
    public static void cancelPathfind() {
        pathfindingCommand.cancel();
    }

    // -- Utils --
    /**
     * Load a PathPlanner path file.
     * 
     * @param fileName The name of the path.
     * 
     * @return An Optional containing the path, or an empty Optional if there was a
     *         error.
     */
    public static Optional<PathPlannerPath> loadPath(String fileName) {
        try {
            return Optional.of(PathPlannerPath.fromPathFile(fileName));
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.printf("Error loading path %s: ", fileName);
            e.printStackTrace();
            return Optional.empty();
        }
    }

    /*
     * Get the distance between two input points.
     */
    public static double distanceToPose2d(Pose2d p1, Pose2d p2) {
        return Math.sqrt(Math.pow(p2.getX() - p1.getX(), 2) + Math.pow(p2.getY() - p1.getY(), 2));
    }

    /**
     * Should we flip a path to to other side of the field?
     * Checks if we are on the Blue Alliance and if so returns false. (Flip on Red)
     */
    public static Boolean shouldFlip() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
