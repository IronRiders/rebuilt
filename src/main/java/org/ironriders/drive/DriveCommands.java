package org.ironriders.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {
    private final DriveSubsystem driveSubsystem;

    public DriveCommands(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        driveSubsystem.publish("Reset odometry red tower", resetOdometryTo(new Pose2d((FieldPositions.Field.CENTER.getX() / 2.5) + FieldPositions.Field.CENTER.getX(), FieldPositions.Field.CENTER.getY(), new Rotation2d())));
        driveSubsystem.publish("Reset odometry blue tower", resetOdometryTo(new Pose2d((FieldPositions.Field.CENTER.getX() / 2.5), FieldPositions.Field.CENTER.getY(), new Rotation2d())));
        driveSubsystem.publish("Reset Gyro trench ", resetOdometryTo(new Pose2d(4.472,0.415, new Rotation2d(-Math.PI/2)), true));
        driveSubsystem.publish("Reset gyro", resetRotation());

        driveSubsystem.publish("Invert drive", Commands.runOnce(() -> driveSubsystem.switchDrive()));
        driveSubsystem.publish("Invert rotation", Commands.runOnce(() -> driveSubsystem.switchRotation()));
        driveSubsystem.publish(".5 Drive speed", Commands.runOnce(() -> driveSubsystem.setDriveSpeedModifer(.5d)));
        driveSubsystem.publish("1.0 Drive speed", Commands.runOnce(() -> driveSubsystem.setDriveSpeedModifer(1d)));

        driveSubsystem.publish("Set False reset pose with vision", setZeroingPoseWithVision(false));
        driveSubsystem.publish("Set True reset pose with vision", setZeroingPoseWithVision(true));

         driveSubsystem.publish("Pathfind to tower scoring", Commands.runOnce(() -> CommandScheduler.getInstance()
                .schedule(pathfindToPoseThenAimAt(
                        DriveSubsystem.getPose().nearest(FieldPositions.Zones.TOWER_SCORING_POINTS),
                        FieldPositions.get(ElementType.HUB).toPose2d()))));

        driveSubsystem.publish("Pathfind to trench scoring", Commands.runOnce(() -> CommandScheduler.getInstance()
                .schedule(pathfindToPoseThenAimAt(
                        DriveSubsystem.getPose().nearest(FieldPositions.Zones.TRENCH_SCORING_POINTS),
                        FieldPositions.get(ElementType.HUB).toPose2d()))));
    }


    public Command invertDrive(){
        return Commands.runOnce(()->driveSubsystem.switchDrive());
    }

     public Command invertRotation(){
        return Commands.runOnce(()->driveSubsystem.switchRotation());
    }

    public Command setDriveSpeedModifer(double speed){
        return Commands.run(()-> driveSubsystem.setDriveSpeedModifer(speed));
    }
    
    
    /**
     * Command to drive the robot given a supplier.
     * 
     * @param translation   A {@link Supplier} of {@link Translation2d}, the
     *                      translation to drive the robot in.
     * @param rotation      A {@link DoubleSupplier} for the rotation value.
     * @param fieldRelative A {@link BooleanSupplier} indicating if the drive is
     *                      field-relative.
     * @return A {@link Command} to drive the robot.
     */
    public Command drive(
            Supplier<Translation2d> translation, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
        return Commands.run(
                () -> {
                    DriveSubsystem.drive(
                            translation.get(), rotation.getAsDouble(),
                            fieldRelative.getAsBoolean());
                },
                driveSubsystem);
    }

    /**
     * Drive the robot given controller input. Corrects for alliance. Calls
     * {@link #drive}, converts inputTranslationX & inputTranslationX ->
     * {@link Translation2d}.
     * 
     * @param inputTranslationX DoubleSupplier, value from 0-1.
     * @param inputTranslationY DoubleSupplier, value from 0-1.
     * @param inputRotation     DoubleSupplier, value from 0-1.
     * @param fieldRelative     Whether the driving is field-relative.
     * @return A command to drive the robot.
     */
    public Command driveTeleop(
            DoubleSupplier inputTranslationX,
            DoubleSupplier inputTranslationY,
            DoubleSupplier inputRotation,
            boolean fieldRelative) {

        double invert = DriverStation.getAlliance().isEmpty()
                || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                        ? 1
                      : -1;

        return drive(
                () -> new Translation2d(inputTranslationX.getAsDouble(),
                        inputTranslationY.getAsDouble())
                        .times(DriveConstants.SWERVE_MAX_TRANSLATION_TELEOP * invert * driveSubsystem.getDriveSpeedModifer()),
                () -> inputRotation.getAsDouble() * DriveConstants.SWERVE_MAX_ANGULAR_TELEOP * invert* driveSubsystem.getDriveSpeedModifer(),
                () -> fieldRelative);
    }

    /**
     * Command to pathfind to a given position.
     */
    public Command pathfindToPose(Pose2d target) {
        return Commands.runOnce(() -> PathPlannerHelpers.pathfindToPose(target));
    }

    /**
     * Command to pathfind to the start of a given path then follow that path.
     */
    public Command pathfindThenFollowPath(PathPlannerPath path) {
        return Commands.runOnce(() -> PathPlannerHelpers.pathfindThenFollowPath(path));
    }

    /**
     * Command to pathfind to the start of a given path then follow the flipped
     * version of that path.
     */
    public Command pathfindThenFollowFlippedPath(PathPlannerPath path) {
        return Commands.runOnce(() -> PathPlannerHelpers.pathfindThenFollowFlippedPath(path));
    }

    /**
     * Command to figure out if the distance to the start point of the flipped
     * version of the provided path is closer than the normal version, and if so
     * follow the flipped version.
     * 
     * TODO: !Uses distance as the crow flies, not path distance to start point!
     */
    public Command pathfindThenFlipPathIfBetterThenFollow(PathPlannerPath path) {
        return Commands.runOnce(() -> PathPlannerHelpers.pathfindThenFlipPathIfBetterThenFollow(path));
    }

    /**
     * Command to pathfind to the start of a given path and then aim at a target.
     */
    public Command pathfindToPoseThenAimAt(Pose2d pose, Pose2d target) {
        return Commands.runOnce(() -> PathPlannerHelpers.pathfindToPoseThenAimAt(pose, target));
    }

    /**
     * Command to cancel the current pathfinding operation.
     */
    public Command cancelPathfind() {
        return Commands.runOnce(() -> PathPlannerHelpers.cancelPathfind());
    }

    /** Command to reset the swerve drive's measured rotation. */
    public Command resetRotation() {
        return Commands.runOnce(() -> DriveSubsystem.resetRotation());
    }

    /**
     * Command to reset the swerve drive's measured position and rotation to the
     * origin
     */
    public Command resetOdometry() {
        return Commands.runOnce(() -> DriveSubsystem.resetOdometry(new Pose2d()));
    }

    /**
     * Command to reset the swerve drive's measured position and rotation to a given
     * pose.
     */
    public Command resetOdometryTo(Pose2d pose) {
        return Commands.runOnce(() -> DriveSubsystem.resetOdometry(pose));
    }

    /**
     * Command to reset the swerve drive's measured position and rotation to a given
     * pose.
     */
    public Command resetOdometryTo(Pose2d pose, boolean setRotation) {
        return Commands.runOnce(() -> DriveSubsystem.resetOdometry(pose, setRotation));
    }

    public Command resetPID() {
        return Commands.runOnce(() -> DriveSubsystem.resetPID());
    }

    public Command setZeroingPoseWithVision(boolean enabled) {
        return Commands.runOnce(() -> DriveSubsystem.zeroingPoseWithVision(enabled));
    }
}