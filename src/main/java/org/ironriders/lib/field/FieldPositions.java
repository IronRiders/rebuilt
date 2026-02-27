package org.ironriders.lib.field;

import java.util.ArrayList;
import java.util.List;

import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.Zone.ZoneType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/*
 * Class with positions of elements on the field.
 */
public class FieldPositions {
    /*
     * Gets the position in meters of the specified element type. Automatically does
     * alliance mirroring.
     * <hr />
     * !Currently only HUB, TOWER and OUTPOST are implemented, all other types will
     * return a
     * blank Pose3d()!
     */
    public static Pose3d get(ElementType element) {
        switch (element) {
            case HUB:
                return prepareInchesPose(Hub.HUB_CENTER);

            case TOWER:
                return prepareInchesPose(Tower.TOWER_CENTER);

            case OUTPOST:
                return prepareInchesPose(Outpost.OUTPOST_CENTER);

            default:
                break;
        }

        return new Pose3d();
    }

    /**
     * Prepares a Pose2d for use by the robot, calling
     * {@link #preparePose(Pose2d, boolean) preparePose} with isMeters = false.
     * 
     * @param pose The pose to prepare, in inches
     * @return The prepared pose, in inches
     */
    public static Pose2d prepareInchesPose(Pose2d pose) {
        return preparePose(pose, false);
    }

    /**
     * Prepares a Pose3d for use by the robot, calling
     * {@link #preparePose(Pose2d, boolean) preparePose} on the flattened
     * {@link Pose3d}.
     * 
     * @param pose The {@link Pose3d} to prepare, in inches (will be flattened
     *             to a {@link Pose2d} for alliance mirroring, but the z value
     *             will be preserved and converted to meters)
     * @return The prepared {@link Pose3d}, in meters
     */
    public static Pose3d prepareInchesPose(Pose3d pose) {
        Pose2d flipped = prepareInchesPose(pose.toPose2d());
        return new Pose3d(flipped.getX(), flipped.getY(), Units.inchesToMeters(pose.getZ()),
                new Rotation3d(flipped.getRotation()));
    }

    /**
     * Prepares a Pose2d for use by the robot, calling
     * {@link #preparePose(Pose2d, boolean) preparePose}.
     * 
     * @param pose The pose to prepare, in meters
     * @return The prepared pose, in meters
     */
    public static Pose2d prepareMetersPose(Pose2d pose) {
        return preparePose(pose, true);
    }

    /**
     * Prepares a Pose3d for use by the robot, calling
     * {@link #preparePose(Pose2d, boolean) preparePose} on the flattened
     * {@link Pose3d}.
     * 
     * @param pose The {@link Pose3d} to prepare, in meters (will be flattened
     *             to a {@link Pose2d} for alliance mirroring)
     * @return The prepared {@link Pose3d}, in meters
     */
    public static Pose3d prepareMetersPose(Pose3d pose) {
        Pose2d flipped = prepareMetersPose(pose.toPose2d());
        return new Pose3d(flipped.getX(), flipped.getY(), pose.getZ(), new Rotation3d(flipped.getRotation()));
    }

    /**
     * Prepares a Pose2d for use by the robot, doing alliance mirroring and unit
     * conversion as necessary.
     * 
     * @param pose     The pose to prepare, in either inches or meters depending on
     *                 the value of isMeters
     * @param isMeters Whether the pose is in meters
     * @return The prepared pose, in meters
     */
    private static Pose2d preparePose(Pose2d pose, boolean isMeters) {
        boolean blue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        if (blue) {
            if (!isMeters) {
                pose = Utils.inchesToMeters(pose);
            }

            return pose;
        }

        Translation2d t = pose.getTranslation();
        Rotation2d r = pose.getRotation();
        Translation2d flippedTranslation;

        if (!isMeters) {
            flippedTranslation = new Translation2d(
                    Field.FIELD_LENGTH - t.getX(),
                    Field.FIELD_WIDTH - t.getY());
        } else {
            flippedTranslation = new Translation2d(
                    Units.inchesToMeters(Field.FIELD_LENGTH) - t.getX(),
                    Units.inchesToMeters(Field.FIELD_WIDTH) - t.getY());
        }

        Rotation2d flippedRotation = new Rotation2d(r.getDegrees()); // TODO

        if (!isMeters) {
            return Utils.inchesToMeters(new Pose2d(flippedTranslation, flippedRotation));
        }
        return new Pose2d(flippedTranslation, flippedRotation);
    }

    /**
     * calls {@link #preparePose(Pose2d, boolean) preparePose} on each
     * {@link Pose2d} in {@code Pose2d[] input}
     * 
     * @param input The array of {@link Pose2d} to prepare, in meters
     * @return The prepared array of {@link Pose2d}, in meters
     */
    public static Pose2d[] preparePolygon(Pose2d[] input) {
        Pose2d[] out = input;

        int i = 0;
        for (Pose2d pose : input) {
            out[i] = prepareMetersPose(pose);
            i += 1;
        }

        return out;
    }

    /** A position for the hub, Measurements in INCHES! */
    public class Hub {
        // Default red
        public static final Pose3d HUB_TOP = new Pose3d(new Translation3d(469.11, 158.84, 72.00),
                new Rotation3d());
        public static final Pose3d HUB_CENTER = new Pose3d(new Translation3d(469.11, 158.84, 44.25),
                new Rotation3d());
    }

    /** A position for the tower, Measurements in inches */
    public class Tower {
        // Default red
        public static final Pose3d TOWER_CENTER = new Pose3d(new Translation3d(158.84 - 11.38, 47 + 155.05, 0),
                new Rotation3d());
    }

    public class Outpost {
        // Default red
        public static final Pose3d OUTPOST_CENTER = new Pose3d();

    }

    /** Length and width of the field, measurements in inches */
    public class Field {
        public static final double FIELD_LENGTH = 651.22;
        public static final double FIELD_WIDTH = 317.69;

        public static final Pose2d CENTER_INCHES = new Pose2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, new Rotation2d());
        public static final Pose2d CENTER = Utils.inchesToMeters(CENTER_INCHES);
    }

    /**
     * A rectangle defined by two opposite corners. Used for defining zones on the
     * {@link FieldPositions field}.
     */
    private static class Rect {
        Pose2d[] rect;

        Rect(Pose2d point1, Pose2d point2) {
            rect = new Pose2d[4];
            Rotation2d rotation = new Rotation2d(0);

            double minX = Math.min(point1.getX(), point2.getX());
            double maxX = Math.max(point1.getX(), point2.getX());
            double minY = Math.min(point1.getY(), point2.getY());
            double maxY = Math.max(point1.getY(), point2.getY());

            rect[0] = new Pose2d(minX, minY, rotation);
            rect[1] = new Pose2d(maxX, minY, rotation);
            rect[2] = new Pose2d(maxX, maxY, rotation);
            rect[3] = new Pose2d(minX, maxY, rotation);
        }

        /**
         * Gets the four corners of the rectangle, starting with the bottom left corner
         * and going clockwise.
         * 
         * @return An array of the four corners.
         */
        public Pose2d[] getPoints() {
            return rect;
        }
    }

    /** Zones on the field, measurements in meters */
    public class Zones {
        public static Pose2d[] get(ZoneType type) {
            // TODO: This limits to only having one zone per type, fine for now
            switch (type) {
                default:
                case PASSING:
                    return preparePolygon(PASSING_ZONE);

                case SCORING:
                    return preparePolygon(SCORING_ZONE);
            }
        }

        private static final double PASSING_ZONE_HEIGHT = 4.5;
        private static final double SCORING_ZONE_HEIGHT = 4;

        private static final double ZONE_BUFFER = 2 + SCORING_ZONE_HEIGHT;

        private static final double FIELD_WIDTH_METERS = Units.inchesToMeters(FieldPositions.Field.FIELD_WIDTH);

        // Center of the field
        public static final Pose2d[] PASSING_ZONE = new Rect(
                new Pose2d(ZONE_BUFFER, 0, new Rotation2d()),
                new Pose2d(PASSING_ZONE_HEIGHT + ZONE_BUFFER, FIELD_WIDTH_METERS, new Rotation2d()))
                .getPoints();

        // Edge of the field
        public static final Pose2d[] SCORING_ZONE = new Rect(
                new Pose2d(0.5, 0, new Rotation2d()),
                new Pose2d(SCORING_ZONE_HEIGHT, FIELD_WIDTH_METERS, new Rotation2d()))
                .getPoints();

        public static final Pose2d[] PASSING_POINTS_ARRAY = {
                new Pose2d((SCORING_ZONE_HEIGHT / 2.3), FIELD_WIDTH_METERS - 1.5, new Rotation2d()),
                new Pose2d((SCORING_ZONE_HEIGHT / 2.3), 1.5, new Rotation2d()) };

        public static final List<Pose2d> PASSING_POINTS = new ArrayList<>();

        static {
            for (Pose2d point : FieldPositions.Zones.PASSING_POINTS_ARRAY) {
                PASSING_POINTS.add(FieldPositions.prepareMetersPose(point));
            }
        }
    }
}
