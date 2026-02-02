package org.ironriders.lib.field;

import org.dyn4j.geometry.Vector2;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.Zone.ZoneType;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
     * 
     * !Currently only HUB and TOWER are implemented, all other types will return a
     * blank Pose3d()!
     */
    public static Pose3d get(ElementType element) {
        switch (element) {
            case HUB:
                return preparePose(Hub.HUB_CENTER);

            case TOWER:
                return preparePose(Tower.TOWER_CENTER);

            default:
                break;
        }

        return new Pose3d();
    }

    public static Pose3d preparePose(Pose3d pose) {
        boolean blue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

        if (blue) {
            pose = Utils.inchesToMeters(pose);
            return pose;
        }

        Translation3d t = pose.getTranslation();
        Rotation3d r = pose.getRotation();

        Translation3d flippedTranslation = new Translation3d(
                Field.FIELD_LENGTH - t.getX(),
                Field.FIELD_WIDTH - t.getY(),
                t.getZ());

        Rotation3d flippedRotation = new Rotation3d(
                r.getX(),
                r.getY(),
                r.getZ() + Math.PI);

        return Utils.inchesToMeters(new Pose3d(flippedTranslation, flippedRotation));
    }

    public static Pose2d[] preparePolygon(Pose2d[] input) {
        Pose2d[] out = input;
        
        int i = 0;
        for (Pose2d pose : input) {
            out[i] = Utils.flattenPose3d(preparePose(Utils.expandPose2d(pose))); // Cursed
        }

        return out;
    }

    /* Measurements in INCHES! */
    public class Hub {
        public static final Pose3d HUB_TOP = new Pose3d(new Translation3d(182.11, 158.84, 72.00),
                new Rotation3d());
        public static final Pose3d HUB_CENTER = new Pose3d(new Translation3d(182.11, 158.84, 44.25),
                new Rotation3d());
    }

    public class Tower {
        public static final Pose3d TOWER_CENTER = new Pose3d(new Translation3d(158.84 - 11.38, 47 + 155.05, 0),
                new Rotation3d());
    }

    public class Field {
        public static final double FIELD_LENGTH = 651.22;
        public static final double FIELD_WIDTH = 317.69;
    }

    public class Zones {
        public static Pose2d[] get(ZoneType type) { // TODO: This limits to only having one zone per type, fine for now
            switch (type) {
                default:
                case PASSING:
                    return preparePolygon(PASSING_ZONE);
                
                case SCORING:
                    return preparePolygon(SCORING_ZONE);
            }
        }

        private static final double PASSING_ZONE_HEIGHT = 4;
        private static final double SCORING_ZONE_HEIGHT = 4.5;

        private static final double ZONE_BUFFER = 2 + PASSING_ZONE_HEIGHT;

        private static final double FIELD_WIDTH_METERS = Units.inchesToMeters(FieldPositions.Field.FIELD_WIDTH);

        public static final Pose2d[] PASSING_ZONE = new FieldPositions().new Square(new Pose2d(), new Pose2d(
                PASSING_ZONE_HEIGHT, FIELD_WIDTH_METERS, new Rotation2d()))
                .getPoints();

        public static final Pose2d[] SCORING_ZONE = new FieldPositions().new Square(
                new Pose2d(ZONE_BUFFER, 0, new Rotation2d()),
                new Pose2d(SCORING_ZONE_HEIGHT + ZONE_BUFFER, FIELD_WIDTH_METERS, new Rotation2d())).getPoints();
    }

    private class Square {
        Pose2d[] square;

        // point1 and point2 are opposite corners, and the square is not rotated
        Square(Pose2d point1, Pose2d point2) {
            square = new Pose2d[4];

            square[0] = point1;
            square[1] = new Pose2d(point1.getX(), point2.getY(), point1.getRotation());
            square[2] = point2;
            square[3] = new Pose2d(point2.getX(), point1.getY(), point2.getRotation());
        }

        public Pose2d[] getPoints() {
            return square;
        }
    }

}
