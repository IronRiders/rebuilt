package org.ironriders.lib.field;

import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/*
 * Class with positions of elements on the field.
 */
public class FieldPositions {
    /*
     * Gets the position in meters of the specified element type. Automatically does alliance mirroring.
     * 
     * !Currently only HUB and TOWER are implemented, all other types will return a blank Pose3d()!
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
            t.getZ()
        );

        Rotation3d flippedRotation = new Rotation3d(
            r.getX(),
            r.getY(),
            r.getZ() + Math.PI
        );

        return Utils.inchesToMeters(new Pose3d(flippedTranslation, flippedRotation));
    }
    
    /* Measurements in INCHES! */
    public class Hub {
        public static final Pose3d HUB_TOP = new Pose3d(new Translation3d(182.11, 158.84, 72.00),
                new Rotation3d());
        public static final Pose3d HUB_CENTER = new Pose3d(new Translation3d(182.11, 158.84, 44.25),
                new Rotation3d());
    }

    public class Tower {
        public static final Pose3d TOWER_CENTER = new Pose3d(new Translation3d(158.84-11.38, 47+155.05, 0), new Rotation3d());
    }

    public class Field {
        public static final double FIELD_LENGTH = 651.22;
        public static final double FIELD_WIDTH = 317.69;
    }
}
