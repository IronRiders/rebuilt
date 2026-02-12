package org.ironriders.lib.field;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveDrive;

public class Zone {
    public enum ZoneType {
        PASSING,
        SCORING;
    }

    public ZoneType type;

    public Pose2d[] polygon;

    static int id = 0;

    public Zone(Pose2d[] polygon, ZoneType type) {
        this.polygon = polygon;

        this.type = type;

        printPolygon();
    }

    public Zone(ZoneType type) {
        this.polygon = FieldPositions.Zones.get(type);

        this.type = type;

        printPolygon();
    }

    public void printPolygon() {
        SwerveDrive swerve = DriveSubsystem.getSwerveDrive();

        if (swerve == null) {
            return;
        }

        Field2d field = swerve.field;

        for (Pose2d point : polygon) {
            field.getObject((type.name() + id).replaceAll(" ", "")).setPose(point); // Hopefully unique
            id += 1;
        }
    }

    /*
     * Get the zone's polygon
     */
    public Pose2d[] getPolygon() {
        return polygon;
    }

    /*
     * Get the type of zone
     */
    public ZoneType getType() {
        return type;
    }

    public List<Pose2d> asList() {
        List<Pose2d> points = new ArrayList<Pose2d>();

        for (Pose2d point : polygon) {
            points.add(FieldPositions.prepareMetersPose(point));
        }

        return points;
    }

    /*
     * Get the center of the polygon.
     */
    public Pose2d centerPoint() {
        return new Pose2d(new Translation2d(Stream.of(polygon).collect(Collectors.averagingDouble(Pose2d::getX)),
                Stream.of(polygon).collect(Collectors.averagingDouble(Pose2d::getY))), new Rotation2d());
    }

    /*
     * Test if the robot is inside the zone.
     */
    public boolean inside() {
        return isPointInPolygon(getPose(), polygon);
    }

    /*
     * Test if an arbitrary point is inside the zone.
     */
    public boolean inside(Pose2d point) {
        return isPointInPolygon(point, polygon);
    }

    /*
     * Test if an arbitrary point is inside an arbitrary zone.
     */
    public boolean inside(Pose2d point, Pose2d[] polygon) {
        return isPointInPolygon(point, polygon);
    }

    /*
     * Get the translation to the closest point to @param point in zone.
     * If no distance is found somehow, return Double.POSITIVE_INFINITY.
     */
    public Translation2d distanceTo(Pose2d point) {
        return distanceTo(point, polygon);
    }

    /*
     * Get the translation to the closest point in the zone to the robot.
     * If no distance is found somehow, return Double.POSITIVE_INFINITY.
     */
    public Translation2d distanceTo() {
        return distanceTo(getPose(), polygon);
    }

    /*
     * Get the translation to the closest point to @param point in the supplied
     * polygon.
     * If no distance is found somehow, return Double.POSITIVE_INFINITY.
     */
    public Translation2d distanceTo(Pose2d point, Pose2d[] polygon) {
        if (isPointInPolygon(point, polygon)) {
            // If we're inside the polygon the closest point is the point itself,
            // so the translation from the point to the closest point is zero.
            return new Translation2d(0.0, 0.0);
        }

        Translation2d minDist = new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

        for (int i = 0; i < polygon.length; i++) {
            Pose2d a = polygon[i];
            Pose2d b = polygon[(i + 1) % polygon.length];

            Translation2d dist = distancePointToSegment(
                    point.getX(), point.getY(),
                    a.getX(), a.getY(),
                    b.getX(), b.getY());

            if (minDist.getNorm() > dist.getNorm()) {
                minDist = dist;
            }
        }

        return minDist.times(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? 1 : -1);
    }

    public Pose2d closestPoint() {
        Translation2d distance = distanceTo();

        return new Pose2d(distance.getX() + getPose().getX(), distance.getY() + getPose().getY(), new Rotation2d());
    }

    public Pose3d closestPointAsPose3d() {
        return Utils.expandPose2d(closestPoint());
    }

    // --- Internal Methods ---
    private Translation2d distancePointToSegment(
            double px, double py,
            double x1, double y1,
            double x2, double y2) {

        double dx = x2 - x1;
        double dy = y2 - y1;

        // Segment is a point
        if (dx == 0 && dy == 0) {
            return new Translation2d(px - x1, py - y1);
        }

        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy); // THIS IS HELL

        t = Utils.clamp(0, 1, t);

        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;

        return new Translation2d(px - closestX, py - closestY);
    }

    private boolean isPointInPolygon(Pose2d point, Pose2d[] polygon) {
        double px = point.getX();
        double py = point.getY();

        boolean inside = false;

        for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
            double xi = polygon[i].getX(); // aaaaa
            double yi = polygon[i].getY();
            double xj = polygon[j].getX();
            double yj = polygon[j].getY();

            // Check if edge crosses horizontal ray at point.y
            boolean intersects = ((yi > py) != (yj > py));
            if (intersects) {
                double xIntersection = (xj - xi) * (py - yi) / (yj - yi) + xi;
                if (px < xIntersection) {
                    inside = !inside;
                }
            }
        }

        return inside;
    }

    private Pose2d getPose() {
        return DriveSubsystem.getSwerveDrive().getPose();
    }

}
