package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private String positionEntry = "botpose_wpiblue";

    public LimeLight() {
        refreshPoseEntry();
    }

    /** Call periodically or before reading pose — alliance may be unknown at construction. */
    public void refreshPoseEntry() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            positionEntry = "botpose_wpired";
        } else {
            positionEntry = "botpose_wpiblue";
        }
    }


    // ONLY CALLED ONCE AS A JOYSTICK COMMAND -> (Doesn't do anything)
    public void getLimeLightData() {
        double[] data = limelight.getEntry(positionEntry).getDoubleArray(new double[7]);
        System.out.println("X pos: " + data[0] + "," + "Y pos: " + data[1] + "," + "Z pos: " + data[2] + "Roll: " + data[3] + "," + "Pitch: " + data[4] + "," + data[5] + "," + "Yaw: " + data[6] + "\nall: " + data);
    }

    /**
     * True only when Limelight has a target and a non-degenerate botpose.
     * Zeros happen when: no tag in view, too far, motion blur, wrong pipeline, or wrong alliance entry.
     */
    public boolean hasPose() {
        refreshPoseEntry();
        // tv = 1 when any target is visible (single double, not an array)
        double tv = limelight.getEntry("tv").getDouble(0);
        if (tv < 0.25) {
            SmartDashboard.putBoolean("limelight enabled", false);
            return false;
        }
        double[] data = limelight.getEntry(positionEntry).getDoubleArray(new double[0]);
        if (data.length < 7) {
            SmartDashboard.putBoolean("limelight enabled", false);
            return false;
        }
        // If MegaTag2 11-field array, index 7 is tag count — require >= 1 when present
        if (data.length >= 8 && data[7] < 0.5) {
            SmartDashboard.putBoolean("limelight enabled", false);
            return false;
        }
        boolean ok = Math.hypot(data[0], data[1]) > 1e-3;
        SmartDashboard.putBoolean("limelight enabled", ok);
        return ok;
    }

    public Pose2d getPose2d() {
        refreshPoseEntry();
        double[] data = limelight.getEntry(positionEntry).getDoubleArray(new double[7]);
        if (data.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]));
    }



    // get the position of the robot as Pose2d from limelight
    public TimestampPose2d getTimestampedPose() {
        refreshPoseEntry();
        double[] data = limelight.getEntry(positionEntry).getDoubleArray(new double[7]);
        if (data.length < 7) {
            return new TimestampPose2d(new Pose2d(), Timer.getFPGATimestamp());
        }
        double x = data[0];
        double y = data[1];
        double yaw = data[5];
        double latencyMs = data[6];
        return new TimestampPose2d(
                new Pose2d(x, y, Rotation2d.fromDegrees(yaw)),
                Timer.getFPGATimestamp() - (latencyMs / 1000.0));
    }

    /** Uses tag count from botpose when 11-value MegaTag2 array is published; else 0/1 from tv. */
    public int getNumAprilTags() {
        refreshPoseEntry();
        double[] data = limelight.getEntry(positionEntry).getDoubleArray(new double[0]);
        if (data.length >= 8) {
            return (int) Math.round(data[7]);
        }
        return limelight.getEntry("tv").getDouble(0) > 0.5 ? 1 : 0;
    }

    public double getDistance() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 60.0; // need to adjust (JERRY HI)

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 15.8; 

        // distance from the target to the floor
        double goalHeightInches = 9.5; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    
    public class TimestampPose2d {
        private double timestamp;
        private Pose2d pose;

        public TimestampPose2d(Pose2d pose, double timestamp) {
            // REMINDER: Use Timer.getFGPAtimestamp or something like that or get from // FGPA is nanosecond hardware timestamp, more accurate than other timestamps like unix ect...
            // limelight networktables
            this.timestamp = timestamp;
            this.pose = pose;
        }

        public Pose2d getPose2d() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }
}
