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
    DriverStation.Alliance color;

    private String positionEntry;

    public LimeLight() {
        color = DriverStation.getAlliance().get(); // Red or blue (idk exact string match)
        
        if (color.equals(DriverStation.Alliance.Red)) { 
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

    // check whether or not limelight has an estimated pose
    public boolean hasPose() {
        SmartDashboard.putBoolean("limelight enabled", false);        
        double[] values = limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        for (int i = 0; i < 2; i++) {
            if (values[i] != 0) {
                SmartDashboard.putBoolean("limelight enabled", true);
                return true;
            }
        }
        SmartDashboard.putBoolean("limelight enabled", true);
        return false;
    }

    public Pose2d getPose2d() {
        double[] data = limelight.getEntry(positionEntry).getDoubleArray(new double[7]);

        return new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]));

    }



    // get the position of the robot as Pose2d from limelight
    public TimestampPose2d getTimestampedPose() {
        double[] data = limelight.getEntry(positionEntry).getDoubleArray(new double[7]);


        // Data Ref -> [x, y, z, roll, pitch, yaw, latency, tagcount, tagspan, avgdist, avgarea] 

        double x = data[0];
        double y = data[1];
        double yaw = data[5];


        TimestampPose2d pose = new TimestampPose2d(new Pose2d(x, y, Rotation2d.fromDegrees(yaw)),
                Timer.getFPGATimestamp() - (data[6] / 1000.0));
        // GET CURRENT POSE FROM LIMELIGHT
        return pose;
    }

    public int getNumAprilTags() {

        // Get the visibility of targets (tv - target visible)
        double[] tv = limelight.getEntry("tv").getDoubleArray(new double[0]); // Empty array to let Limelight return the correct size

        // Count the number of detected AprilTags based on the tv values (1 means visible target)
        int detectedAprilTags = 0;
        
        // Iterate through tv array to count how many targets are visible (tv value = 1)
        for (double isVisible : tv) {
            if (isVisible == 1.0) {
                detectedAprilTags++;
            }
        }

        return detectedAprilTags;

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
