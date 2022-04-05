package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;

public class Odometry {
    Drivetrain drivetrain; // = new Drivetrain();
    SwerveConstants constants;
    SwerveModule swerveModule;

    Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d backRightLocation = new Translation2d(-0.381, -0.381);
    SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(kinematics, drivetrain.getHeading());

    public Odometry(XboxController controller) {
        constants = new SwerveConstants();
        swerveModule = new SwerveModule(constants);
        drivetrain = new Drivetrain(controller);
    }

    public double getXPosition() {
        return swerveOdometry.getPoseMeters().getX();
    }

    public double getYPosition() {
        return swerveOdometry.getPoseMeters().getY();
    }

    public Rotation2d getRot() {
        return drivetrain.getHeading();
    }

    public void setPosition(double xPosition, double yPosition, double rotation, double time) {
        double xPos = swerveOdometry.getPoseMeters().getX();
        double yPos = swerveOdometry.getPoseMeters().getY();
        double rot = swerveOdometry.getPoseMeters().getRotation().getRadians();
        xPos = xPosition - xPos;
        yPos = yPosition - yPos;
        rot = rot % 2 * Math.PI;
        rot = rotation - rot;
        // while(swerveOdometry.getPoseMeters().getX()!=xPosition ||
        // swerveOdometry.getPoseMeters().getY()!=yPosition){
        drivetrain.drive(xPos / time, yPos / time, rot / time, false);
        // }
        // drivetrain.drive(0, 0, 0, false);

    }

    public void setPositionv2(SwerveModuleState setState) {
        // SwerveModuleState swerveState = setState;

    }
}