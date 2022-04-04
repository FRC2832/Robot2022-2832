package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

public class Odometry{

    SwerveConstants constants = new SwerveConstants();
    SwerveModule swerveModule = new SwerveModule(constants);
    Drivetrain drivetrain = new Drivetrain();

    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);


    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    

    SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(kinematics, drivetrain.getHeading());
    
    public double getXPosition(){
        double xPos = swerveOdometry.getPoseMeters().getX();
        return xPos;
    }
    
    public double getYPosition(){
        double yPos = swerveOdometry.getPoseMeters().getY();
        return yPos;
    }
    
    public Rotation2d getRot(){
        Rotation2d rotation2d = drivetrain.getHeading();
        return rotation2d;
    }
    
    public void setPosition(double xPosition, double yPosition, double rotation, double time){
        double xPos = swerveOdometry.getPoseMeters().getX();
        double yPos = swerveOdometry.getPoseMeters().getY();
        double rot = swerveOdometry.getPoseMeters().getRotation().getRadians();
        xPos = xPosition-xPos;
        yPos = yPosition-yPos;
        rot = rot % 2*Math.PI;
        rot = rotation-rot;
        //while(swerveOdometry.getPoseMeters().getX()!=xPosition || swerveOdometry.getPoseMeters().getY()!=yPosition){
            drivetrain.drive(xPos/time, yPos/time, rot/time, false);
         //   }
       // drivetrain.drive(0, 0, 0, false);

    } 

    public void setPositionv2(SwerveModuleState setState){
        //SwerveModuleState swerveState = setState;
        
    }
}