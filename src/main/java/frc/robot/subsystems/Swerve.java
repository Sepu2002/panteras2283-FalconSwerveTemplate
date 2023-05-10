package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator SwervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public PIDController anglePID ;
    public PIDController XPID ;
    public PIDController YPID ;
    private int AutoPilotTarget;

    private ShuffleboardTab tab = Shuffleboard.getTab("Dashboard");
    private GenericEntry AutoPilotSection = tab.add("AP Selection",0).withPosition(1, 0).getEntry();
    
    
    public Swerve() {
        anglePID = new PIDController(0.005, 0.0001, 0);
        anglePID.enableContinuousInput(0, 360);
        XPID=new PIDController(0.5,0,0);
        YPID=new PIDController(0.5,0,0);

        AutoPilotTarget=0;

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        SwervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), getPose());



    }
    /*Default drive state for teleop */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    
    
    /*Drive state with rotation locked to cardinal directions */
    public void lockedrive(Translation2d translation, boolean fieldRelative, boolean isOpenLoop, double setPoint) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    anglePID.calculate(gyro.getYaw(), setPoint), 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    anglePID.calculate(gyro.getYaw(), setPoint))
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }  

    /*Updates current autopilot target location selection */
    public void AutoPilotDestinationSelector(int index){
        AutoPilotTarget = index;
        
    }
    
    
    
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public void setModuleStatesAuto(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }   

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return SwervePoseEstimator.getEstimatedPosition();
    }
    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetEstimation(Pose2d pose) {
        SwervePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public double getArea(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
      }

    public double[] getLimeInfo(){
        /*
         * 1-x
         * 0-y
         * 2-z
         * 3-Roll
         * 4-Pitch
         * 5-Yaw
         */
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0)!=-1.0){
            return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
        }

        else{
            double[] blank={0,0,0,0,0,0};
            return blank;
          
        }
      }

    public double getLimeDelay(){

        double tl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
        double cl= NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0);
        return (tl+cl);
        
    }
    
    public void VisionResetEstimation() {
        double[] Limeinfo = getLimeInfo();
        Pose2d pose=new Pose2d(new Translation2d(Limeinfo[0]+8.3, Limeinfo[1]+4.1), new Rotation2d(Limeinfo[5]));
        if(Limeinfo[0]!=0 && Limeinfo[1]!=0){
            resetOdometry(pose);
        }
    }

   

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               this.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             this::getPose, // Pose supplier
             Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
             new PIDController(0.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(0.5, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             this::setModuleStatesAuto, // Module states consumer
             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             this // Requires this drive subsystem
         )
     );

 }


    @Override
    public void periodic(){

        
        swerveOdometry.update(getYaw(), getModulePositions());

       if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0)!=-1.0){
        double[] Limeinfo = getLimeInfo();
        //SwervePoseEstimator.addVisionMeasurement(new Pose2d(new Translation2d(Limeinfo[1]+8.3, Limeinfo[0]+4.1), new Rotation2d(Limeinfo[5])), Timer.getFPGATimestamp()-getLimeDelay());
        //SwervePoseEstimator.update(getYaw(), getModulePositions());

        SmartDashboard.putNumber("LimePose X", Limeinfo[0]+8.3);  
        SmartDashboard.putNumber("LimePose Y", Limeinfo[1]+4.1);   
       }

       SmartDashboard.putNumber("EncoderPose X", getPose().getX());  
       SmartDashboard.putNumber("EncoderPose Y", getPose().getY());

       //Pose2d estPose= SwervePoseEstimator.getEstimatedPosition();
       //SmartDashboard.putNumber("Estx",estPose.getX());
       //SmartDashboard.putNumber("Esty",estPose.getY());
        
      
    

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

    
        AutoPilotSection.setDouble(AutoPilotTarget);

       
        


    }

    

}