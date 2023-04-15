package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.DigitalOutput;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier turboBumper;
    private BooleanSupplier precisionBumper;
    private BooleanSupplier AutoPilot;
    

    DigitalOutput ledport1 = new DigitalOutput(0);
    DigitalOutput ledport2 = new DigitalOutput(1);


    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier turboBumper, BooleanSupplier precisionBumper, BooleanSupplier AutoPilot) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.turboBumper = turboBumper;
        this.precisionBumper = precisionBumper;
        this.AutoPilot = AutoPilot;

        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = -1*(Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 1));
        double strafeVal = -1*(Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 1));
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 1);

        /* Drive */
        if(turboBumper.getAsBoolean()==true){
            ledport1.set(true);
            ledport2.set(false);
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.turbomaxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        }
        else if(precisionBumper.getAsBoolean()==true){
            ledport1.set(false);
            ledport2.set(true);
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.precisionmaxSpeed), 
            rotationVal * Constants.Swerve.precisionmaxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        }
        else if(AutoPilot.getAsBoolean()==true){
        }
        else{
            ledport1.set(false);
            ledport2.set(false);
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );

        }
    }
}