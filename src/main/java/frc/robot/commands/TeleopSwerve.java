package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.DigitalOutput;




public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier turboBumper;
    private BooleanSupplier precisionBumper;
    private BooleanSupplier AutoPilot;
    private Boolean front;
    private Boolean back;
    private Boolean left;
    private Boolean right;

    

    DigitalOutput ledport1 = new DigitalOutput(0);
    DigitalOutput ledport2 = new DigitalOutput(1);

    


    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier turboBumper, BooleanSupplier precisionBumper, BooleanSupplier AutoPilot, BooleanSupplier front,BooleanSupplier back,BooleanSupplier left,BooleanSupplier right) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.turboBumper = turboBumper;
        this.precisionBumper = precisionBumper;
        this.AutoPilot = AutoPilot;
        this.front=front.getAsBoolean();
        this.back=back.getAsBoolean();
        this.left=left.getAsBoolean();
        this.right=right.getAsBoolean();

        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = (Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 1));
        double strafeVal = (Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 1));
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 1);
        double targetAngle=0;

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
        else if(front==true || back==true||left==true||right==true){
            if(front==true){
                targetAngle=0;
            }
            else if(back==true){
                targetAngle=180;
            }
            else if(left==true){
                targetAngle=90;
            }
            else if(right==true){
                targetAngle=270;
            }

            s_Swerve.lockedrive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),  
            !robotCentricSup.getAsBoolean(), 
            true, targetAngle
        );
            
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