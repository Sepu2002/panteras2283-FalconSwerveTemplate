package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
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
    private Limelight l_Limelight;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier turboBumper;
    private BooleanSupplier precisionBumper;
    private BooleanSupplier AutoPilot;
    private BooleanSupplier front;
    private BooleanSupplier back;
    private BooleanSupplier left;
    private BooleanSupplier right;
    private BooleanSupplier autoTarget;
    public PIDController limelightPID ;

    

    DigitalOutput ledport1 = new DigitalOutput(0);
    DigitalOutput ledport2 = new DigitalOutput(1);

    


    public TeleopSwerve(
    Limelight l_Limelight, 
    Swerve s_Swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, 
    BooleanSupplier robotCentricSup, 
    BooleanSupplier turboBumper, 
    BooleanSupplier precisionBumper, 
    BooleanSupplier AutoPilot, 
    BooleanSupplier front,
    BooleanSupplier back,
    BooleanSupplier left,
    BooleanSupplier right, 
    BooleanSupplier autoTarget) {

        this.s_Swerve = s_Swerve;
        this.l_Limelight=l_Limelight;
        addRequirements(s_Swerve, l_Limelight);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.turboBumper = turboBumper;
        this.precisionBumper = precisionBumper;
        this.AutoPilot = AutoPilot;
        this.front=front;
        this.back=back;
        this.left=left;
        this.right=right;
        this.autoTarget=autoTarget;

        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = (Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 1));
        double strafeVal = (Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 1));
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 1);
        double targetAngle=0;
        
        limelightPID = new PIDController(0.01, 0.15, 0);
        
        /* Turbo Drive (Increases mas speeds for rotation and translation)*/
        if(turboBumper.getAsBoolean()==true){

            l_Limelight.SelectPipeline(1);
            l_Limelight.LEDsOFF();
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.turbomaxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        }
        /*Precision Drive (Reduces max speeds for both rotation and translation)*/
        else if(precisionBumper.getAsBoolean()==true){

            l_Limelight.SelectPipeline(1);
            l_Limelight.LEDsOFF();
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.precisionmaxSpeed), 
            rotationVal * Constants.Swerve.precisionmaxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        }
        /*Autpilot to pose */
        else if(AutoPilot.getAsBoolean()==true){
            
            
        }

        /*Lock robot to specific angles (Disables rotation joystick while specific orientation is selected) */
        else if(front.getAsBoolean()==true || back.getAsBoolean()==true||left.getAsBoolean()==true||right.getAsBoolean()==true){
            if(front.getAsBoolean()==true){
                targetAngle=0;
            }
            else if(back.getAsBoolean()==true){
                targetAngle=180;
            }
            else if(left.getAsBoolean()==true){
                targetAngle=90;
            }
            else if(right.getAsBoolean()==true){
                targetAngle=270;
            }

            l_Limelight.SelectPipeline(1);
            l_Limelight.LEDsOFF();
            s_Swerve.lockedrive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),  
            !robotCentricSup.getAsBoolean(), 
            true, targetAngle
        );
            
        }

        /*Use limelight vision to always look at vision target (Disables ritation joystick unless target is in view)*/
        else if(autoTarget.getAsBoolean()==true){
            
            l_Limelight.SelectPipeline(0);
            l_Limelight.LEDsON();

            if(l_Limelight.getArea()>0){
                s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                limelightPID.calculate(l_Limelight.getTX(), 0) * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true);
            }
            else{
                s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true);
            }
        }

        /*Default two joystick teleop drive */
        else{
            l_Limelight.SelectPipeline(1);
            l_Limelight.LEDsOFF();
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