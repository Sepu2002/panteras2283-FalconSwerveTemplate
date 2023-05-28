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




public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private Limelight l_Limelight;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier front;
    private BooleanSupplier back;
    private BooleanSupplier left;
    private BooleanSupplier right;
    public PIDController limelightPID ;


    


    public TeleopSwerve(
    Limelight l_Limelight, 
    Swerve s_Swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, 
    BooleanSupplier robotCentricSup, 
    BooleanSupplier front,
    BooleanSupplier back,
    BooleanSupplier left,
    BooleanSupplier right
    ) {

        this.s_Swerve = s_Swerve;
        this.l_Limelight=l_Limelight;
        addRequirements(s_Swerve, l_Limelight);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.front=front;
        this.back=back;
        this.left=left;
        this.right=right;
        

        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = (Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 1));
        double strafeVal = (Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 1));
        double rotationVal = (Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 1));
        double targetAngle=0;

        l_Limelight.SelectPipeline(1);
        l_Limelight.LEDsOFF();
        
        
        /*Lock robot to specific angles (Disables rotation joystick while specific orientation is selected) */
        if(front.getAsBoolean()==true || back.getAsBoolean()==true||left.getAsBoolean()==true||right.getAsBoolean()==true){
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
            s_Swerve.lockedrive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),  
            !robotCentricSup.getAsBoolean(), 
            true, targetAngle
        );
            
        }
        /*Default two joystick teleop drive */
        else{
           
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );

        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
     return false;
    }
}