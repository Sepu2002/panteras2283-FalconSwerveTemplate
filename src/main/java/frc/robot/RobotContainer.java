package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.PathPlanerAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;





/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final POVButton zeroGyro = new POVButton(driver, 0);
    private final POVButton calibrateEstimation = new POVButton(driver, 90);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton turboBumper = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton precisionBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final POVButton AutoPilot = new POVButton(driver, 180);
    private final JoystickButton lockFront = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton lockBack = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton lockLeft = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton lockRight = new JoystickButton(driver, XboxController.Button.kB.value);
    
    
    /*Operator buttons */
    private final JoystickButton one = new JoystickButton(operator,1);
    private final JoystickButton two = new JoystickButton(operator,2);
    private final JoystickButton three = new JoystickButton(operator,3);
    private final JoystickButton four = new JoystickButton(operator,4);
    private final JoystickButton five = new JoystickButton(operator,5);
    private final JoystickButton six = new JoystickButton(operator,6);
    private final JoystickButton seven = new JoystickButton(operator,7);
    private final JoystickButton eight = new JoystickButton(operator,8);
    private final JoystickButton nine = new JoystickButton(operator,9);
    //private final JoystickButton high = new JoystickButton(operator,10);

    private final JoystickButton autoTarget = new JoystickButton(operator, 14);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Limelight l_Limelight = new Limelight();

    /*Shuffleboard structures */
    SendableChooser<Command> m_Autochooser = new SendableChooser<>();
    
    /*AUTO list */
    private final Command m_ExampleAuto = new PathPlanerAuto(s_Swerve);



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        

        m_Autochooser.setDefaultOption("Example Auto", m_ExampleAuto);
        
        Shuffleboard.getTab("Dashboard").add(m_Autochooser).withPosition(0, 0);
        


        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                l_Limelight,
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> turboBumper.getAsBoolean(),
                () -> precisionBumper.getAsBoolean(),
                () -> AutoPilot.getAsBoolean(),
                () -> lockFront.getAsBoolean(),
                () -> lockBack.getAsBoolean(),
                () -> lockLeft.getAsBoolean(),
                () -> lockRight.getAsBoolean(),
                () -> autoTarget.getAsBoolean()
                
            )
        );



        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        calibrateEstimation.onTrue(new InstantCommand(()-> s_Swerve.VisionResetEstimation()));
        //one.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(1)));
        two.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(2)));
        three.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(3)));
        //four.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(4)));
        //five.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(5)));
        //six.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(6)));
        //seven.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(7)));
        //eight.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(8)));
        //nine.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(9)));
        //ten.onTrue(new InstantCommand(()-> s_Swerve.AutoPilotDestinationSelector(10)));

        
    }
    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_Autochooser.getSelected();
    }
}
