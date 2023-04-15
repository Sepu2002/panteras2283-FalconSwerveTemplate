package frc.robot;

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




    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton turboBumper = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton precisionBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton AutoPilot = new JoystickButton(driver, XboxController.Button.kA.value);
   

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /*Shuffleboard structures */
    SendableChooser<Command> m_Autochooser = new SendableChooser<>();
    
    

    /*AUTO list */
    private final Command m_AutoEjemplo = new PathPlanerAuto(s_Swerve);


    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        

        m_Autochooser.setDefaultOption("Example Auto", m_AutoEjemplo);
        
        Shuffleboard.getTab("Dashboard").add(m_Autochooser).withPosition(0, 0);

        

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> turboBumper.getAsBoolean(),
                () -> precisionBumper.getAsBoolean(),
                () -> AutoPilot.getAsBoolean()
                
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
