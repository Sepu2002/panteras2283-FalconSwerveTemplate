package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.GP_Balance;
import frc.robot.autos.PathPlanerAuto;
import frc.robot.autos.TWGP_Balance;
import frc.robot.commands.ArmControl;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick codriver = new Joystick(1);
    private final Joystick driverSation = new Joystick(2);



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

    /*CoDriver Buttons */
    private final JoystickButton Low = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton Mid = new JoystickButton(codriver, XboxController.Button.kB.value);
    private final JoystickButton High = new JoystickButton(codriver, XboxController.Button.kY.value);
    private final JoystickButton Close = new JoystickButton(codriver, XboxController.Button.kX.value);
    private final JoystickButton RielAtras = new JoystickButton(codriver, XboxController.Button.kLeftStick.value);
    private final POVButton Retraer = new POVButton(codriver, 0);
    private final POVButton Extender = new POVButton(codriver, 180);

    private final JoystickButton Comer = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton Escupir = new JoystickButton(codriver, XboxController.Button.kRightBumper.value);
    

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Brazo b_Brazo = new Brazo();
    private final Gripper g_Gripper = new Gripper();

    /*Estructuras shuffleboard */
    SendableChooser<Command> m_Autochooser = new SendableChooser<>();
    
    

    /*Lista de Autónomos */
    private final Command m_AutoEjemplo = new PathPlanerAuto(s_Swerve);
    private final Command m_GP_Balance = new GP_Balance(s_Swerve, b_Brazo, g_Gripper);
    private final Command m_TWGP_Balance = new TWGP_Balance(s_Swerve, b_Brazo, g_Gripper);

    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        

        m_Autochooser.setDefaultOption("Auto Ejemplo", m_AutoEjemplo);
        m_Autochooser.addOption("GPBalance", m_GP_Balance);
        m_Autochooser.addOption("TWGP_Balance", m_TWGP_Balance);

        Shuffleboard.getTab("Dashboard").add(m_Autochooser).withPosition(0, 0);
    
        final Boolean DashLow =  Shuffleboard.getTab("Dashboard").add("Low Level", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(1, 0).getEntry().getBoolean(false);
        

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

        b_Brazo.setDefaultCommand(
            new ArmControl(
                b_Brazo,
                () -> Close.getAsBoolean(),
                () -> Low.getAsBoolean(),
                () -> Mid.getAsBoolean(),
                () -> High.getAsBoolean(),
                () -> RielAtras.getAsBoolean(),
                () -> Retraer.getAsBoolean(),
                () -> Extender.getAsBoolean()
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

        Comer.onTrue(new InstantCommand(() -> g_Gripper.come()));
        Comer.onFalse(new InstantCommand(() -> g_Gripper.quieto()));
        Escupir.onTrue(new InstantCommand(() -> g_Gripper.escupe()));
        Escupir.onFalse(new InstantCommand(() -> g_Gripper.quieto()));
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
