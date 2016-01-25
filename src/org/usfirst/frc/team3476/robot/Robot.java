
package org.usfirst.frc.team3476.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team3476.Communications.OrangeCamera;
import org.usfirst.frc.team3476.Communications.OrangeCamera.ExposureControl;
import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.ScriptableAuto.Clock;
import org.usfirst.frc.team3476.ScriptableAuto.Main;
import org.usfirst.frc.team3476.Subsystems.*;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Toggle;
import org.usfirst.frc.team3476.Utility.Control.DifferentialGyro;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoder;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    Joystick joy;
	
	File logFile = new File("/usr/local/frc/logs/start.txt");
	FileWriter logger;
	
	Joystick xbox = new Joystick(0);
	Joystick joystick = new Joystick(1);
	
	double xAxis = -xbox.getRawAxis(4);
	double yAxis = -xbox.getRawAxis(1);
	double rightTrigger = xbox.getRawAxis(3);
	
	Talon flyTalon1 = new Talon(0);
	Talon flyTalon2 = new Talon(1);
	Talon turretTalon = new Talon(2);
	
	//Flywheel constants
	final double FLY1 = -1, FLY2 = 1;
	
	RobotDrive drive = new RobotDrive(7, 8, 4, 5);
	Solenoid shifterSoleniod = new Solenoid(3);
	
	enum Mode {DEFAULT, INTAKE, SHOOTUP, SHOOTDOWN}
    Mode mode = Mode.DEFAULT;
    
    //Shifting state
    RunningAverage avgRate = new RunningAverage(8);
    final double IPS = 48.0;
    final double HYSTERESIS = 0.2;
    enum ShiftingState {LOW, HIGH}
    ShiftingState shiftingState = ShiftingState.LOW;
    
    //Joystick buttons
    final int DEFAULT = 12, TRIGGER = 1, REVERSE = 3;
    boolean defaultButton = joystick.getRawButton(DEFAULT);
    boolean trigger = joystick.getRawButton(TRIGGER);
    boolean reverseButton = joystick.getRawButton(REVERSE);
    
    //Xbox buttons
    final int INTAKEUP = 5, INTAKEDOWN = 6;
    boolean intakeUpButton = xbox.getRawButton(INTAKEUP);
    boolean intakeDownButton = xbox.getRawButton(INTAKEDOWN);
    
    //Encoders
    MedianEncoder leftDrive = new MedianEncoder(3, 4, false, EncodingType.k4X, 5);
    MedianEncoder rightDrive = new MedianEncoder(1, 2, true, EncodingType.k4X, 5);
    MedianEncoder turretenc = new MedianEncoder(5, 6, true, EncodingType.k4X, 5);
    
    Main automain;
    Subsystem[] systems;
    AutoTask auto = new AutoTask();
    Thread autoThread = new Thread(auto, "autoThread");
    
    DifferentialGyro gyro = new DifferentialGyro(0, 5);//TODO: get this channel
    
    Counter tach = new Counter(0);//TODO: get this channel
    
    int iters = 0;
    int threads = 0;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        //SmartDashboard.putData("Auto choices", chooser);
        joy = new Joystick(0);
        
        systems = new Subsystem[10];
		systems[0] = new Drive(leftDrive, rightDrive, gyro, drive, shifterSoleniod);
		systems[1] = new Shooter(flyTalon1, flyTalon2, turretTalon, tach, turretenc);
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    @Override
	public void autonomousInit()
	{
		if(automain == null) automain = new Main("2016", systems);
		//This is first to appease watchdog
		//Start all threads for auto
		for(Subsystem sys : systems)
		{
			if(sys != null) sys.startThreads();
		}
		automain.robotDriveClear();
		automain.stop(autoThread);//Reset that sucker
		autoThread = new Thread(auto, "autoThread");
		autoThread.start();
		
		cameraInit();
	}
	
	@Override
	public void disabledInit()
	{
		if(automain != null)
		{
			automain.stop(autoThread);//Stop auto thread, we're not in auto
			automain.robotDriveClear();
		}
		automain = null;
		//Stop auto threads, we're not in auto
		for(Subsystem sys : systems)
		{
			if(sys != null) sys.stopThreads();
		}
		threads = 0;
	}
	
	public void disabledPeriodic()
	{
		iters++;
    	if(threads != Thread.getAllStackTraces().keySet().size()) System.out.println("Threads changed: " + Thread.getAllStackTraces().keySet().size());
    	threads = Thread.getAllStackTraces().keySet().size();
	}

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
    	if(Timer.getMatchTime() > 15.0) automain.stop(autoThread);
    }
    
    public void teleopInit()
    {
    	/*if(automain != null)
		{
			automain.stop(autoThread);//Stop auto thread, we're not in auto
			automain.robotDriveClear();
		}
		automain = null;
    	//Stop auto threads, we're not in auto
    	for(Subsystem sys : systems)
		{
			if(sys != null) sys.stopThreads();
		}*/
    	if(automain == null) automain = new Main("2016", systems);
    	
    	for(Subsystem sys : systems)
		{
			if(sys != null && !sys.threadsActive()) sys.startThreads();
		}
    	
    	cameraInit();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
    	automain.robotDriveClear();
    	if(joy.getRawButton(1))
    		((Shooter)systems[1]).aim();
    	else
    		((Shooter)systems[1]).stopAim();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {}
    
    public class AutoTask implements Runnable
    {
		@Override
		public void run()
		{
			automain.start();
		}
    }
    
    private void cameraInit()
    {
    	System.out.println("Camera Initialized");
    	OrangeCamera cam = new OrangeCamera("axis-camera.local");
    	cam.writeExposureControl(ExposureControl.kHold);
    	cam.writeExposure(0);
    }
}