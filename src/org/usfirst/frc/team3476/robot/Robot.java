
package org.usfirst.frc.team3476.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import java.io.File;
import java.io.FileWriter;

import org.usfirst.frc.team3476.Communications.Dashcomm;
import org.usfirst.frc.team3476.Main.Starter;
import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.ScriptableAuto.Main;
import org.usfirst.frc.team3476.Subsystems.*;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Control.DifferentialAnalogGyro;
import org.usfirst.frc.team3476.Utility.Control.DifferentialSPIGyro;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoder;

import edu.wpi.first.wpilibj.CANTalon.SetValueMotionProfile;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
	
	Talon flyTalon1 = new Talon(0),
			flyTalon2 = new Talon(1),
			turretTalon = new Talon(2),
			loaderTalon = new Talon(3);//TODO: get this channel
	
	
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
    
    DigitalInput loaderSwitch = new DigitalInput(-1);//TODO: get this channel
    
    Main main;
    Subsystem[] systems;
    
    DifferentialAnalogGyro gyro = new DifferentialAnalogGyro(0, 5);//TODO: get this channel
    //DifferentialSPIGyro spiGyro = new DifferentialSPIGyro(SPI.Port.kOnboardCS0);
    
    Counter tach = new Counter(0);//TODO: get this channel
    
    Starter starter;
    Thread starterThread;
    
    int iters = 0;
    int threads = 0;
    boolean lastJoy = false;
    
    boolean first = true, camfirst = true;
    
    enum CameraMode {VISION, INTAKE};
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
    	System.out.println("robotInit");
        joy = new Joystick(0);
        
        //Systems
        systems = new Subsystem[10];
		systems[0] = new Drive(leftDrive, rightDrive, gyro, drive, shifterSoleniod);
		systems[1] = new Shooter(flyTalon1, flyTalon2, loaderTalon, turretTalon, tach, turretenc, loaderSwitch);
		
		//Main
		main = new Main();
		
		//Starter
		starter = new Starter(main, "2016", systems, 0, "10.94.76.11");
		starterThread = new Thread(starter);
		starter.resume();
		starterThread.start();
		
		//spiGyro.calibrate();
    }
    
	/**
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    @Override
	public void autonomousInit()
    {
    	first = true;
    }
    
    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic()
    {
    	if(starter.importantDone() && first)//WE ARE REEADY TO RUMMMMMMBLEEEEEEE and it's the first time
    	{
    		//DO THINGS
    		main.startSubsystems();
    		main.startThread();
    		first = false;
    	}
    	
    	if(Timer.getMatchTime() > 15.0) main.stop();
    }
	
	@Override
	public void disabledInit()
	{
		System.out.println("disabledInit");
		first = true;
		camfirst = true;
		threads = 0;
	}
	
	public void disabledPeriodic()
	{
		if(starter.importantDone())//WE ARE REEADY TO RUMMMMMMBLEEEEEEE
		{
			if(first)
			{
				System.out.println("disabledPeriodic first");
				main.stopSubsystems();//Stop auto threads, we're not in auto
				main.stop();
				first = false;
			}
			if(camfirst && starter.cameraDone())
			{
				setCameramode(CameraMode.INTAKE);
			}
			main.updateData();
			main.robotDriveClear();
		}
    	if(threads != Thread.getAllStackTraces().keySet().size())
    	{
    		System.out.println("Threads changed: " + Thread.getAllStackTraces().keySet().size());
    	}
    	threads = Thread.getAllStackTraces().keySet().size();
    	
	}
    
	@Override
    public void teleopInit()
    {
		System.out.println("teleopInit");
    	first = true;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
    	if(starter.importantDone())//WE ARE REEADY TO RUMMMMMMBLEEEEEEE
    	{
    		if(first)//For the first time in forever
    		{
    			System.out.println("teleopPeriodic first");
				//DO THINGS
				main.startSubsystems();//we need subsystems but not main
				first = false;
			}
    		if(!first)
    		{
    			Shooter shooter = (Shooter)systems[1];
    			main.robotDriveClear();
    			if (joy.getRawButton(1) && !lastJoy)
    			{
    				shooter.aim();
    				System.out.println("Aiming");
    			}
    			if(!joy.getRawButton(1) && lastJoy)
    			{
    				shooter.stopAim();
    				System.out.println("Stopping");
    			}
    			
    			double FLYWHEELMAX = 8500;
    			
    			if(joy.getRawButton(2))
    			{
    				shooter.setFly(((joy.getRawAxis(4) + 1)/2)*FLYWHEELMAX);
    			}
    			else
    			{
    				shooter.setFly(0);
    			}
    			
    			/*if(!joy.getRawButton(1))
    			{
    				System.out.println("Joy Value: " + joy.getAxis(AxisType.kX));
    				turretTalon.set(joy.getAxis(AxisType.kX));
    			}*/
    			
    			setCameramode(CameraMode.VISION);
    			
    			//System.out.println("SPI Gyro: " + spiGyro.get());
    		}
		}
    	lastJoy = joy.getRawButton(1);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {}
    
    public void setCameramode(CameraMode mode)
    {
    	Dashcomm.putBoolean("data/visioncam", mode == CameraMode.VISION);
    }
}