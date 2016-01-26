
package org.usfirst.frc.team3476.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.Thread.State;

import org.usfirst.frc.team3476.Communications.OrangeCamera;
import org.usfirst.frc.team3476.Communications.OrangeCamera.ExposureControl;
import org.usfirst.frc.team3476.Main.Starter;
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
    
    Main main;
    Subsystem[] systems;
    
    DifferentialGyro gyro = new DifferentialGyro(0, 5);//TODO: get this channel
    
    Counter tach = new Counter(0);//TODO: get this channel
    
    Starter starter;
    Thread starterThread;
    
    int iters = 0;
    int threads = 0;
    
    boolean first = true;
    
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        joy = new Joystick(0);
        systems = new Subsystem[10];
		systems[0] = new Drive(leftDrive, rightDrive, gyro, drive, shifterSoleniod);
		systems[1] = new Shooter(flyTalon1, flyTalon2, turretTalon, tach, turretenc);
		starter = new Starter(main, "2016", systems);
		starterThread = new Thread(starter);
		starterThread.start();
    }
    
	/**
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    @Override
	public void autonomousInit(){}
    
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
		//Stop auto threads, we're not in auto
		if(starter.importantDone())
			main.stopSubsystems();
		first = true;
		threads = 0;
	}
	
	public void disabledPeriodic()
	{
		main.updateData();
		iters++;
    	if(threads != Thread.getAllStackTraces().keySet().size()) System.out.println("Threads changed: " + Thread.getAllStackTraces().keySet().size());
    	threads = Thread.getAllStackTraces().keySet().size();
	}
    
    public void teleopInit()
    {
    	
    	
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
				//DO THINGS
				main.startSubsystems();
				main.startThread();
				first = false;
			}
    		if(!first)
    		{
    			main.robotDriveClear();
    			if (joy.getRawButton(1))
    				((Shooter) systems[1]).aim();
    			else
    				((Shooter) systems[1]).stopAim();
    		}
		}
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {}
}