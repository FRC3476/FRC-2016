
package org.usfirst.frc.team3476.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import java.io.File;
import java.io.FileWriter;

import org.usfirst.frc.team3476.Communications.Dashcomm;
import org.usfirst.frc.team3476.Main.Starter;
import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.ScriptableAuto.Clock;
import org.usfirst.frc.team3476.ScriptableAuto.Main;
import org.usfirst.frc.team3476.Subsystems.*;
import org.usfirst.frc.team3476.Subsystems.Drive.ShiftingState;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Toggle;
import org.usfirst.frc.team3476.Utility.Control.DifferentialAnalogGyro;
import org.usfirst.frc.team3476.Utility.Control.DifferentialSPIGyro;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoder;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.SetValueMotionProfile;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
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
	File logFile = new File("/usr/local/frc/logs/start.txt");
	FileWriter logger;
	
	Joystick xbox = new Joystick(0);
	Joystick joy = new Joystick(1);
	
	//TODO: get incorrect channels
	Talon flyTalon1 = new Talon(8),//correct
			flyTalon2 = new Talon(9),//correct
			loaderTalon = new Talon(7),//correct
			intake1 = new Talon(6),
			intake2 = new Talon(5);
											
	
	
	CANTalon turret = new CANTalon(4),
			ddmotor = new CANTalon(3);
	
	//Flywheel constants
	final double FLY1 = -1, FLY2 = 1;
	
	RobotDrive drive = new RobotDrive(2, 3, 0, 1);

	Solenoid shifterSoleniod = new Solenoid(0);
	Solenoid hood = new Solenoid(1);
	
	enum Mode {DEFAULT, INTAKE, SHOOTUP, SHOOTDOWN}
    Mode mode = Mode.DEFAULT;
    
    //Joystick buttons
    final int DEFAULT = 12, TRIGGER = 1, REVERSE = 3;
    boolean defaultButton = joy.getRawButton(DEFAULT);
    boolean trigger = joy.getRawButton(TRIGGER);
    boolean reverseButton = joy.getRawButton(REVERSE);
    
    //Encoders
    MedianEncoder leftDrive = new MedianEncoder(10, 11, false, EncodingType.k4X, 5);
    MedianEncoder rightDrive = new MedianEncoder(12, 13, true, EncodingType.k4X, 5);
    
    DigitalInput loaderSwitch = new DigitalInput(5), halleffect = new DigitalInput(0);//TODO: ensure switch channel at a later time
    
    Main main;
    Subsystem[] systems;
    
    DifferentialAnalogGyro gyro = new DifferentialAnalogGyro(0, 5);//TODO: replace with spi
    //DifferentialSPIGyro spiGyro = new DifferentialSPIGyro(SPI.Port.kOnboardCS0);
    
    AnalogInput pressure = new AnalogInput(3);
    
    DigitalInput banner = new DigitalInput(1);
    Counter tach = new Counter(banner);
    
    Starter starter;
    Thread starterThread;             
    
    Toggle hoodToggle = new Toggle();
    
    int iters = 0;
    int threads = 0;
    boolean lastJoy = false;
    boolean lastJoy2 = false;
    
    boolean first = true, camfirst = true;
    
    enum CameraMode {VISION, INTAKE};
    
    boolean homed = false;
    
    boolean automatic = false;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
    	System.out.println("robotInit");
        
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
    	tach.setSamplesToAverage(4);
    	
    	if(!automatic)
    	{
    		flyTalon2.setInverted(true);
    		ddmotor.setInverted(true);
        	loaderTalon.setInverted(true);
    	}
    	
    	//DONE BY SAIKIRAN TO TEST SHOOTER
//    	
//    	loaderTalon.setInverted(true);
    	
        //Systems
        systems = new Subsystem[10];
		systems[0] = new Drive(leftDrive, rightDrive, gyro, drive, shifterSoleniod);
		systems[1] = new Shooter(flyTalon1, flyTalon2, loaderTalon, turret, tach, loaderSwitch, halleffect);
		systems[2] = new Intake(intake1, intake2, ddmotor);
		systems[3] = new Watcher(systems);
		systems[9] = new Clock(systems);
		((Clock)systems[9]).megaEnd();
		
		//Main
		main = new Main();
		
		//Starter
		starter = new Starter(main, "2016", systems, 0, "10.94.76.11");
		starterThread = new Thread(starter);
		starter.resume();
		starterThread.start();
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
    		if(!homed)
    		{
    			((Shooter)systems[1]).resetHomer();
    			homed = true;
    		}
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
    		boolean axisprint = false, turretgo = false, tachprint = true, currentprint = false,
    				pressureprint = true, driveencoderprint = true;
    		
    		Drive drive = (Drive)systems[0];
    		Shooter shooter = (Shooter)systems[1];
    		Intake intake = (Intake)systems[2];
    		
			PIDCANTalonEncoderWrapper encoder = new PIDCANTalonEncoderWrapper(turret, 2.2379557291666666666666666666667e-5);
    		switch(first ? 1 : 0)
    		{
	    		case 1://first - For the first time in forever
	    			System.out.println("teleopPeriodic first");
					//DO THINGS
	    			((Watcher)systems[3]).watch(false);
					main.startSubsystems();//we need subsystems but not main
					//systems[1].stopThreads();//stop shooter
					first = false;
					//if first, will continue to regular execution
					if(automatic)
					{
						shooter.stopAim();
						if(!homed)
			    		{
			    			((Shooter)systems[1]).resetHomer();
			    			homed = true;
			    		}
					}
					else
					{
						((Shooter)systems[1]).killHomer();
					}
					
					
	    		default://!first - Normal execution
	    			if(automatic)
	    			{
	    				main.robotDriveClear();
	    				if(turretgo)
	    				{
			    			if(joy.getRawButton(1))
			    			{
			    				shooter.aim(0);
			    			}
			    			if(joy.getRawButton(2))
			    			{
			    				shooter.aim(-0.25);
			    			}
			    			
			    			if (joy.getRawButton(3))
			    			{
			    				System.out.println("encoder: " + encoder.get() + ", " + encoder.getDistance());
			    			}
	    				}
	    				else
	    				{
	    					shooter.manualTurret(joy.getAxis(AxisType.kZ));//zaxis
	    				}
		    			
		    			double FLYWHEELMAX = 8500;
		    			
		    			if(joy.getRawButton(2))
		    			{
		    				shooter.setFly(((-joy.getRawAxis(3) + 1)/2)*FLYWHEELMAX);
		    			}
		    			else
		    			{
		    				shooter.setFly(0);
		    			}
		    			
		    			
    				}
	    			else
	    			{
	    				double 	xAxis = -xbox.getRawAxis(4),
	    		    			yAxis = -xbox.getRawAxis(1);
	    				drive.manualDrive(yAxis, xAxis);
	    				
	    				shooter.manualTurret(joy.getAxis(AxisType.kX));//xaxis
	    				shooter.manualShooter((-joy.getRawAxis(3)+1)/2);//throttle
	    				shooter.manualLoader(joy.getRawButton(1) ? 1 : 0);
	    				
	    				if(joy.getRawButton(3))
	    				{
	    					intake.manualIntake(1);
	    				}
	    				else if(joy.getRawButton(4))
	    				{
	    					intake.manualIntake(-1);
	    				}
	    				else if(joy.getRawButton(5))
	    				{
	    					intake.manualIntake(0.5);
	    				}
	    				else if(joy.getRawButton(6))
	    				{
	    					intake.manualIntake(-0.5);
	    				}
	    				else
						{
	    					intake.manualIntake(0);
						}
	    				intake.manualDropdown(joy.getRawAxis(1));//yaxis
	    				
	    				hoodToggle.input(joy.getRawButton(11));
	    				hood.set(hoodToggle.get());
	    				
	    				drive.setShifterState(xbox.getRawAxis(3) > 0.5 ? ShiftingState.HIGH : ShiftingState.LOW);
	    				
	    				//System.out.println("Banner: " + banner.get());
	    			}
	    			
    				if(axisprint)
    				{
	    				System.out.println("X: " + joy.getAxis(AxisType.kX));
	    				System.out.println("Y: " + joy.getAxis(AxisType.kY));
	    				System.out.println("Z: " + joy.getAxis(AxisType.kZ));
	    				System.out.println("Throttle: " + joy.getAxis(AxisType.kThrottle));
    				}
    				
    				if(tachprint)
    				{
		        		System.out.println("Tach: " + (60/tach.getPeriod()));
    				}
    				
    				if(pressureprint)
    				{
		        		System.out.println("Pressure: " + (250*pressure.getVoltage()/5 - 25));
    				}
    				
    				if(driveencoderprint)
    				{
    					System.out.println("Left: " + leftDrive.getDistance());
    					System.out.println("Right: " + rightDrive.getDistance());
    				}
    				
	    			//drive.manualDrive(yAxis, xAxis);
	    			//shooter.manualTurret(joy.getAxis(AxisType.kX));
	    			//turret.set(joy.getAxis(AxisType.kX));
	    			/*ystem.out.println("Joy: " + joy.getAxis(AxisType.kX));
	    			System.out.println("Encoder: " + turret.getPosition());
	    			System.out.println("DIO: " + banner.get());
	    			System.out.println("Hall: " + halleffect.get());*/
	    			
	    			setCameramode(CameraMode.VISION);
	    			
	    			//System.out.println("SPI Gyro: " + spiGyro.get());
    		}
		}
    	lastJoy = joy.getRawButton(1);
    	lastJoy2 = joy.getRawButton(2);
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