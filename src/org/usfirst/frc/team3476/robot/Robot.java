package org.usfirst.frc.team3476.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.hal.PDPJNI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.io.File;
import java.io.FileWriter;
import java.util.function.DoubleToLongFunction;

import org.usfirst.frc.team3476.Communications.Dashcomm;
import org.usfirst.frc.team3476.Main.Starter;
import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.ScriptableAuto.Clock;
import org.usfirst.frc.team3476.ScriptableAuto.Main;
import org.usfirst.frc.team3476.Subsystems.*;
import org.usfirst.frc.team3476.Subsystems.Drive.ShiftingState;
import org.usfirst.frc.team3476.Subsystems.Shooter.HoodState;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.ShooterLogger;
import org.usfirst.frc.team3476.Utility.Toggle;
import org.usfirst.frc.team3476.Utility.Control.AtoD;
import org.usfirst.frc.team3476.Utility.Control.DifferentialAnalogGyro;
import org.usfirst.frc.team3476.Utility.Control.DifferentialSPIGyro;
import org.usfirst.frc.team3476.Utility.Control.DonutCANTalon;
import org.usfirst.frc.team3476.Utility.Control.DonutDrive;
import org.usfirst.frc.team3476.Utility.Control.DonutTalon;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoder;
import org.usfirst.frc.team3476.Utility.Control.OrangeDigital;
import org.usfirst.frc.team3476.Utility.Control.OrangeDigitalInput;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDCounterPeriodWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper.Data;
import org.usfirst.frc.team3476.Utility.Control.SimpleRamper;

import com.ni.vision.VisionException;

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
	
	PowerDistributionPanel pdp = new PowerDistributionPanel(1);
	
	//TODO: get incorrect channels
	Talon flyTalon1 = new Talon(8),//correct
			flyTalon2 = new Talon(9),//correct
			loaderTalon = new Talon(7),//correct
			intake1 = new Talon(6),
			intake2 = new Talon(5);
											
	DonutCANTalon turretMotor = new DonutCANTalon(4, 0);
	
	PIDDashdataWrapper vision = new PIDDashdataWrapper(Data.VISIONX);
	
	CANTalon ddmotor = new CANTalon(3);
	
	//Flywheel constants
	final double FLY1 = -1, FLY2 = 1;
	
	//Drive
	DonutDrive drive = new DonutDrive(2, 3, 0, 1);

	Solenoid shifterSoleniod = new Solenoid(0);
	Solenoid hood = new Solenoid(1), wheelie = new Solenoid(2);
	
	enum Mode {DEFAULT, INTAKE, SHOOTCLOSE, SHOOTFAR, SHOOTFLEX}
    Mode mode = Mode.DEFAULT;
    
    //Joystick buttons
    final int DEFAULT = 12, TRIGGER = 1, REVERSE = 3;
    boolean defaultButton = joy.getRawButton(DEFAULT);
    boolean trigger = joy.getRawButton(TRIGGER);
    boolean reverseButton = joy.getRawButton(REVERSE);
    
    //Encoders
    MedianEncoder leftDrive = new MedianEncoder(10, 11, false, EncodingType.k4X, 5);
    MedianEncoder rightDrive = new MedianEncoder(12, 13, true, EncodingType.k4X, 5);
    
    DigitalInput halleffect = new DigitalInput(0);//TODO: ensure switch channel at a later time
    
    Main main;
    Subsystem[] systems;
    
    //Gyro analoggyro = new DifferentialAnalogGyro(0, 5);//TODO: replace with spi
    Gyro spiGyro = new DifferentialSPIGyro(SPI.Port.kOnboardCS0, 5);
    
    AnalogInput pressure = new AnalogInput(3), ballsensor = new AnalogInput(2);
    
    //AtoD loaderSwitch = new AtoD(ballsensor, 0.65, false);
    OrangeDigital loaderSwitch = new OrangeDigitalInput(5);
    
    DigitalInput banner = new DigitalInput(1);
    PIDCounterPeriodWrapper tach = new PIDCounterPeriodWrapper(banner, 60);
    
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
    
    final boolean autodropdown = true;
    
    boolean automatic = false;
    
    boolean shooterdatacollect = false;
    ShooterLogger shooterLogger;
    
    boolean[] 	joybuttons = new boolean[13], xboxbuttons = new boolean[11],
    			lastjoybuttons = new boolean[joybuttons.length], lastxboxbuttons = new boolean[xboxbuttons.length];
    
    double joysetpoint;
    int joycount;
    int lastPOV;
	private double testset = 0;
	private boolean posmove;
	private double watchset;
	private boolean rehome = false;
    
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
    	
    	leftDrive.setDistancePerPulse(0.05235987755982988730771072305464);
    	rightDrive.setDistancePerPulse(0.05235987755982988730771072305464);
    	
    	intake1.setInverted(false);
    	intake2.setInverted(false);
    	
    	tach.setSamplesToAverage(4);
    	spiGyro.calibrate();
    	turretMotor.setSafetyEnabled(false);
    	loaderTalon.setInverted(true);
    	flyTalon2.setInverted(true);
    	new PIDCANTalonEncoderWrapper(ddmotor, 1).reset();
    	
    	
    	if(!automatic)
    	{
    		ddmotor.setInverted(true);
    	}
    	
    	if(shooterdatacollect)
    	{
    		shooterLogger = new ShooterLogger();
    	}
    	
        //Systems
        systems = new Subsystem[10];
		systems[0] = new Drive(leftDrive, rightDrive, spiGyro, drive, shifterSoleniod);
		systems[1] = new Turret(turretMotor, halleffect, vision);
		systems[2] = new Shooter(	flyTalon1, flyTalon2, loaderTalon, (Turret)systems[1], tach,
									loaderSwitch, vision, hood);
		systems[3] = new Intake(intake1, intake2, ddmotor, pdp);
		systems[8] = new Watcher(systems, 10);
		systems[9] = new Clock(systems);
		
		((Watcher)systems[8]).watch(false);
		
		//misc init
		((Clock)systems[9]).megaEnd();
		((Drive)systems[0]).autoShifting(false);
    	
    	if(!autodropdown)
    	{
    		((Intake)systems[3]).stopHoming();
    		((Intake)systems[3]).stopDD();
    	}
		
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
    			((Turret)systems[1]).resetHomer();
    			homed = true;
    		}
    		setCameramode(CameraMode.VISION);
    		main.startSubsystems();
    		main.startThread();
    		first = false;
    		
    		Dashcomm.putBoolean("match/enabled", true);
    		
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
				
				Dashcomm.putBoolean("match/enabled", false);
				
			}
			if(camfirst && starter.cameraDone())
			{
				setCameramode(CameraMode.VISION);
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
    	joysetpoint = 0;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
    	int POV = joy.getPOV(0);
    	if(starter.importantDone())//WE ARE REEADY TO RUMMMMMMBLEEEEEEE
    	{
    		//intake pos constants
    		final double up = 295, horiz = 4630, down = 5705;
    		
    		final double FLYWHEELMAX = 9100;
    		
    		final double CLOSESPEED = 6750, FARSPEED = 6750;
    		
    		//prints
    		boolean axisprint = false,  tachprint = true, currentprint = false,
    				pressureprint = false, driveencoderprint = false, spigyroprint = false,
    				intakeenc = false, shooterout = false, ballsensorprint = false,
    				distanceprint = false, shootervdist = false, povprint = false,
    				turretencoderprint = false, loaderstateprint = false, xboxaxisprint = false;
    		
    		//others
    		boolean turretgo = true, buttons = false, distmode = false,
    				searchgo = false, manualLoad = false, ddtuning = false;
    		
    		for(int i = 1; i < joybuttons.length; i++)
    		{
    			joybuttons[i] = joy.getRawButton(i);
    		}
    		
    		for(int i = 1; i < xboxbuttons.length; i++)
    		{
    			xboxbuttons[i] = xbox.getRawButton(i);
    		}
    		
    		double 	xAxis = -xbox.getRawAxis(4),
	    			yAxis = -xbox.getRawAxis(1);
    		
    		boolean joy1 = joy.getRawButton(1), joy2 = joy.getRawButton(2);
    		
    		Drive drive = (Drive)systems[0];
    		Turret turret = (Turret)systems[1];
    		Shooter shooter = (Shooter)systems[2];
    		Intake intake = (Intake)systems[3];
    		
			PIDCANTalonEncoderWrapper encoder = turret.getTurretEncoder();
    		switch(first ? 1 : 0)
    		{
	    		case 1://first - For the first time in forever
	    			System.out.println("teleopPeriodic first");
					//DO THINGS
	    			shooter.end();
	    			turret.end();
					main.startSubsystems();//we need subsystems but not main
					//systems[1].stopThreads();//stop shooter
					first = false;
					//if first, will continue to regular execution
					if(automatic)
					{
						turret.stopAim();
						if(!homed)
			    		{
			    			turret.resetHomer();
			    			homed = true;
			    		}
						if(shooterdatacollect)
						{
							shooterLogger.clearLogger();
						}
						if(searchgo)
						{
							turret.aim(0);
						}
					}
					else
					{
						turret.killHomer();
					}
					
					Dashcomm.putBoolean("match/enabled", true);
					
					
	    		default://!first - Normal execution
	    			if(automatic)
	    			{
	    				drive.augmentedDrive(yAxis, xAxis);
	    				
	    				if(joybuttons[8])
	    				{
	    					setCameramode(CameraMode.VISION);
	    					mode = Mode.SHOOTCLOSE;
	    				}
	    				else if(joybuttons[10])
	    				{
	    					setCameramode(CameraMode.VISION);
	    					mode = Mode.SHOOTFAR;
	    				}
	    				else if(joybuttons[12])
	    				{
	    					setCameramode(CameraMode.INTAKE);
	    					mode = Mode.DEFAULT;
	    				}
	    				else if(joybuttons[6])
	    				{
	    					setCameramode(CameraMode.VISION);
	    					mode = Mode.SHOOTFLEX;
	    				}
	    				
	    				if(xbox.getRawButton(2))
	    				{
	    					setCameramode(CameraMode.INTAKE);
	    				}
	    				
	    				boolean eject = joybuttons[4];
	    				
	    				if(eject)
	    				{
	    					shooter.resetLoader();
	    					shooter.manualLoader(-1);
	    				}
	    				
	    				if(eject)
	    				{
	    					intake.manualIntake(-1);
	    				}
	    				else if(joybuttons[3])
	    				{
	    					shooter.intakeNotify(true);
	    					intake.manualIntake(1*shooter.getIntakeScaling());
	    				}
	    				else if(joybuttons[5])
	    				{
	    					shooter.intakeNotify(true);
	    					intake.manualIntake(-1*shooter.getIntakeScaling());
	    				}
	    				else
						{
	    					shooter.intakeNotify(false);
	    					intake.manualIntake(0);
						}
	    				
	    				if(eject)
	    				{
	    					shooter.manualShooter(-1);
	    				}
	    				else
	    				{
		    				switch(mode)
		    				{
			    				case SHOOTCLOSE:
			    					shooter.setHood(HoodState.HIGH);
			    					shooter.setFly(CLOSESPEED);
		    						break;
		    						
			    				case SHOOTFAR:
			    					shooter.setHood(HoodState.LOW);
			    					shooter.setFly(FARSPEED);
		    						break;
		    						
			    				case SHOOTFLEX:
			    					shooter.setHood(HoodState.LOW);
		    						if(distmode)
		    		    			{
		    		    				if(joy.getRawButton(2))
		    		    				{
		    		    					shooter.setFly();
		    		    				}
		    		    				else
		    		    				{
		    		    					shooter.stopFly();
		    		    				}
		    		    			}
		    		    			else//nondist
		    		    			{
		    		    				//System.out.println("Flyset: " + ((-joy.getRawAxis(3) + 1)/2)*FLYWHEELMAX);
		    		    				shooter.setFly(((-joy.getRawAxis(3) + 1)/2)*FLYWHEELMAX);
		    		    			}
		    						break;
		    						
			    				case DEFAULT:
			    					shooter.setHood(HoodState.LOW);
			    					shooter.setFly(0);
			    					break;
			    				default:
			    					System.out.println("Invalid Teleop State: " + mode);
			    					break;
		    				}
	    				}
	    				
	    				if(turretgo)
	    				{
	    					if(xboxbuttons[7] && !lastxboxbuttons[7])//button 7 just pressed
	    					{
	    						//start homing
	    						turret.resetHomer();
	    						rehome = true;
	    					}
	    					else if(!xboxbuttons[7] && lastxboxbuttons[7])
	    					{
	    						turret.killHomer();
	    						rehome = false;
	    					}
	    					
	    					if(!rehome)
	    					{
	    						if(buttons)
		    					{
					    			if(joy.getRawButton(1))
					    			{
					    				turret.aim(0);
					    			}
					    			if(joy.getRawButton(7))
					    			{
					    				turret.aim(0.25);
					    			}
					    			if(joy.getRawButton(8))
					    			{
					    				turret.aim(-0.25);
					    			}
					    			if(joy.getRawButton(9))//bed tings
					    			{
					    				turret.aim(0.65);
					    			}
					    			if(joy.getRawButton(10))//bed tings
					    			{
					    				turret.aim(-0.65);
					    			}
		    					}
		    					else if(searchgo)//search testing
		    					{
		    						if(joy2 && !lastJoy2)
	    							{
		    							turret.search(-0.1);
	    							}
		    						else if(!turret.isSearching())
		    						{
		    							turret.aim();
		    						}
		    					}
		    					else//joystick setpoint
		    					{
		    						//if(intake.intakeRunning()) POV = 0;//turn the turret if the intake is running
		    						switch(POV)
	    							{
		    							case -1://nothing
		    								watchset = Double.NaN;
		    								posmove = false;
		    								break;
		    								
		    							default:
		    								watchset = POV/360.0 + 0.000000001;
		    								posmove = true;
	    							}
		    						
		    						if(posmove)//moving to position
		    						{
		    							if(turret.isAutoDone() && turret.getTurretSet() == watchset)//posmove done
		    							{
		    								posmove = false;
		    							}
		    							else//not there yet, set it yo
		    							{
		    								turret.aim(watchset);
		    							}
		    						}
		    						else//joystick setpoint
		    						{
			    						if(joybuttons[2])
			    						{
			    							if(!lastjoybuttons[2])
			    							{
			    								turret.aim();
			    							}
			    						}
			    						else
			    						{
			    							if(lastJoy2)
			    							{
			    								joysetpoint = turret.getTurretEncoder().getDistance();
			    							}
			    							if(Math.abs(joy.getRawAxis(0)) > 0.1)
				    							joysetpoint += joy.getRawAxis(0)*0.01;
			    							else
			    							{
			    								joysetpoint = turret.getTurretEncoder().getDistance();
			    								//System.out.println("joysetpoint: " + joysetpoint);
			    							}
				    						
			    							turret.aim(joysetpoint);
			    						}
		    						}
		    					}
	    					}
	    				}
	    				else
	    				{
	    					turret.manualTurret(joy.getAxis(AxisType.kX));//xaxis
	    				}
		    			
		    			//shooter.manualShooter((-joy.getRawAxis(3)+1)/2);//throttle
		    			//shooter.manualShooter(0);
		    			//shooter.manualLoader(joy.getRawButton(1) ? 1 : 0);
		    			
	    				if(autodropdown)
	    				{
	    					if(ddtuning)
	    					{
	    						if(joybuttons[1])
	    						{
	    							if(joy.getRawButton(7))
				    				{
				    					intake.moveDropdown(up);
				    				}
				    				else if(joy.getRawButton(9))
				    				{
				    					intake.moveDropdown(horiz);
				    				}
				    				else if(joy.getRawButton(11))
				    				{
				    					intake.moveDropdown(down);
				    				}
	    						}
	    						else
								{
	    							intake.stopDD();
								}
	    					}
	    					else
	    					{
	    						if(xboxbuttons[8] && !lastxboxbuttons[8])
	    						{
	    							intake.home();
	    						}
	    						if(joy.getRawButton(7))
			    				{
			    					intake.moveDropdown(up);
			    				}
			    				else if(joy.getRawButton(9))
			    				{
			    					intake.moveDropdown(horiz);
			    				}
			    				else if(joy.getRawButton(11))
			    				{
			    					intake.moveDropdown(down);
			    				}
	    					}
	    				}
	    				else//manual dropdown
	    				{
	    					intake.stopDD();
	    					intake.manualDropdown(-joy.getRawAxis(1));//yaxis
	    				}
	    				
	    				if(xbox.getRawButton(1))//a - extend
	    				{
	    					wheelie.set(true);
	    				}
	    				else if(xbox.getRawButton(3))//x - retract
	    				{
	    					wheelie.set(false);
	    				}
		    			
		    			if(shooterdatacollect)
		    			{
		    				if(joy1 && !lastJoy)
		    				{
		    					shooterLogger.prepValue(turret.getVisionWrapper().getClosestDist(), 60/tach.getPeriod());
		    				}
		    				
		    				if(shooterdatacollect && shooterLogger.valueToLog() && joy.getPOV(0) == 0)
		    				{
		    					shooterLogger.logPrepped();
		    				}
		    				
		    				if(shooterdatacollect && POV == 180 && lastPOV != 180)
		    				{
		    					System.out.println(shooterLogger);
		    				}
		    			}
		    			else if(manualLoad)
		    			{
		    				if(joybuttons[1] && !lastjoybuttons[1])
		    				{
		    					System.out.println("DOING THE LOAD HACK\n\n\n\n\n\n");
		    					shooter.setLoaded();
		    					shooter.startFire();
		    				}
		    			}
		    			else
		    			{
			    			if(joy.getRawButton(1)) shooter.startFire();
		    			}
		    			
		    			drive.setShifterState(xbox.getRawAxis(3) > 0.5 ? ShiftingState.HIGH : ShiftingState.LOW);
		    			
		    			hoodToggle.input(joy.getRawButton(12));
//	    				hood.set(hoodToggle.get());
    				}
	    			else//manual
	    			{
	    				boolean calibrate = false;
	    				
	    				if(calibrate)
	    				{
	    					if(xbox.getRawButton(1))
		    				{
		    					drive.manualDrive(1, 0);
		    				}
		    				else if(xbox.getRawButton(2))
		    				{
		    					drive.manualDrive(-1, 0);
		    				}
		    				else
		    				{
		    					drive.manualDrive(0, 0);
		    				}
	    				}
	    				else
	    				{
	    					drive.manualDrive(yAxis, xAxis);
	    				}
	    				
	    				boolean donuttest = false;
	    				
	    				if(donuttest)
	    				{
	    					if(joy.getRawButton(1) && !lastJoy)
	    					{
	    						testset += 0.005;
	    					}
	    					else if(joy.getRawButton(2) && !lastJoy2)
	    					{
	    						testset -= 0.005;
	    					}
	    					turret.manualTurret(testset);//xaxis
	    					System.out.println("Motor value: " + turretMotor.get());
	    				}
	    				else
	    				{
	    					turret.manualTurret(joy.getRawAxis(0));//xaxis
	    				}
	    				
	    				shooter.manualShooter((-joy.getRawAxis(3)+1)/2);//throttle
	    				//shooter.manualLoader(joy.getRawButton(1) ? 1 : 0);
	    				
	    				if(joybuttons[1] && !lastjoybuttons[1])
	    				{
	    					System.out.println("DOING THE LOAD HACK\n\n\n\n\n\n");
	    					shooter.setLoaded();
	    					shooter.startFire();
	    				}
	    				
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
	    				
	    				hoodToggle.input(joy.getRawButton(12));
	    				hood.set(hoodToggle.get());
	    				
	    				drive.setShifterState(xbox.getRawAxis(3) > 0.5 ? ShiftingState.HIGH : ShiftingState.LOW);
	    				
	    				//System.out.println("Banner: " + banner.get());
	    			}
	    			
	    			Dashcomm.putNumber("data/flywheel/tachometer", 60/tach.getPeriod());
	    			Dashcomm.putNumber("data/flywheel/setpoint", shooter.getFlySet());
	    			
	    			if(loaderstateprint)
	    			{
	    				System.out.println("Loader State: " + shooter.getLoaderState());
	    			}
	    			
	    			if(turretencoderprint)
	    			{
	    				System.out.println("Turret encoder: " + turret.getTurretEncoder().getDistance());
	    			}
	    			
	    			if(xboxaxisprint)
	    			{
	    				System.out.println("Xbox x: " + xAxis + ", Xbox y:" + yAxis);
	    			}
	    			
	    			if(povprint)
	    			{
	    				System.out.println("POV: " + joy.getPOV(0));
	    			}
	    			
	    			if(shootervdist)
	    			{
	    				System.out.println("{Tach, Dist}: {" + (60/tach.getPeriod()) + ", " + turret.getVisionWrapper().getClosestDist() + "}");
	    			}
	    			
	    			if(distanceprint)
	    			{
	    				System.out.println("Distance to goal: " + turret.getVisionWrapper().getClosestDist());
	    			}
	    			
	    			if(ballsensorprint)
	    			{
	    				System.out.println("Voltage: " + ballsensor.getVoltage() + ", AtoD Value: " + loaderSwitch.get());
	    			}
	    			
	    			if(currentprint)
	    			{
	    				String print = "";
	    				
	    				boolean one = true;
	    				
	    				if(one)
	    				{
	    					int channel = 2;
	    					print += "Channel " + channel + ": " + pdp.getCurrent(channel) + "\n";
	    				}
	    				else
	    				{
	    					for(int i = 0; i < 13; i++)
		    				{
		    					print += "Channel " + i + ": " + pdp.getCurrent(i) + "\n";
		    				}
	    				}
	    				System.out.print(print);
	    			}
	    			
	    			if(intakeenc)
	    			{
	    				System.out.println("Setpoint: " + intake.getDDSet());
	    				System.out.println("Encoder: " + intake.getIntakeEncoder().getDistance());
	    			}
	    			
	    			if(spigyroprint)
	    			{
	    				System.out.println("SPIGyro: " + spiGyro.getAngle());
	    			}
	    			
    				if(axisprint)
    				{
	    				System.out.println("X: " + joy.getAxis(AxisType.kX));
	    				System.out.println("Y: " + joy.getAxis(AxisType.kY));
	    				System.out.println("Z: " + joy.getAxis(AxisType.kZ));
	    				System.out.println("Throttle: " + joy.getAxis(AxisType.kThrottle));
    				}
    				
    				if(shooterout)
    				{
    					System.out.println("Fly1: " + flyTalon1.get() + ", Fly2: " + flyTalon2.get());
    				}
    				
    				if(tachprint)
    				{
		        		if(iters % 10 == 0) System.out.println("Tach: " + (60/tach.getPeriod()));
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
    		}//end time switch
		}//end importantdone
    	lastPOV = POV;
    	lastJoy = joy.getRawButton(1);
    	lastJoy2 = joy.getRawButton(2);
    	
    	for(int i = 1; i < lastjoybuttons.length; i++)
		{
    		lastjoybuttons[i] = joybuttons[i];
		}
		
		for(int i = 1; i < lastxboxbuttons.length; i++)
		{
			lastxboxbuttons[i] = xboxbuttons[i];
		}
		
    	iters++;
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