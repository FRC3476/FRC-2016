package org.usfirst.frc.team3476.Subsystems;


import java.util.Arrays;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Subsystems.Turret.VisionMode;
import org.usfirst.frc.team3476.Utility.DIOHomer;
import org.usfirst.frc.team3476.Utility.DIOHomer.HomeState;
import org.usfirst.frc.team3476.Utility.ManualHandler;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.PolynomialFunction;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Control.AtoD;
import org.usfirst.frc.team3476.Utility.Control.BangBang;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDCounterPeriodWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.SmoothBangBang;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper.Data;
import org.usfirst.frc.team3476.Utility.Control.PIDMotorGroup;
import org.usfirst.frc.team3476.Utility.Control.PIDPolynomialWrapper;
import org.usfirst.frc.team3476.Utility.Control.TakeBackHalf;

import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDController.Tolerance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.hal.DIOJNI;

//TODO: javadoc comments if not specified by Subsystem
/*
 * Represents the robot's shooter.
 */
public class Shooter implements Subsystem
{
	private final String[] autoCommands = {"shooter", "flywheel", "fire", "reload", "directfire"};
	private final String[] constants = {"SHOOTEROUTPUTRANGEHIGH", "SHOOTEROUTPUTRANGELOW", "SHOOTERIGAIN",
			"FLY1DIR", "FLY2DIR", "FLYWHEELDEAD", "FLYWHEELMAXSPEED", "LOADERSPEED",
			"SHOOTERPGAIN", "SHOOTERDGAIN", "LOADERSPEEDLOW", "FIRINGTIME"};
	private final double RPSPERRPM = 60, TURRETOUTPUTRANGE = 0.25;
	
	enum LoaderState{RELOADINGSLOW, RELOADINGFAST, LOADED, FIRING, WAITING}
	LoaderState loaderState;
	enum FlyState{SETPOINT, DISTANCE}
	FlyState flyState;
	public enum HoodState{HIGH, LOW}
	
	private double SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW, SHOOTERIGAIN, FLYWHEELDEAD,
	FLYWHEELMAXSPEED, LOADERSPEED, SHOOTERPGAIN, SHOOTERDGAIN, LOADERSPEEDLOW,
	FIRINGTIME;
	
	private int softDir;
	private int[] FLYDIRS;
	private boolean flyDone, loadDone, toFire, toReload, firingLast,
					pass1, intakeRunning;
	private SpeedController fly1, fly2, loader;
	private TakeBackHalf control;
	private PIDCounterPeriodWrapper tach;
	private Timer shootingTimer;
	
	private PIDController flywheelcontrol;
	private PIDMotorGroup flygroup;
	
	private AtoD loaderSwitch;
	
	private SubsystemTask task;
	private Thread shooterThread;
	
	private PolynomialFunction distToFly;
	private PIDDashdataWrapper vision;
	
	private RunningAverage tachavg;
	
	private String lastCommand;
	
	private int iters;
	
	final long MANUALTIMEOUT = 50;//in ms
	final double FLYPIDEXECTIME = 1;
	private ManualHandler shooterManual, loaderManual;
	
	private Solenoid hood;
	
	Turret turret;
	private boolean lastballsw;
	private double flyset, lastflyset;
	private boolean lastintakeRunning;
	private LoaderState prevstate;
	private boolean fireDone;
	private double lastsetvision;
	
	public Shooter(	SpeedController fly1in, SpeedController fly2in, SpeedController loaderin,
					Turret turretin, PIDCounterPeriodWrapper tachin, AtoD loaderSwitchin,
					PIDDashdataWrapper vision, Solenoid hood)
	{
		turret = turretin;
		
		distToFly = new PolynomialFunction(22870.2, -4285.51, 243.7);//2nd order, 0, 1, 2
		this.vision = vision;
		
		//Shooter
		fly1 = fly1in;
		fly2 = fly2in;
		flyDone = true;
		loadDone = true;
		tach = tachin;
		tachavg = new RunningAverage(8);
		toFire = false;
		toReload = false;
		shootingTimer = new Timer();
		FLYDIRS = new int[]{0, 0};
		flygroup = new PIDMotorGroup(new SpeedController[]{fly1, fly2}, Arrays.copyOf(FLYDIRS, FLYDIRS.length));
		
		//Flywheel
		//control = new TakeBackHalf(new double[]{0, 0}, 0, 0);
		//control = new TakeBackHalf(new double[]{0, 0}, 1);
		flywheelcontrol = new PIDController(0, 0, 0, tach, flygroup, FLYPIDEXECTIME/1000.0);
		flyState = FlyState.SETPOINT;
		lastCommand = "";
		
		//Loader
		loaderSwitch = loaderSwitchin;
		loader = loaderin;
		loaderState = LoaderState.WAITING;
		prevstate = LoaderState.RELOADINGFAST;
		loadDone = true;
		fireDone = true;
		
		//Hood
		this.hood = hood;
		
		//Manuals
		shooterManual = new ManualHandler(MANUALTIMEOUT);
		loaderManual = new ManualHandler(MANUALTIMEOUT);
		
		iters = 0;
		task = new SubsystemTask(this, 5);//x ms minimum exec time
		shooterThread = new Thread(task, "shooterThread");
		shooterThread.start();
	}
	
	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}
	
	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		lastCommand = command;
		switch(command)
		{
			case "shooter":
				setFly(params[1]);
				break;
			case "flywheel":
				setFly(params[0]);
				break;
			case "fire":
				startFire();
				break;
			case "reload":
				reload();
				break;
			case "directfire":
				setLoaded();
				startFire();
				break;
		}
	}
	
	@Override
	public synchronized boolean isAutoDone()
	{
		/*switch(lastCommand)
		{
			case "shooter":
				return flyDone;
			case "flywheel":
				return flyDone;
			case "fire":
				return fireDone;
			case "reload":
				return loadDone;
			case "directfire":
				return fireDone;
			default:
				return true;
		}*/
		return fireDone && flyDone && loadDone;
	}
	
	@Override
	public String[] getConstantRequest()//Request all needed constants
	{
		return constants;
	}
	
	@Override
	public synchronized void returnConstantRequest(double[] constantsin)//Get all needed constants
	{
		double 	prevshooterp = SHOOTERPGAIN,
				prevshooteri = SHOOTERIGAIN,
				prevshooterd = SHOOTERDGAIN,
				prevshooterdead = FLYWHEELDEAD,
				prevoutputrangehigh = SHOOTEROUTPUTRANGEHIGH,
				prevoutputrangelow = SHOOTEROUTPUTRANGELOW;
		
		int[] prevflydirs = Arrays.copyOf(FLYDIRS, FLYDIRS.length);
		
		int i = 0;
		SHOOTEROUTPUTRANGEHIGH = constantsin[i];
		i++;//1
		SHOOTEROUTPUTRANGELOW = constantsin[i];
		i++;//2
		SHOOTERIGAIN = constantsin[i];
		i++;//3
		FLYDIRS[i - 3] = (int)constantsin[i];
		i++;//4
		FLYDIRS[i - 3] = (int)constantsin[i];
		i++;//5
		FLYWHEELDEAD = constantsin[i];
		i++;//6
		FLYWHEELMAXSPEED = constantsin[i];
		i++;//7
		LOADERSPEED = constantsin[i];
		i++;//8
		SHOOTERPGAIN = constantsin[i];
		i++;//9
		SHOOTERDGAIN = constantsin[i];
		i++;//10
		LOADERSPEEDLOW = constantsin[i];
		i++;//11
		FIRINGTIME = constantsin[i];
		
		if(!Arrays.equals(FLYDIRS, prevflydirs))
		{
			flygroup.setDirs(Arrays.copyOf(FLYDIRS, FLYDIRS.length));
		}
		
		boolean pdiff = prevshooterp != SHOOTERPGAIN, idiff = prevshooteri != SHOOTERIGAIN,
				ddiff = prevshooterd != SHOOTERDGAIN, deaddiff = prevshooterdead != FLYWHEELDEAD,
				outdiff = prevoutputrangelow != SHOOTEROUTPUTRANGELOW || prevoutputrangehigh != SHOOTEROUTPUTRANGEHIGH;
		
		if(pdiff || idiff || ddiff || deaddiff || outdiff)//different or null
		{
			String print = "Different constants: ";
			print += pdiff ? "SHOOTERPGAIN " + SHOOTERPGAIN + " ": "";
			print += idiff ? "SHOOTERIGAIN " + SHOOTERIGAIN + " ": "";
			print += ddiff ? "SHOOTERDGAIN " + SHOOTERDGAIN + " ": "";
			print += deaddiff ? "FLYWHEELDEAD " + FLYWHEELDEAD + " ": "";
			print += outdiff ? "SHOOTEROUTPUTRANGEHIGH " + SHOOTEROUTPUTRANGEHIGH + " SHOOTEROUTPUTRANGELOW " + SHOOTEROUTPUTRANGELOW: "";
			System.out.println(print);
			
			flywheelcontrol.setPID(SHOOTERPGAIN, SHOOTERIGAIN, SHOOTERDGAIN);
			flywheelcontrol.setOutputRange(SHOOTEROUTPUTRANGELOW, SHOOTEROUTPUTRANGEHIGH);
			flywheelcontrol.setAbsoluteTolerance(FLYWHEELDEAD);
		}
		
		/*TakeBackHalf temptbh = new TakeBackHalf(new double[]{SHOOTEROUTPUTRANGELOW, SHOOTEROUTPUTRANGEHIGH}, SHOOTERIGAIN, FLYWHEELMAXSPEED);
		
		if(!temptbh.equals(control))
		{
			control.setGain(SHOOTERIGAIN);
			control.setMAX(FLYWHEELMAXSPEED);
			control.setOutputrange(new double[]{SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW});
		}*/
	}
	
	@Override
	public synchronized void update()//Flywheel and turret control loop
	{
		//=====================
		//=======Shooter=======
		//=====================
		if(shooterManual.isTimeUp())
		{
			if(!flywheelcontrol.isEnabled())
			{
				flywheelcontrol.enable();
			}
			
			switch(flyState)
			{
				case DISTANCE:
					if(vision.targetAvailable())
					{
						double visionval = distToFly.calc(vision.getClosestDist());
						
						if(vision.checkFrameDouble())//new frame? checkFrame resets the flag
						{
							//recalc angle, new vision value (vision error may or may not be in tolerance)
							//System.out.println("Vision: " + visionval + " DEAD: " + TURRETENCODERDEAD);
							if(visionval != lastsetvision)//if this value has not been previously set, set it
							{
								lastsetvision = visionval;
								flyset = visionval;
							}
						}
					}
					else
					{
						flywheelcontrol.disable();
						break;
					}
					
					
				case SETPOINT:
					if(flyset != lastflyset)//new value
					{
						flywheelcontrol.setSetpoint(flyset);
					}
					//if(iters % 15 == 0) System.out.println(OrangeUtility.PIDData(flywheelcontrol));
					break;
			}
			
			if(flywheelcontrol.onTarget() && Math.abs(flywheelcontrol.getError()) < FLYWHEELDEAD)
			{
				flyDone = true;
			}
			
			lastflyset = flyset;
		}
		else//manual control
		{
			if(flywheelcontrol.isEnabled())
			{
				flywheelcontrol.disable();
			}
		}
		
		//======================
		//========Loader========
		//======================
		boolean cancelLoad = !intakeRunning && lastintakeRunning;
		boolean ballsw = loaderSwitch.get();
		if(loaderManual.isTimeUp())
		{
			switch(loaderState)
			{
				case WAITING:
					loader.set(0);
					if(toReload)
					{
						loaderState = LoaderState.RELOADINGFAST;
					}
					else if(intakeRunning)
					{
						loaderState = prevstate;
					}
					else
					{
						loadDone = true;
					}
					
					//Reset state to avoid unpredictable behavior
					toFire = false;
					break;
					
				case RELOADINGFAST:
					loader.set(LOADERSPEED);
					if(cancelLoad)
					{
						cancelLoad = false;
						prevstate = LoaderState.RELOADINGFAST;//remember state
						loaderState = LoaderState.WAITING;
					}
					else if(ballsw)
					{
						loaderState = LoaderState.RELOADINGSLOW;
					}
					
					//Reset state to avoid unpredictable behavior
					toReload = false;
					toFire = false;
					break;
					
				case RELOADINGSLOW:
					loader.set(LOADERSPEEDLOW);
					if(cancelLoad)
					{
						cancelLoad = false;
						prevstate = LoaderState.RELOADINGSLOW;//remember state
						loaderState = LoaderState.WAITING;
					}
					else if(!ballsw)
					{
						prevstate = LoaderState.RELOADINGFAST;//reset state
						loaderState = LoaderState.LOADED;
						loadDone = true;
					}
					
					//Reset state to avoid unpredictable behavior
					toReload = false;
					toFire = false;
					break;
					
				case LOADED:
					loader.set(0);
					loadDone = true;
					
					if(toFire)
					{
						prevstate = LoaderState.RELOADINGFAST;//reset state
						loaderState = LoaderState.FIRING;
						shootingTimer.reset();
						shootingTimer.start();
					}
					
					//Reset state to avoid unpredictable behavior
					toReload = false;
					break;
					
				case FIRING:
					loader.set(LOADERSPEED);
					if(shootingTimer.get() > FIRINGTIME)//done firing
					{
						prevstate = LoaderState.RELOADINGFAST;//reset state
						loaderState = LoaderState.WAITING;
						fireDone = true;
					}
					//Reset state to avoid unpredictable behavior
					toReload = false;
					toFire = false;
					break;
			}
		}
		
		//Check if we're done here 
		//TODO: Decide if the flywheel needs to be in the deadzone for multiple iterations
		
		iters++;
		lastballsw = ballsw;
		lastintakeRunning = intakeRunning;
	}
	
	public double getFlySet()
	{
		return flywheelcontrol.getSetpoint();
	}
	
	public synchronized void stopFly()
	{
		setFly(0);
	}
	
	public LoaderState getLoaderState()
	{
		return loaderState;
	}
	
	public synchronized void setLoaded()
	{
		prevstate = LoaderState.RELOADINGFAST;//reset state
		loaderState = LoaderState.LOADED;
		loadDone = true;
	}
	
	/**
	 * Controls the loader with simple motor values.
	 * @param move the movement value
	 */
	public void manualLoader(double move)
	{
		loaderManual.poke();
		loader.set(move);
	}
	
	/**
	 * Controls the turret with simple motor values.
	 * @param rotate the rotation value
	 */
	public void manualShooter(double speed)
	{
		shooterManual.poke();
		fly1.set(speed*FLYDIRS[0]);
		fly2.set(speed*FLYDIRS[1]);
	}
	
	/**
	 * Notifies the Shooter of the intake's activities.
	 * @param runnning if the shooter is running
	 */
	public synchronized void intakeNotify(boolean running)
	{
		intakeRunning = running;
	}
	
	/**
	 * Tells the loader to reload. Only has an effect when waiting(not loaded or moving).
	 */
	public synchronized void reload()
	{
		toReload = true;
		loadDone = false;
	}
	
	/**
	 * Tells the shooter to fire. Only has an effect when loaded(not waiting or moving).
	 */
	public synchronized void startFire()
	{
		toFire = true;
		fireDone = false;
	}
	
	/**
	 * Resets the loader state to WAITING in case of bad things.
	 */
	public synchronized void resetLoader()
	{
		prevstate = LoaderState.RELOADINGFAST;//reset state
		loaderState = LoaderState.WAITING;
		fireDone = true;
	}
	
	public synchronized boolean isFiring()
	{
		return loaderState == LoaderState.FIRING;
	}
	
	/**
	 * Tells the control loop to change the flywheel rpm. 
	 * @param rpm the setpoint for the flywheel in RPM
	 */
	public void setFly(double rpm)
	{
		flyDone = false;
		flyset = rpm;
		flyState = FlyState.SETPOINT;
	}
	
	public synchronized void setFly()
	{
		flyDone = false;
		flyState = FlyState.DISTANCE;
	}
	
	public void setHood(HoodState state)
	{
		hood.set(state != HoodState.LOW);//low by default
	}
	
	public String toString()
	{
		return "Shooter";
	}
	
	@Override
	public void stopThreads()
	{
		task.hold();
	}
	
	@Override
	public void startThreads()
	{
		task.resume();
	}
	
	public void terminateThreads()
	{
		task.terminate();
		try
		{
			shooterThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}
	
	public synchronized void end()
	{
		flyDone = true;
		//control.setSetpoint(0);
		flywheelcontrol.setSetpoint(0);
		flywheelcontrol.disable();
	}
	
	@Override
	public boolean threadsActive()
	{
		return task.isActive();
	}
}
