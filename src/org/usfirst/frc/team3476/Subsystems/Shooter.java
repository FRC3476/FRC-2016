package org.usfirst.frc.team3476.Subsystems;


import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper.Data;
import org.usfirst.frc.team3476.Utility.Control.TakeBackHalf;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.hal.DIOJNI;

//TODO: javadoc comments if not specified by Subsystem
/*
 * Represents the robot's shooter.
 */
public class Shooter implements Subsystem
{
	private final String[] autoCommands = {"shooter", "visionaim", "encoderaim", "flywheel", "fire", "reload"};
	private final String[] constants = {"SHOOTEROUTPUTRANGEHIGH", "SHOOTEROUTPUTRANGELOW", "SHOOTERIGAIN",
			"FLY1DIR", "FLY2DIR", "FLYWHEELDEAD", "FLYWHEELMAXSPEED", "TURRETVISIONP", "TURRETVISIONI",
			"TURRETVISIOND", "TURRETVISIONDEAD", "TURRETENCODERP", "TURRETENCODERI", "TURRETENCODERD",
			"TURRETENCODERDEAD", "LOADERSPEED"};
	private final double RPSPERRPM = 60, TURRETOUTPUTRANGE = 0.25;
	
	enum AimMode{VISION, ENCODER}
	AimMode aimmode;
	enum LoaderState{RELOADING, LOADED, FIRING, WAITING}
	LoaderState loaderState;
	
	private double SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW, SHOOTERIGAIN, FLYWHEELDEAD,
	FLYWHEELMAXSPEED, TURRETVISIONP, TURRETVISIONI, TURRETVISIOND, TURRETVISIONDEAD, TURRETENCODERP,
	TURRETENCODERI, TURRETENCODERD, TURRETENCODERDEAD, LOADERSPEED;
	private double aimangle;
	private double[] FLYDIRS;
	private boolean flyDone, loadDone, toFire, toReload, firingLast, pass1, turretdone, intakeRunning;
	private SpeedController fly1, fly2, turret, loader;
	private TakeBackHalf control;
	private Counter tach;
	private Timer shootingTimer;
	
	private PIDDashdataWrapper vision;
	private Encoder encoder;
	private PIDController turretvisioncontrol;
	private PIDController turretencodercontrol;
	private DigitalInput loaderSwitch;
	
	private SubsystemTask task;
	private Thread flyThread;
	
	private int iters;
	private boolean lastturretdone;
	
	/**
	 * 
	 * @param fly1in
	 * @param fly2in
	 * @param loaderin
	 * @param turretin
	 * @param tachin
	 * @param encoderin
	 * @param loaderSwitchin
	 */
	public Shooter(SpeedController fly1in, SpeedController fly2in, SpeedController loaderin, SpeedController turretin, Counter tachin, Encoder encoderin, DigitalInput loaderSwitchin)
	{
		//Turret setup
		turret = turretin;
		turretdone = true;
		aimmode = AimMode.ENCODER;
		loaderState = LoaderState.WAITING;
		
		//Vision tracking control
		vision = new PIDDashdataWrapper(Data.VISIONX);
		turretvisioncontrol = new PIDController(0, 0, 0, vision, turret);
		turretvisioncontrol.disable();
		turretvisioncontrol.setOutputRange(-TURRETOUTPUTRANGE, TURRETOUTPUTRANGE);
		
		//Encoder control
		aimangle = 0;
		encoder = encoderin;
		turretencodercontrol = new PIDController(0, 0, 0, encoder, turret);
		turretencodercontrol.disable();
		turretencodercontrol.setOutputRange(-TURRETOUTPUTRANGE, TURRETOUTPUTRANGE);
		
		//Shooter
		fly1 = fly1in;
		fly2 = fly2in;
		flyDone = true;
		loadDone = true;
		tach = tachin;
		toFire = false;
		toReload = false;
		shootingTimer = new Timer();
		FLYDIRS = new double[2];
		
		//Loader
		loaderSwitch = loaderSwitchin;
		loader = loaderin;
		
		iters = 0;
		task = new SubsystemTask(this, 10);
		flyThread = new Thread(task, "flyThread");
		flyThread.start();
	}
	
	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}
	
	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		switch(command)
		{
			case "shooter":
				setFly(params[1]);
				break;
			case "visionaim":
				aim();
				break;
			case "encoderaim":
				aim(params[0]);
				break;
			case "flywheel":
				setFly(params[0]);
				break;
			//TODO: do firing right
			case "fire":
				startFire();
				break;
			case "reload":
				startFire();
				break;
		}
	}
	
	@Override
	public synchronized boolean isAutoDone()
	{
		return flyDone && loadDone;
	}
	
	@Override
	public String[] getConstantRequest()//Request all needed constants
	{
		return constants;
	}
	
	@Override
	public synchronized void returnConstantRequest(double[] constantsin)//Get all needed constants
	{
		int i = 0;
		SHOOTEROUTPUTRANGEHIGH = constantsin[i];
		i++;//1
		SHOOTEROUTPUTRANGELOW = constantsin[i];
		i++;//2
		SHOOTERIGAIN = constantsin[i];
		i++;//3
		FLYDIRS[i - 3] = constantsin[i];
		i++;//4
		FLYDIRS[i - 3] = constantsin[i];
		i++;//5
		FLYWHEELDEAD = constantsin[i];
		i++;//6
		FLYWHEELMAXSPEED = constantsin[i];
		i++;//7
		TURRETVISIONP = constantsin[i];
		i++;//8
		TURRETVISIONI = constantsin[i];
		i++;//9
		TURRETVISIOND = constantsin[i];
		i++;//10
		TURRETVISIONDEAD = constantsin[i];
		i++;//11
		TURRETENCODERP = constantsin[i];
		i++;//12
		TURRETENCODERI = constantsin[i];
		i++;//13
		TURRETENCODERD = constantsin[i];
		i++;//14
		TURRETENCODERDEAD = constantsin[i];
		i++;//15
		LOADERSPEED = constantsin[i];
		
		//System.out.println("Setting PID");
		turretvisioncontrol.setAbsoluteTolerance(TURRETVISIONDEAD);
		turretencodercontrol.setAbsoluteTolerance(TURRETENCODERDEAD);
		//System.out.println("P: " + TURRETENCODERP);
	}

	//TODO: add firing logic
	@Override
	public synchronized void update()//Flywheel and turret control loop
	{
		//=====================
		//=======Shooter=======
		//=====================
		double output = 0;
		double process = tach.getRate()/RPSPERRPM;//Get rps > to rpm
		if(control == null)
		{
			throw new NullPointerException("No TakeBackHalf controller in Subsystem \"" + this +  "\" - constants not returned");
		}
		else
		{
			output = control.output(process);
		}
		//output = control.getSetpoint() > 0 ? 1 : 0;
		fly1.set(output*FLYDIRS[0]);
		fly2.set(output*FLYDIRS[1]);
		
		//======================
		//========Loader========
		//======================
		switch(loaderState)
		{
			case WAITING:
				loader.set(0);
				if(intakeRunning || toReload)
				{
					loaderState = LoaderState.RELOADING;
				}
				loadDone = true;
				
				//Reset state to avoid unpredictable behavior
				toFire = false;
				break;
				
			case RELOADING:
				loader.set(LOADERSPEED);
				if(loaderSwitch.get())
				{
					loaderState = LoaderState.LOADED;
				}
				
				//Reset state to avoid unpredictable behavior
				toReload = false;
				toFire = false;
				break;
				
			case LOADED:
				loader.set(0);
				if(toFire)
				{
					loaderState = LoaderState.FIRING;
				}
				loadDone = true;
				
				//Reset state to avoid unpredictable behavior
				toReload = false;
				break;
				
			case FIRING:
				loader.set(LOADERSPEED);
				if(!loaderSwitch.get())
				{
					loaderState = LoaderState.WAITING;
				}
				
				//Reset state to avoid unpredictable behavior
				toReload = false;
				toFire = false;
				break;
		}
		
		//Turret update
		if(!turretdone)
		{
			switch(aimmode)
			{
				case VISION:
					//If first exec, make sure we're using the right control
					if(lastturretdone)
					{
						turretencodercontrol.disable();
						turretvisioncontrol.enable();
					}
					
					//If not enabled, do it
					if(targetAvailable())
					{
						if(iters % 5 == 0) System.out.println(OrangeUtility.PIDData(turretvisioncontrol));
						
						//If not enabled, do it
						if(!turretvisioncontrol.isEnabled())
						{
							turretvisioncontrol.enable();
						}
						
						turretvisioncontrol.setSetpoint(0);
						
						if(turretvisioncontrol.onTarget())
						{
							turretdone = true;
							turretvisioncontrol.disable();
						}
					}
					else
					{
						turretvisioncontrol.disable();
					}
					break;
					
				case ENCODER:
					//If first exec, make sure we're using the right control
					if(lastturretdone)
					{
						turretencodercontrol.enable();
						turretvisioncontrol.disable();
					}
					
					//If not enabled, do it
					if(!turretencodercontrol.isEnabled())
					{
						turretencodercontrol.enable();
					}
					
					turretencodercontrol.setSetpoint(aimangle);
					
					if(turretencodercontrol.onTarget())
					{
						turretdone = true;
						turretencodercontrol.disable();
					}
					break;
			}
		}
		else
		{
			turretencodercontrol.disable();
			turretvisioncontrol.disable();
		}
		
		//Check if we're done here 
		//TODO: Decide if the flywheel needs to be in the deadzone for multiple iterations
		
		if(Math.abs(control.getSetpoint() - process) < FLYWHEELDEAD)
		{
			flyDone = true;
		}
		iters++;
		
		lastturretdone = turretdone;
	}
	
	/**
	 * Notifies the Shooter of the intake's activities.
	 * @param runnning if the shooter is running
	 */
	public void intakeNotify(boolean runnning)
	{
		intakeRunning = runnning;
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
		loadDone = false;
	}
	
	/**
	 * Resets the loader state to WAITING in case of bad things.
	 */
	public synchronized void resetLoader()
	{
		loaderState = LoaderState.WAITING;
	}
	
	public synchronized boolean isFiring()
	{
		return loaderState == LoaderState.FIRING;
	}
	
	/**
	 * @return True if a vision target is available
	 */
	public synchronized boolean targetAvailable()
	{
		return vision.pidGet() != Double.NaN;
	}
	
	/**
	 * Aims the turret at the largest vision target.
	 */
	public synchronized void aim()
	{
		turretdone = false;
		aimmode = AimMode.VISION;
		turretvisioncontrol.setPID(TURRETVISIONP, TURRETVISIONI, TURRETVISIOND);
	}
	
	/**
	 * Aims the turret to the specified angle.
	 * @param angle the angle to turn to
	 */
	public synchronized void aim(double angle)
	{
		turretdone = false;
		aimangle = angle;
		aimmode = AimMode.ENCODER;
		turretencodercontrol.setPID(TURRETENCODERP, TURRETENCODERI, TURRETENCODERD);
	}
	
	/**
	 * Stops the aiming for the turret.
	 * In the case of encoder aiming, stops prematurely.
	 * In the case of vision aiming, stops normally.
	 */
	public synchronized void stopAim()
	{
		System.out.println("Stopaim");
		turretdone = true;
	}
	
	//TODO: do we need this?
	/*public synchronized void loader(Load dir)
	{
		switch(dir)
		{
			case IN:
				loader.set(false);
				break;
			case OUT:
				loader.set(true);
				break;
		}
	}
	
	public synchronized Load getLoader()
	{
		return loader.get() ? Load.OUT : Load.IN;
	}*/
	
	/**
	 * Tells the control loop to change the flywheel rpm. 
	 * @param rpm the setpoint for the flywheel in RPM
	 */
	public void setFly(double rpm)
	{
		flyDone = false;
		control.setGain(SHOOTERIGAIN);
		control.setMAX(FLYWHEELMAXSPEED);
		control.setOutputrange(new double[]{SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW});
		control.setSetpoint(rpm);
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
			flyThread.join();
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
		turretdone = true;
		control.setSetpoint(0);
	}
	
	@Override
	public boolean threadsActive()
	{
		return task.isActive();
	}
}
