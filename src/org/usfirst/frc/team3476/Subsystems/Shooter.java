package org.usfirst.frc.team3476.Subsystems;


import java.util.Arrays;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.DIOHomer;
import org.usfirst.frc.team3476.Utility.DIOHomer.HomeState;
import org.usfirst.frc.team3476.Utility.ManualHandler;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Control.BangBang;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.SmoothBangBang;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper.Data;
import org.usfirst.frc.team3476.Utility.Control.TakeBackHalf;

import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDController.Tolerance;
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
			"FLY1DIR", "FLY2DIR", "FLYWHEELDEAD", "FLYWHEELMAXSPEED", "TURRETENCODERP", "TURRETENCODERI", "TURRETENCODERD",
			"TURRETENCODERDEAD", "LOADERSPEED", "SOFTLIMITS", "USESOFT", "HOMINGDIR", "HOMEHIGH",
			"HOMESEEKSPEED", "HOMESLOWSPEED", "HOMETIMEOUT", "NUMSMOOTHSAMPLES", "SHOOTERPGAIN", "SHOOTERDGAIN"};
	private final double RPSPERRPM = 60, TURRETOUTPUTRANGE = 0.25, SOFTRESETANGLE = 1, ROTATIONSPERTICK = 2.2379557291666666666666666666667e-5;
	
	enum AimMode{VISION, ENCODER}
	AimMode aimmode;
	enum LoaderState{RELOADING, LOADED, FIRING, WAITING}
	LoaderState loaderState;
	
	private double SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW, SHOOTERIGAIN, FLYWHEELDEAD,
	FLYWHEELMAXSPEED, TURRETENCODERP, TURRETENCODERI, TURRETENCODERD,
	TURRETENCODERDEAD, LOADERSPEED, SOFTLIMITS, USESOFT, HOMINGDIR, HOMEHIGH,
	HOMESEEKSPEED, HOMESLOWSPEED, HOMETIMEOUT, NUMSMOOTHSAMPLES,
	SHOOTERPGAIN, SHOOTERDGAIN;
	private double aimangle, lastaimangle, softDir, lastvision;
	private double[] FLYDIRS;
	private boolean flyDone, loadDone, toFire, toReload, firingLast,
					pass1, turretdone, intakeRunning, softlimitsPassed,
					resettingSoft, softflag, onereturn;
	private SpeedController fly1, fly2, loader;
	private CANTalon turret;
	private TakeBackHalf control;
	private Counter tach;
	private Timer shootingTimer;
	
	private PIDDashdataWrapper vision;
	private PIDCANTalonEncoderWrapper encoder;
	private PIDController turretencodercontrol;
	//private PIDController flywheelcontrol;
	private DigitalInput loaderSwitch, halleffect;
	
	private SubsystemTask task;
	private Thread flyThread;
	
	private RunningAverage tachavg;
	
	private int iters;
	private boolean lastturretdone;
	
	final long MANUALTIMEOUT = 50;//in ms
	private ManualHandler shooterManual, loaderManual, turretManual;
	
	private DIOHomer turretHomer;
	
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
	public Shooter(SpeedController fly1in, SpeedController fly2in, SpeedController loaderin, CANTalon turretin,
					Counter tachin, DigitalInput loaderSwitchin, DigitalInput halleffectin)
	{
		//Turret setup
		turret = turretin;
		turretdone = true;
		aimmode = AimMode.ENCODER;
		loaderState = LoaderState.WAITING;
		softlimitsPassed = false;
		softDir = 1;
		resettingSoft = false;
		halleffect = halleffectin;
		
		//Vision tracking control
		vision = new PIDDashdataWrapper(Data.VISIONX);
		
		//Encoder control
		aimangle = 0;
		lastaimangle = 0;
		encoder = new PIDCANTalonEncoderWrapper(turret, ROTATIONSPERTICK);
		encoder.reset();
		turretencodercontrol = new PIDController(0, 0, 0, encoder, turret);
		turretencodercontrol.disable();
		turretencodercontrol.setOutputRange(-TURRETOUTPUTRANGE, TURRETOUTPUTRANGE);
		turretencodercontrol.setToleranceBuffer(6);
		
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
		FLYDIRS = new double[2];
		
		//Flywheel
		control = new TakeBackHalf(new double[]{0, 0}, 0, 0);
		//control = new TakeBackHalf(new double[]{0, 0}, 1);
		
		//Loader
		loaderSwitch = loaderSwitchin;
		loader = loaderin;
		
		//Manuals
		shooterManual = new ManualHandler(MANUALTIMEOUT);
		turretManual = new ManualHandler(MANUALTIMEOUT);
		loaderManual = new ManualHandler(MANUALTIMEOUT);
		
		iters = 0;
		task = new SubsystemTask(this, 4);
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
			case "fire":
				startFire();
				break;
			case "reload":
				startFire();
				break;
		}
	}
	
	private double getRotations()
	{
		return turret.getPosition()*ROTATIONSPERTICK;
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
		double 	prevencoderdead = TURRETENCODERDEAD,
				prevencoderp = TURRETENCODERP,
				prevencoderi = TURRETENCODERI,
				prevencoderd = TURRETENCODERD;
		
		double 	prevshooterp = SHOOTERPGAIN,
				prevshooteri = SHOOTERIGAIN,
				prevshooterd = SHOOTERDGAIN;
		
		int i = 0;
		SHOOTEROUTPUTRANGEHIGH = constantsin[i];
		i++;//1
		SHOOTEROUTPUTRANGELOW = constantsin[i];
		i++;//2
		SHOOTERIGAIN = constantsin[i];
		i++;//3
		FLYDIRS[i - 3] = constantsin[i];
		FLYDIRS[i - 3] = 1;
		i++;//4
		FLYDIRS[i - 3] = constantsin[i];
		FLYDIRS[i - 3] = -1;
		i++;//5
		FLYWHEELDEAD = constantsin[i];
		i++;//6
		FLYWHEELMAXSPEED = constantsin[i];
		i++;//7
		TURRETENCODERP = constantsin[i];
		i++;//8
		TURRETENCODERI = constantsin[i];
		i++;//9
		TURRETENCODERD = constantsin[i];
		i++;//10
		TURRETENCODERDEAD = constantsin[i];
		i++;//11
		LOADERSPEED = constantsin[i];
		i++;//12
		SOFTLIMITS = constantsin[i];
		i++;//13
		USESOFT = constantsin[i];
		i++;//14
		HOMINGDIR = constantsin[i];
		i++;//15
		HOMEHIGH = constantsin[i];
		i++;//16
		HOMESEEKSPEED = constantsin[i];
		i++;//17
		HOMESLOWSPEED = constantsin[i];
		i++;//18
		HOMETIMEOUT = constantsin[i];
		i++;//19
		NUMSMOOTHSAMPLES = constantsin[i];
		i++;//20
		SHOOTERPGAIN = constantsin[i];
		i++;//21
		SHOOTERDGAIN = constantsin[i];
		
		//System.out.println("Shooter constants: " + Arrays.toString(constantsin));
		
		//System.out.println("Setting PID");
		//System.out.println("P: " + TURRETENCODERP);
		
		/*System.out.println("prevencoderp: " + prevencoderp + "TURRETENCODERP" + TURRETENCODERP +
				"prevencoderi: " + prevencoderi + "TURRETENCODERI" + TURRETENCODERI +
				"prevencoderd: " + prevencoderd + "TURRETENCODERD" + TURRETENCODERD +
				"prevencoderdead: " + prevencoderdead + "TURRETENCODERDEAD" + TURRETENCODERDEAD);*/
		
		if(	prevencoderp != TURRETENCODERP ||
			prevencoderi != TURRETENCODERI || prevencoderd != TURRETENCODERD ||
			prevencoderdead != TURRETENCODERDEAD)//different or null
		{
			String print = "Different constants: ";
			print += prevencoderp != TURRETENCODERP ? "TURRETENCODERP ": "";
			print += prevencoderi != TURRETENCODERI ? "TURRETENCODERI ": "";
			print += prevencoderd != TURRETENCODERD ? "TURRETENCODERD ": "";
			print += prevencoderdead != TURRETENCODERDEAD ? "TURRETENCODERDEAD": "";
			System.out.println(print);
			
			turretencodercontrol.setPID(TURRETENCODERP, TURRETENCODERI, TURRETENCODERD);
			turretencodercontrol.setAbsoluteTolerance(TURRETENCODERDEAD);
		}
		
		if(	prevshooterp != SHOOTERPGAIN ||
			prevshooteri != SHOOTERIGAIN ||
			prevshooterd != SHOOTERDGAIN)//different or null
		{
			String print = "Different constants: ";
			print += prevshooterp != SHOOTERPGAIN ? "SHOOTERPGAIN ": "";
			print += prevshooteri != SHOOTERIGAIN ? "SHOOTERIGAIN ": "";
			print += prevshooterd != SHOOTERDGAIN ? "SHOOTERDGAIN ": "";
			System.out.println(print);
			
			turretencodercontrol.setPID(TURRETENCODERP, TURRETENCODERI, TURRETENCODERD);
			turretencodercontrol.setAbsoluteTolerance(TURRETENCODERDEAD);
		}
		
		/*TakeBackHalf temptbh = new TakeBackHalf(new double[]{SHOOTEROUTPUTRANGELOW, SHOOTEROUTPUTRANGEHIGH}, SHOOTERIGAIN, FLYWHEELMAXSPEED);
		
		if(!temptbh.equals(control))
		{
			control.setGain(SHOOTERIGAIN);
			control.setMAX(FLYWHEELMAXSPEED);
			control.setOutputrange(new double[]{SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW});
		}*/
		
		DIOHomer temp = new DIOHomer(halleffect, HOMESEEKSPEED, HOMESLOWSPEED, HOMINGDIR, HOMEHIGH != 0, (long)HOMETIMEOUT);
		
		if(!temp.equals(turretHomer))
		{
			turretHomer = temp;
		}
	}

	@Override
	public synchronized void update()//Flywheel and turret control loop
	{
		
		
		//=====================
		//=======Shooter=======
		//=====================
		double process = RPSPERRPM/tach.getPeriod();//Get rps > to rpm
		double output = 0;
		
		if(shooterManual.isTimeUp())
		{
			if(control == null)
			{
				throw new NullPointerException("No TakeBackHalf controller in Subsystem \"" + this +  "\" - constants not returned");
			}
			else
			{
				output = control.output(process);
			}
			//OrangeUtility.ControlData(control.getError(process), process, output)
			/*if(iters % 20 == 0)
			{
				String print = OrangeUtility.ControlLoopData(control, process) + "\nI: " + control.getGain() + "\nSetpoint: " + control.getMax();
				System.out.println(print);
			}*/
			//output = control.getSetpoint() > 0 ? output : 0;
			fly1.set(output*FLYDIRS[0]);
			fly2.set(output*FLYDIRS[1]);
		}
		
		//======================
		//========Loader========
		//======================
		if(loaderManual.isTimeUp())
		{
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
		}
			
		
		//======================
		//========Turret========
		//======================
		
		//Initializes the turretHomer when ready
		if(turretHomer != null && turretHomer.isStopped())
		{
			if(turretManual.isTimeUp())
			{
				//checks whether or not the encoder control is being bad, does a 360 noscope if so
				if(turretencodercontrol.isEnabled())
				{
					if(!softAcceptable(turretencodercontrol.get()))
					{
						aim(encoder.getDistance() - softDir*SOFTRESETANGLE);
						resettingSoft = true;
						softflag = true;
					}
				}
				
				if(!turretdone)
				{
					switch(aimmode)
					{
						case VISION:
							//If first exec, make sure we're using the right control
							/*if(lastturretdone)
							{
								turretencodercontrol.disable();
								turretvisioncontrol.enable();
							}*/
							
							//If not enabled, do it
							if(targetAvailable())
							{
								double visionval = vision.pidGet();
								
								if(visionval != lastvision)//recalc angle, new vision value
								{
									aimangle = encoder.getDistance() + rectifyAngle(visionval);
									lastvision = visionval;
								}
							}
							else
							{
								turretencodercontrol.disable();
								break;
							}
								//If not enabled, do it
								/*if(!turretvisioncontrol.isEnabled())
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
							break;*/
							
						case ENCODER:
							//If first exec, make sure we're using the right control
							if(lastturretdone)
							{
								turretencodercontrol.reset();
								turretencodercontrol.enable();
							}
							
							//If not enabled, do it
							/*if(!turretencodercontrol.isEnabled())
							{
								turretencodercontrol.enable();
							}*/
							
							//blocks the setting of the setpoint if we are resetting due to a soft limit failure
							//softflag allows the setpoint to be set once - for the reset, and no more
							if(!resettingSoft || softflag)
							{
								if(softflag)
								{
									turretencodercontrol.setSetpoint(aimangle);
									softflag = false;
								}
								else
								{
									//setSetpoint resets the average error buffer - onTarget doesn't work
									if(lastaimangle != aimangle)
									{
										turretencodercontrol.setSetpoint(aimangle);
									}
								}
							}
							
							/*if(iters % 15 == 0)
							{
								System.out.println(OrangeUtility.PIDData(turretencodercontrol));
								System.out.println("Absolute Tolerance: " + TURRETENCODERDEAD + "\n==============");
							}*/
							
							if(turretencodercontrol.onTarget() && Math.abs(turretencodercontrol.getError()) < TURRETENCODERDEAD)
							{
								System.out.println("onTarget error: " + turretencodercontrol.getError());
								turretdone = true;
								resettingSoft = false;
								turretencodercontrol.disable();
							}
							break;
					}
				}
				else
				{
					if(turretencodercontrol.isEnabled())
					{
						turretencodercontrol.disable();
					}
				}
			}
			else
			{
				if(turretencodercontrol.isEnabled())
				{
					turretencodercontrol.disable();
				}
			}
			
			if(USESOFT != 0)//use soft limits
			{
				double enc = getRotations();
				if(enc >= SOFTLIMITS)//softlimits passed
				{
					softlimitsPassed = true;
					softDir = 1;//signifies the direction that softlimits have been passed
				}
				else if(enc <= SOFTLIMITS)
				{
					softlimitsPassed = true;
					softDir = -1;
				}
				else
				{
					softlimitsPassed = false;
				}
			}
		}
		else//homing
		{
			if(turretHomer != null)
			{
				turretHomer.update();
				if(turretHomer.isHome())
				{
					encoder.reset();
					turret.set(turretHomer.getSpeed());
					OrangeUtility.sleep(100);
					System.out.println("home: " + encoder.get() + ", " + encoder.getDistance());
					if(encoder.get() > 1500)System.out.println("wtf encoder");
				}
				else if(turretHomer.getState() == HomeState.TIMEOUT)
				{
					turret.set(turretHomer.getSpeed());
					System.out.println("Homer timed out");
				}
				else
				{
					turret.set(turretHomer.getSpeed());
				}
			}
		}
		lastaimangle = aimangle;
		
		//Check if we're done here 
		//TODO: Decide if the flywheel needs to be in the deadzone for multiple iterations
		
		if(Math.abs(control.getSetpoint() - process) < FLYWHEELDEAD)
		{
			flyDone = true;
		}
		iters++;
		
		lastturretdone = turretdone;
	}
	
	public CANTalon getTurretMotor()
	{
		return turret;
	}
	
	public void resetHomer()
	{
		turretHomer.restart();
	}
	
	public void killHomer()
	{
		turretHomer.kill();
	}
	
	/**
	 * Controls the turret with simple motor values.
	 * @param rotate the rotation value
	 */
	public void manualTurret(double rotate)
	{
		turretManual.poke();
		if(softAcceptable(rotate))
		{
			turret.set(rotate);
		}
		else
		{
			turret.set(0);
		}
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
	
	private double rectifyAngle(double angle)
	{
		double enc = encoder.getDistance();
		double localEnc = convertAngleLocal(enc);
		angle = convertAngleLocal(angle);
		
		double counterclockwise = localEnc - angle;
		double clockwise = localEnc + angle;
		
		if(Math.abs(counterclockwise) > Math.abs(clockwise)) //cw most efficient
		{
			if(enc + clockwise <= SOFTLIMITS)//cw legal
			{
				return clockwise;
			}
			else if(enc + counterclockwise >= -SOFTLIMITS) //cw illegal, ccw legal
			{
				return counterclockwise;
			}
			else //both illegal
			{
				return enc;
			}
		}
		else //ccw most efficient
		{
			if(enc + counterclockwise >= -SOFTLIMITS) //ccw legal
			{
				return clockwise;
			}
			else if(enc + clockwise <= SOFTLIMITS) //ccw illegal, cw legal
			{
				return counterclockwise;
			}
			else //both illegal
			{
				return enc;
			}
		}
	}
	
	private double convertAngleLocal(double angle)
	{
		angle = angle % 1;
		
		if(angle >= 0.5)
		{
			angle -= 1;
		}
		else if(angle <= -0.5)
		{
			angle += 1;
		}
		return angle;
	}
	
	/**
	 * Returns whether this move will violate soft limits based on the state of the shooter
	 * @param rotate the value to check
	 * @return true if acceptable
	 */
	public boolean softAcceptable(double rotate)
	{
		return !softlimitsPassed || (softDir * rotate) < 0;
	}
	
	/**
	 * Controls the turret with simple motor values.
	 * @param rotate the rotation value
	 */
	public void manualShooter(double speed)
	{
		shooterManual.poke();
		fly1.set(speed);
		fly2.set(speed);
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
		if(!resettingSoft)
		{
			turretdone = false;
			aimmode = AimMode.VISION;
		}
	}
	
	/**
	 * Aims the turret to the specified angle.
	 * @param angle the angle to turn to
	 */
	public synchronized void aim(double angle)
	{
		if(!resettingSoft)
		{
			turretdone = false;
			aimangle = angle;
			aimmode = AimMode.ENCODER;
		}
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
	
	/**
	 * Tells the control loop to change the flywheel rpm. 
	 * @param rpm the setpoint for the flywheel in RPM
	 */
	public void setFly(double rpm)
	{
		flyDone = false;
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
