package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Subsystems.Shooter.LoaderState;
import org.usfirst.frc.team3476.Utility.DIOHomer;
import org.usfirst.frc.team3476.Utility.ManualHandler;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Control.BangBang;
import org.usfirst.frc.team3476.Utility.Control.DonutCANTalon;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.TakeBackHalf;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper.Data;
import org.usfirst.frc.team3476.Utility.DIOHomer.HomeState;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class Turret implements Subsystem
{
	enum AimMode{VISION, ENCODER}
	
	private final String[] autoCommands = {"visionaim", "encoderaim", "search", "searchleft", "searchright", "waitforhome"};
	
	private final String[] constants = {"TURRETENCODERP", "TURRETENCODERI", "TURRETENCODERD",
										"TURRETENCODERDEAD", "SOFTLIMITS", "USESOFT", "HOMINGDIR",
										"HOMEHIGH", "HOMESEEKSPEED", "HOMESLOWSPEED", "HOMETIMEOUT",
										"RESETSPEED", "RESETDEAD", "TURRETOUTPUTRANGE", "TURRETDONUT",
										"TURRETCLAMP", "VISIONTHROWAWAY", "VISIONAVERAGEQTY"};
	
	private double 	TURRETENCODERP, TURRETENCODERI, TURRETENCODERD, TURRETENCODERDEAD, SOFTLIMITS,
					USESOFT, HOMINGDIR, HOMEHIGH, HOMESEEKSPEED, HOMESLOWSPEED, HOMETIMEOUT, RESETSPEED,
					RESETDEAD, TURRETOUTPUTRANGE, TURRETDONUT, TURRETCLAMP, VISIONTHROWAWAY, VISIONAVERAGEQTY;
	
	private int iters, softDir;
	private boolean softlimitsPassed, resettingSoft, turretdone;
	private double lastaimangle, aimangle;
	
	private final double ROTATIONSPERTICK = 2.2379557291666666666666666666667e-5, PIDOUTPUT = 0, FRAMEPERIOD = 110/1000.0;
	private final int SYSTEMEXECTIME = 2, PIDEXECTIME = 1;//ms
	final long MANUALTIMEOUT = 50;//in ms
	private ManualHandler turretManual;
	
	private AimMode aimmode;
	private DonutCANTalon turret;
	private DigitalInput halleffect;
	private DIOHomer turretHomer;
	
	private PIDController turretencodercontrol;
	private PIDCANTalonEncoderWrapper encoder;
	
	private PIDDashdataWrapper vision;
	private RunningAverage visionavg;
	private int visionthrowawaycount;
	enum VisionMode{GO, DISCARD, AVERAGE}
	private VisionMode visionMode;
	private Timer visiontimer;
	
	private BangBang resetBang;
	
	private SubsystemTask task;
	private Thread turretThread;

	private boolean softFlag;

	private boolean lastturretdone;

	private double lastsetvision, lastvision;

	private double lastenc;

	private boolean searching;

	private boolean waitingforhome;

	private boolean turretautodone;

	public Turret(DonutCANTalon turretin, DigitalInput halleffectin, PIDDashdataWrapper visionin)
	{
		//Turret setup
		turret = turretin;
		turretdone = true;
		turretautodone = false;
		aimmode = AimMode.ENCODER;
		softlimitsPassed = false;
		softDir = 1;
		resettingSoft = false;
		halleffect = halleffectin;
		
		//Vision tracking control
		vision = visionin;
		visionavg = new RunningAverage(1);
		visionthrowawaycount = 0;
		visionMode = VisionMode.AVERAGE;
		visiontimer = new Timer();
		visiontimer.start();
		
		searching = false;
		
		//Encoder control
		aimangle = 0;
		lastaimangle = 0;
		turretdone = true;
		lastturretdone = true;
		encoder = new PIDCANTalonEncoderWrapper(turret, ROTATIONSPERTICK);
		encoder.reset();
		turretencodercontrol = new PIDController(0, 0, 0, encoder, turret, ((double)PIDEXECTIME)/1E3);//convert to seconds for stupid
		turretencodercontrol.disable();
		turretencodercontrol.setOutputRange(-PIDOUTPUT, PIDOUTPUT);
		turretencodercontrol.setToleranceBuffer(100);
		
		//Resetting
		resetBang = new BangBang(new double[]{0, 0});
		
		turretManual = new ManualHandler(MANUALTIMEOUT);
		
		iters = 0;
		task = new SubsystemTask(this, SYSTEMEXECTIME);//4 ms minimum exec time
		turretThread = new Thread(task, "turretThread");
		turretThread.start();
	}

	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}

	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		System.out.println(this + " " + command);
		switch(command)
		{
			case "visionaim":
				aim();
				break;
			case "encoderaim":
				aim(params[0]);
				break;
			case "search":
				search(params[0]);
				break;
			case "searchleft":
				search(-0.1);
				break;
			case "searchright":
				search(0.1);
				break;
			case "waitforhome":
				waitingforhome = true;
				break;
		}
	}

	@Override
	public synchronized boolean isAutoDone()
	{
		if(waitingforhome)
		{
			if(!(turretHomer != null && turretHomer.isStopped()))
			{
				waitingforhome = false;
				return true;
			}
		}
		return turretdone;
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
				prevencoderd = TURRETENCODERD,
				prevoutputrange = TURRETOUTPUTRANGE;
		
		double 	prevturretdonut = TURRETDONUT,
				prevturretclamp = TURRETCLAMP;
		
		double prevvisionavg = VISIONAVERAGEQTY;
		
		int i = 0;
		TURRETENCODERP = constantsin[i];
		i++;//1
		TURRETENCODERI = constantsin[i];
		i++;//2
		TURRETENCODERD = constantsin[i];
		i++;//3
		TURRETENCODERDEAD = constantsin[i];
		i++;//4
		SOFTLIMITS = constantsin[i];
		i++;//5
		USESOFT = constantsin[i];
		i++;//6
		HOMINGDIR = constantsin[i];
		i++;//7
		HOMEHIGH = constantsin[i];
		i++;//8
		HOMESEEKSPEED = constantsin[i];
		i++;//9
		HOMESLOWSPEED = constantsin[i];
		i++;//10
		HOMETIMEOUT = constantsin[i];
		i++;//11
		RESETSPEED = constantsin[i];
		i++;//12
		RESETDEAD = constantsin[i];
		i++;//13
		TURRETOUTPUTRANGE = constantsin[i];
		i++;//14
		TURRETDONUT = constantsin[i];
		i++;//15
		TURRETCLAMP = constantsin[i];
		i++;//16
		VISIONTHROWAWAY = constantsin[i];
		i++;//17
		VISIONAVERAGEQTY = constantsin[i];
		
		//System.out.println("Shooter constants: " + Arrays.toString(constantsin));
		
		//System.out.println("Setting PID");
		//System.out.println("P: " + TURRETENCODERP);
		
		/*System.out.println("prevencoderp: " + prevencoderp + "TURRETENCODERP" + TURRETENCODERP +
				"prevencoderi: " + prevencoderi + "TURRETENCODERI" + TURRETENCODERI +
				"prevencoderd: " + prevencoderd + "TURRETENCODERD" + TURRETENCODERD +
				"prevencoderdead: " + prevencoderdead + "TURRETENCODERDEAD" + TURRETENCODERDEAD);*/
		
		boolean pdiff = prevencoderp != TURRETENCODERP, idiff = prevencoderi != TURRETENCODERI,
				ddiff = prevencoderd != TURRETENCODERD, deaddiff = prevencoderdead != TURRETENCODERDEAD,
				outdiff = prevoutputrange != TURRETOUTPUTRANGE;
		
		if(pdiff || idiff || ddiff || deaddiff || outdiff)
		{
			String print = "Different constants: ";
			print += pdiff ? "TURRETENCODERP " + TURRETENCODERP + " ": "";
			print += idiff ? "TURRETENCODERI " + TURRETENCODERI + " ": "";
			print += ddiff ? "TURRETENCODERD " + TURRETENCODERD + " ": "";
			print += deaddiff ? "TURRETENCODERDEAD " + TURRETENCODERDEAD + " ": "";
			print += outdiff ? "TURRETOUTPUTRANGE " + TURRETOUTPUTRANGE: "";
			System.out.println(print);
			
			turretencodercontrol.setPID(TURRETENCODERP, TURRETENCODERI, TURRETENCODERD);
			turretencodercontrol.setAbsoluteTolerance(TURRETENCODERDEAD);
			turretencodercontrol.setOutputRange(-TURRETOUTPUTRANGE, TURRETOUTPUTRANGE);
		}
		
		boolean donutdiff = prevturretdonut != TURRETDONUT, clampdiff = prevturretclamp != TURRETCLAMP;
		
		if(donutdiff || clampdiff)
		{
			String print = "Different constants: ";
			print += donutdiff ? "TURRETDONUT " + TURRETDONUT + " ": "";
			print += clampdiff ? "TURRETCLAMP " + TURRETCLAMP: "";
			System.out.println(print);
			
			turret.setDonutThreshold(TURRETDONUT);
			turret.setClamp(TURRETCLAMP);
		}
		
		BangBang tempBB = new BangBang(new double[]{-RESETSPEED, RESETSPEED});
		
		if(!tempBB.equals(resetBang))
		{
			resetBang = tempBB;
		}
		
		DIOHomer temp = new DIOHomer(halleffect, HOMESEEKSPEED, HOMESLOWSPEED, HOMINGDIR, HOMEHIGH != 0, (long)HOMETIMEOUT);
		
		if(!temp.equals(turretHomer))
		{
			turretHomer = temp;
		}
		
		if(prevvisionavg != VISIONAVERAGEQTY)
		{
			String print = "Different constants: ";
			print += prevvisionavg != VISIONAVERAGEQTY ? "VISIONAVERAGEQTY " + VISIONAVERAGEQTY + " ": "";
			System.out.println(print);
			visionavg.setNumValues((int)VISIONAVERAGEQTY);
		}
		
	}

	@Override
	public synchronized void update()//Flywheel and turret control loop
	{
		String timereport = "";
		long startTime = System.nanoTime();
		double rectifyangle = aimangle;
		boolean setpointDiff = false;
		double enc = encoder.getDistance();
		boolean visiondone = false;
		if(resettingSoft) aimmode = AimMode.ENCODER;
		//======================
		//========Turret========
		//======================
		
		//Initializes the turretHomer when ready
		if(turretHomer != null && turretHomer.isStopped())
		{
			if(turretManual.isTimeUp())
			{
				//THIS IS NO LONGER NECCESSARY BECAUSE OF RECTIFYANGLE()
				//checks whether or not the encoder control is being bad, does a 360 noscope if so
				/*if(turretencodercontrol.isEnabled())
				{
					if(!softAcceptable(turretencodercontrol.get()))
					{
						aim(encoder.getDistance() - softDir*SOFTRESETANGLE);
						resettingSoft = true;
						softFlag = true;
					}
				}*/
				
				if(!turretdone)
				{
					boolean searchdone = searching && vision.targetAvailable();
					switch(aimmode)
					{
						case VISION:
							//If first exec, make sure we're using the right control
							/*if(lastturretdone)
							{
								turretencodercontrol.disable();
								turretvisioncontrol.enable();
							}*/
							
							if(vision.targetAvailable())
							{
								double visionval = vision.pidGet();
								
								if(vision.checkFrameDouble())//new frame? checkFrame resets the flag
								{
									switch(visionMode)
									{
										case AVERAGE://average a certain number of values, then...
											if(visionavg.isFull())//...when full...
											{
												visionMode = VisionMode.GO;//...go
											}
											else
											{
												visionavg.addValue(visionval);
												break;
											}
											
										case GO://move to the averaged value
											System.out.println("GO");
											double avgval = visionavg.getAverage();
											visionavg.reset();
											System.out.println("avgval: " + avgval);
											//recalc angle, new vision value (vision error may or may not be in tolerance)
											System.out.println("Vision: " + visionval + " error: " + turretencodercontrol.getError());
											if(avgval != lastsetvision)//if this value has not been previously set, set it
											{
												if(Math.abs(avgval + turretencodercontrol.getError()) < TURRETENCODERDEAD*1.5)
												{
													System.out.println("VISIONDONE");
													visiondone = true;
												}
												else
												{
													rectifyangle = enc + rectifyAngle(avgval + enc, enc);
													lastsetvision = avgval;
													lastaimangle = aimangle;//hack to make the encoder check not recheck the angle
													setpointDiff = true;
												}
											}
											visionMode = VisionMode.DISCARD;//next...
											visionthrowawaycount = 0;
											break;
											
										case DISCARD://discard a couple frames so that the move can complete
											visionthrowawaycount++;
											if(visionthrowawaycount >= VISIONTHROWAWAY)//if we have thrown away enough frames...
											{
												visionMode = VisionMode.AVERAGE;//...start averaging again
											}
											break;
									}
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
							
							timereport += "visionchk: " + (System.nanoTime() - startTime) + "\n";
							
						case ENCODER:
							//If first exec, make sure we're using the right control
							//System.out.println("Lastturretdone: " + lastturretdone);
							if(!searchdone)
							{
								conditionalEnable();
							}
							//System.out.println("Enable: " + turretencodercontrol.isEnabled() + ", Mode: " + aimmode);
							
							String print = "";
							
							//Rectify the angle - finds softlimit problems - sets resettingSoft and softFlag
							if(lastaimangle != aimangle)
							{
								rectifyangle = enc + rectifyAngle(aimangle, enc);
								setpointDiff = true;
							}
							
							timereport += "rectifychk: " + (System.nanoTime() - startTime) + "\n";
							
							//blocks the setting of the setpoint if we are resetting due to a soft limit failure
							//softFlag allows the setpoint to be set once - for the reset, and no more
							if(!resettingSoft || softFlag)
							{
								if(softFlag)
								{
									print += "360 noscope: " + rectifyangle + "\n";
									turretencodercontrol.reset();//disables as well
									resetBang.setSetpoint(rectifyangle);
									turret.setClamp(1);
									turretencodercontrol.setSetpoint(rectifyangle);
									softFlag = false;
								}
								else//!resettingSoft && !softFlag
								{
									//setSetpoint resets the average error buffer - then onTarget doesn't work
									if(setpointDiff)
									{
										//print += "Setting setpoint to " + rectifyangle + "\nResetting error: " + turretencodercontrol.getAvgError() + "\n";
										turretencodercontrol.setSetpoint(rectifyangle);
									}
								}
							}
							if(resettingSoft)
							{
								turret.set(resetBang.output(enc));
								//System.out.println("RESETDEAD: " + RESETDEAD + ", Error: " + resetBang.getError(enc));
								if(Math.abs(resetBang.getError(enc)) < RESETDEAD)
								{
									turretencodercontrol.enable();
									resettingSoft = false;
								}
							}
							
							timereport += "softchk: " + (System.nanoTime() - startTime) + "\n";
							
							if(!print.equals(""))
								System.out.print(print);
							
							if(searching) System.out.println("Searchdone: " + searchdone);
							if((turretencodercontrol.onTarget() && Math.abs(turretencodercontrol.getError()) < TURRETENCODERDEAD) ||
									searchdone || visiondone)
							{
								//System.out.println("onTarget: " + turretencodercontrol.getError());
//								System.out.println("onTarget error: " + turretencodercontrol.getAvgError() +
//										", error: " + turretencodercontrol.getError());
								turret.setClamp(TURRETCLAMP);
								if(aimmode == AimMode.ENCODER) turretdone = true;

								if(aimmode == AimMode.VISION && visiondone)
								{
									turretdone = true;
									System.out.println("VISION DONE");
								}
								visiondone = false;
								resettingSoft = false;
								softFlag = false;
								searching = false;
								turretencodercontrol.disable();
							}
							timereport += "targetchk: " + (System.nanoTime() - startTime) + "\n";
							break;
					}
					turretautodone = turretdone;
				}
				else
				{
					conditionalDisable();
				}
			}
			else
			{
				conditionalDisable();
			}
			
			if(USESOFT != 0)//use soft limits
			{
				if(enc >= SOFTLIMITS)//softlimits passed
				{
					softlimitsPassed = true;
					softDir = 1;//signifies the direction that softlimits have been passed
				}
				else if(enc <= -SOFTLIMITS)
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
			conditionalDisable();
			
			if(turretHomer != null)
			{
				turretHomer.update();
				if(turretHomer.isHome())
				{
					encoder.reset();
					turret.set(turretHomer.getSpeed());
					System.out.println("home: " + encoder.get() + ", " + encoder.getDistance());
					if(encoder.get() > 1500)System.out.println("wtf encoder");
					aim(0);
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
			timereport += "homechk: " + (System.nanoTime() - startTime) + "\n";
		}
		
		lastaimangle = aimangle;
		lastturretdone = turretdone;
		long endTime = (System.nanoTime() - startTime);
		if(endTime > 4*1000000 && false)
		{
			timereport += "endchk: " + endTime;
			System.out.println(timereport);
		}
	}
	
	private void conditionalDisable()
	{
		if(turretencodercontrol.isEnabled())
		{
			turretencodercontrol.disable();
		}
	}
	
	private void conditionalEnable()
	{
		if(!turretencodercontrol.isEnabled())
		{
			turretencodercontrol.enable();
		}
	}
	
	public synchronized boolean isSearching()
	{
		return searching;
	}
	
	public synchronized double getTurretSet()
	{
		return turretencodercontrol.getSetpoint();
	}
	
	public PIDDashdataWrapper getVisionWrapper()
	{
		return vision;
	}
	
	public void printPID()
	{
		String print = "Different constants: ";
		print += "TURRETENCODERP " + TURRETENCODERP + " ";
		print += "TURRETENCODERI " + TURRETENCODERI + " ";
		print += "TURRETENCODERD " + TURRETENCODERD + " ";
		print += "TURRETENCODERDEAD " + TURRETENCODERDEAD;
		System.out.println(print);
	}
	
	public PIDCANTalonEncoderWrapper getTurretEncoder()
	{
		return encoder;
	}
	
	/**
	 * Returns whether this move will violate soft limits based on the state of the shooter
	 * @param rotate the value to check
	 * @return true if acceptable
	 */
	private boolean softAcceptable(double rotate)
	{
		return !softlimitsPassed || (softDir * rotate) < 0;
	}
	
	public String getVisionVals()
	{
		return "Visionval: " + vision.pidGet() + ", Error: " + turretencodercontrol.getError();
	}
	
	public synchronized void search(double angle)
	{
		searching = true;
		aim(encoder.getDistance() + angle);
	}
	
	/**
	 * Aims the turret at the largest vision target.
	 */
	public synchronized void aim()
	{
		turretdone = false;
		aimmode = AimMode.VISION;
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
	}
	
	/**
	 * Returns an encoder difference that should be executed.
	 * @param angle the absolute angle to rectify
	 * @return the difference to be moved
	 */
	private double rectifyAngle(double angle, double enc)
	{
		int cableDir = enc < 0 ? -1 : 1;
		double OVERTRAVEL = Math.max(SOFTLIMITS - 0.5, 0);
		double TRAVEL = SOFTLIMITS - OVERTRAVEL; 
		
		double limitLeft = -TRAVEL, limitRight = TRAVEL;
		
		if(cableDir == -1)//wrapped left
		{
			//shift limits left
			limitLeft -= OVERTRAVEL;
			limitRight -= OVERTRAVEL;
		}
		else if(cableDir == 1)//wrapped right
		{
			//shift limits right
			limitRight += OVERTRAVEL;
			limitLeft += OVERTRAVEL;
		}
		//cabledir cannot be 0, if enc == 0, the limits don't matter anyway
		
		angle = convertAngleLocal(angle, limitLeft, limitRight);
		double localEnc = convertAngleLocal(enc, limitLeft, limitRight);
		//System.out.println("Local angle: " + angle*360 + ", Local encoder: " + localEnc*360);
		
		double counterclockwise = 0, clockwise = 0;
		
		//GET RELEVANT TRAVEL DISTANCES
		//(setpoint)s is angle
		//(process)p is localEnc
		if(angle < localEnc)//s < p
		{
			counterclockwise = angle - localEnc;//ccw = s - p (-)
			clockwise = 1 + counterclockwise;//cw = 1 + ccw (+)
		}
		else if(angle > localEnc)//s > p
		{
			clockwise = angle - localEnc;//cw = s - p (+)
			counterclockwise = clockwise - 1;//ccw = cw - 1 (-)
		}
		
		//SOFTLIMIT CHECK
		if(Math.abs(counterclockwise) > Math.abs(clockwise)) //cw most efficient
		{
			if(enc + clockwise <= limitRight)//cw legal
			{
				return clockwise;
			}
			else if(enc + counterclockwise >= limitLeft) //cw illegal, ccw legal
			{
				resettingSoft = true;
				softFlag = true;
				return counterclockwise;
			}
			else //both illegal
			{
				throw new IllegalStateException("Rectify angle cw both illegal");
			}
		}
		else //ccw most efficient
		{
			if(enc + counterclockwise >= limitLeft) //ccw legal
			{
				return counterclockwise;
			}
			else if(enc + clockwise <= limitRight) //ccw illegal, cw legal
			{
				resettingSoft = true;
				softFlag = true;
				return clockwise;
			}
			else //both illegal
			{
				throw new IllegalStateException("Rectify angle ccw both illegal");
			}
		}
	}
	
	/**
	 * 
	 * @param angle
	 * @param leftLimit
	 * @param rightLimit
	 * @return
	 */
	private double convertAngleLocal(double angle, double leftLimit, double rightLimit)
	{
		if(angle < 0)//leftLimit
		{
			while(angle < leftLimit)
			{
				angle += 1;//360 degrees - essentially % 360
			}
		}
		else//rightLimit
		{
			while(angle > rightLimit)
			{
				angle -= 1;//360 degrees - essentially % 360
			}
		}
		return angle;
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
	 * Stops the aiming for the turret.
	 * In the case of encoder aiming, stops prematurely.
	 * In the case of vision aiming, stops normally.
	 */
	public synchronized void stopAim()
	{
		turretdone = true;
	}
	
	public boolean getSoftLimits()
	{
		return softlimitsPassed;
	}
	
	public int getSoftDir()
	{
		return softDir;
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
			turretThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}
	
	public synchronized void end()
	{
		turretdone = true;
		resettingSoft = false;
		softFlag = false;
		turretencodercontrol.disable();
	}
	
	@Override
	public boolean threadsActive()
	{
		return task.isActive();
	}
	
	public String toString()
	{
		return "Turret";
	}

}
