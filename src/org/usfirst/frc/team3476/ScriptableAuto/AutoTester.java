package org.usfirst.frc.team3476.ScriptableAuto;

import org.usfirst.frc.team3476.Subsystems.*;
import org.usfirst.frc.team3476.Main.Subsystem;

public class AutoTester
{
	public static void main(String[] args)
	{
		String script = "test: 47@48 \ntest";
		
		String constants = "2016~test1 = 3.0\ntest2=4.0\u001B2015~test1 = 1.0\ntest2=2.0";
		
		Subsystem[] systems = new Subsystem[]{new Test()};
		
		Main main = new Main("2016", systems, script, constants);
		
		main.start();
	}
}
