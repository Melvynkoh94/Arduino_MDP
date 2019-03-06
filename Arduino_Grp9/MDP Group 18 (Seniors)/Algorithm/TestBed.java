import java.util.Scanner;

import Network.*;
import Robot.RobotConstants.Command;

public class TestBed {
	private static NetMgr net;

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		System.out.println(Command.FORWARD.ordinal());
		char[] moves = {'W','A','D','S'};
		System.out.println(moves[Command.FORWARD.ordinal()]);
			
			
			
//		net.send("Alg|Ard|0|1\n");
		//Keep trying to connect if fail to connec
		
	}
}