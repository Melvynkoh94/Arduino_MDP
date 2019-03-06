package Algorithm;

import java.awt.Point;

import Robot.*;
import Robot.RobotConstants.Direction;
import Robot.RobotConstants.Command;
import Map.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

/*
 * Basic Concept of A* is f(u) = g(u) + h(u), where we choose the lowest f(u)
 * g(u) is the cost of moving from the START CELL to current cell
 * h(u) is the distance of new node to the goal
 * g(u) is calculated based on the direction of the robot, turning incurs a greater cost compared to going straight
 * 
*/
/**
 * @author Saklani Pankaj
 *
 */
public class FastestPath {
	
	private Robot robot;
	private Map exploredMap;
	private HashMap <Cell, Cell> prevCell;	//Used to identify predecessor cell
	private ArrayList<Cell> toVisit;		//Store cells to be visited
	private ArrayList<Cell> visited;		//Store visited cells
	private HashMap<Point, Double> costG;
	private ArrayList<Cell> path;
	private boolean sim = true;
	
	
	public FastestPath(Map exploredMap, Robot robot, boolean sim) {
		this.exploredMap = exploredMap;
		this.robot = robot;
		this.sim = sim;
		prevCell = new HashMap<Cell, Cell>();
		
		costG = new HashMap<Point, Double>();

		//Init costG array for explored cells
		for(int row=0; row< MapConstants.MAP_HEIGHT; row++) {
			for(int col=0; col< MapConstants.MAP_WIDTH; col++) {		
				//if the cell is obstacle/ unexplored/ unexplored CostG is high
				if(!exploredMap.getCell(row, col).movableCell())
					costG.put(new Point(col,row), RobotConstants.INFINITE_COST);
				
				//Else set costG for the cell to 0 first
				else
					costG.put(new Point(col,row), 0.0);
			}
		}
	}
	
	//Getting traversal cost from cellA to cellB
	private double getCostG(Point cellA, Point cellB, Direction dir) {
		double moveCost = RobotConstants.MOVE_COST;
		
		double turnCost = getTurnCost(dir, getCellDirection(cellA, cellB));
		
		return moveCost + turnCost;
	}
	
	private double getCostH(Point cell, Point goal) {
		return cell.distance(goal);
	}
	
	
	//Get the direction of cellB from cellA
	private Direction getCellDirection(Point cellA, Point cellB){
		if(cellA.y - cellB.y > 0)
			return Direction.DOWN;
		else if(cellA.y - cellB.y < 0)
			return Direction.UP;
		else if(cellA.x - cellB.x > 0)
			return Direction.LEFT;
		else
			return Direction.RIGHT;
	}
	
	private double getTurnCost(Direction a, Direction b)
	{
		//Max of 2 turns in either direction same direction will get 0
		int turns = Math.abs(a.ordinal() - b.ordinal());
		if(turns>2)
			turns%=2;
		
		return turns * RobotConstants.TURN_COST;
	}
	
	
	//Get the min cost in toVisit ArrayList
	public Cell minCostCell(Point goal) {
		double min = 10000;
		double cost;
		Point p;
		Cell cell = null;
		
		for(int i=0; i< toVisit.size(); i++)
		{
			p = toVisit.get(i).getPos();
			cost = costG.get(p) + getCostH(p, goal);
			
			if(cost< min) {
				min = cost;
				cell = toVisit.get(i);
			}
		}
		return cell;
	}
	
	//Actual A* Algo running
	public ArrayList<Cell> run(Point start, Point goal, Direction dir) {
		
		System.out.println("Starting Fastest Run");
		//Temp Holder gor gCost
		double gCost;
		
		path = new ArrayList<Cell>();
		toVisit = new ArrayList<Cell>();
		visited = new ArrayList<Cell>();
		Cell cur = exploredMap.getCell(start);
		toVisit.add(cur);
		ArrayList<Cell> neighbours;
		
		//Set curDir to starting dir
		Direction curDir = dir;
		
		//Loop through all the items in toVisit
		while(!toVisit.isEmpty()) {
			cur = minCostCell(goal);
			if(prevCell.containsKey(cur))
				curDir = getCellDirection(prevCell.get(cur).getPos(),cur.getPos());
			
			visited.add(cur);
			toVisit.remove(cur);
			
			//Check If Goal Reached
			if(visited.contains(exploredMap.getCell(goal))) {
				path = getPath(start,goal);
				return path;
			}
			
			neighbours = exploredMap.getNeighbours(cur);
			for(int i=0; i<neighbours.size(); i++) {
				
				//if cell has been visited skip
				if(visited.contains(neighbours.get(i)))
					continue;
				
				gCost = costG.get(cur.getPos())+ getCostG(cur.getPos(),neighbours.get(i).getPos(),curDir);
				
				//if the cell is not in toVisit
				if(!toVisit.contains(neighbours.get(i))) {
					prevCell.put(neighbours.get(i), cur);
					
					costG.put(neighbours.get(i).getPos(), gCost);
					toVisit.add(neighbours.get(i));
				}
				else {
					double curGCost = costG.get(neighbours.get(i).getPos());
					//If path from current is shorter update costG of the neighbour cell;
					if(gCost < curGCost) {
						costG.replace(neighbours.get(i).getPos(), gCost);
						prevCell.replace(neighbours.get(i), cur);
					}
				}
			}
		}
		
		// return Null if cannot find a path
		System.out.println("Returning NULL");
		return null;
		
	}
	
	//returns the path from the prevCell hashmap, moving backwards from goal to start
	public ArrayList<Cell> getPath(Point start, Point goal) {
		Cell cur = exploredMap.getCell(goal);
		Cell startCell = exploredMap.getCell(start);
		ArrayList<Cell> path = new ArrayList<Cell>();
		while(cur != startCell) {
			path.add(cur);
			cur = prevCell.get(cur);
		}
		Collections.reverse(path);
		return path;	
	}
	
	//Method to display path on Sim Map
	public void displayFastestPath(ArrayList<Cell> path, boolean display) {
		System.out.println("The number of steps is: " + (path.size() - 1) + "\n");

		Cell temp;
		System.out.println("Path:");
		for(int i=0; i<path.size(); i++) {
			temp = path.get(i);
			//Set the path cells to display as path on the Sim
			exploredMap.getCell(temp.getPos()).setPath(display);
			System.out.println(exploredMap.getCell(temp.getPos()).toString());
			
			//Output Path on console
			if(i != (path.size()-1))
				System.out.print("(" + temp.getPos().y + ", " + temp.getPos().x + ") --> ");
			else
				System.out.print("(" + temp.getPos().y + ", " + temp.getPos().x + ")");
		}
		System.out.println("\n");
	}
	
	//Returns the movements required to execute the path
	public ArrayList<Command> getPathCommands(ArrayList<Cell> path) {
		Robot tempRobot = new Robot(true, robot.getDirection(), robot.getPosition().y,robot.getPosition().x);
		tempRobot.setFastSense(true);
		ArrayList<Command> moves = new ArrayList<Command>();
		
		Command move;
		Cell cell = exploredMap.getCell(tempRobot.getPosition());
		Cell newCell;
		Direction cellDir;
		
//		int calibrateCount = 0; //Calibrate after 3 moves
		
		
		//Iterate through the path
		for(int i=0; i< path.size(); i++) {
			move = Command.ERROR;
			newCell = path.get(i);
			cellDir = getCellDirection(cell.getPos(),newCell.getPos());
			
			if(!sim) {
//				if(calibratePoint(cell, tempRobot.getDirection())/*||calibrateCount == RobotConstants.CALIBRATE_AFTER*/)
//				{
////					calibrateCount = 0;
//					//Found a Calibration Point
//					moves.add(Command.ALIGN_RIGHT);
//					moves.add(Command.ALIGN_FRONT);
//				}
//				else
//					calibrateCount++;
			}
			
//			while(tempRobot.getDirection()!=cellDir) {
//				move = getTurnMovement(tempRobot.getDirection(), cellDir);
//				tempRobot.move(move, RobotConstants.MOVE_STEPS, exploredMap);
//				moves.add(move);
//			}
			//If the TempRobot and cell direction not the same
			if(tempRobot.getDirection()!=cellDir) {
				if(Direction.reverse(tempRobot.getDirection()) == cellDir) {
					move = Command.TURN_LEFT;
					tempRobot.move(move, RobotConstants.MOVE_STEPS, exploredMap);
					moves.add(move);
					tempRobot.move(move, RobotConstants.MOVE_STEPS, exploredMap);
					moves.add(move);
//					cell = newCell;
//					continue;
				}
				else {
					move = getTurnMovement(tempRobot.getDirection(), cellDir);
					tempRobot.move(move, RobotConstants.MOVE_STEPS, exploredMap);
					moves.add(move);
					
				}
			}
			//Keep Moving Forward in the Path
			move = Command.FORWARD;
			tempRobot.move(move, RobotConstants.MOVE_STEPS, exploredMap);
			moves.add(move);
			cell = newCell;
		}
		//System.out.println("Generated Moves: "+moves.toString());
		return moves;
	}
	
	//Determine if a cell is a calibration point (all front and right cells are virtual walls) 
	public boolean calibratePoint(Cell cur, Direction robotDir) {
		Cell front1 = cur, front2= cur, front3 = cur, right1 = cur, right2 = cur;
		switch(robotDir) {
		case UP:
			front1 = exploredMap.getCell(cur.getPos().y+1, cur.getPos().x-1);
			front2 = exploredMap.getCell(cur.getPos().y+1, cur.getPos().x);
			front3 = exploredMap.getCell(cur.getPos().y+1, cur.getPos().x+1);
			right1 = exploredMap.getCell(cur.getPos().y+1, cur.getPos().x+1);
			right2 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x+1);
			break;
		case DOWN:
			front1 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x-1);
			front2 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x);
			front3 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x+1);
			right1 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x-1);
			right2 = exploredMap.getCell(cur.getPos().y+1, cur.getPos().x-1);
			break;
		case RIGHT:
			front1 = exploredMap.getCell(cur.getPos().y+1, cur.getPos().x+1);
			front2 = exploredMap.getCell(cur.getPos().y, cur.getPos().x+1);
			front3 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x+1);
			right1 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x+1);
			right2 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x-1);
			break;
		case LEFT:
			front1 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x-1);
			front2 = exploredMap.getCell(cur.getPos().y, cur.getPos().x-1);
			front3 = exploredMap.getCell(cur.getPos().y+1, cur.getPos().x-1);
			right1 = exploredMap.getCell(cur.getPos().y-1, cur.getPos().x-1);
			right2 = exploredMap.getCell(cur.getPos().y+1, cur.getPos().x+1);
			break;
		}
		if(front1.isVirtualWall()&&front2.isVirtualWall()&&front3.isVirtualWall()&&right1.isVirtualWall()&&right2.isVirtualWall())
			return true;
		
		return false;
	}
	
	
	//Returns which side to turn to face the cell based on the robot's direction
	public Command getTurnMovement(Direction botDir, Direction cellDir) {
		Command move;
		
		switch(botDir) {
		case UP:
			if(cellDir == Direction.LEFT)
				move = Command.TURN_LEFT;
			else
				move = Command.TURN_RIGHT;
			break;
			
		case LEFT:
			if(cellDir == Direction.UP)
				move = Command.TURN_RIGHT;
			else
				move = Command.TURN_LEFT;
			break;
			
		case RIGHT:
			if(cellDir == Direction.UP)
				move = Command.TURN_LEFT;
			else
				move = Command.TURN_RIGHT;
			break;
			
		case DOWN:
			if(cellDir == Direction.LEFT)
				move = Command.TURN_RIGHT;
			else
				move = Command.TURN_LEFT;
			break;
			
		default:
			move = Command.ERROR;
			break;
		}
		
		return move;
		
	}

}
