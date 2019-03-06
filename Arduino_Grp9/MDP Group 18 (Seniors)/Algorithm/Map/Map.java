package Map;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import Robot.RobotConstants.Direction;

/**
 * @author Saklani Pankaj
 *
 */

public class Map {

	private final Cell[][] grid;
	private ArrayList<Point> detectedImg;
	private HashMap<Point,Direction> imgDir;
	
	// KIV add Robot once Created
	public Map() {
		grid = new Cell[MapConstants.MAP_HEIGHT][MapConstants.MAP_WIDTH];
		detectedImg = new ArrayList<Point>();
		imgDir = new HashMap<Point,Direction>();
		initMap();
	}
	
	
	private void initMap() {
		// Init Cells on the grid;
		for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
			for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
				grid[row][col] = new Cell(new Point(col, row));

				// Init Virtual wall
				if (row == 0 || col == 0 || row == MapConstants.MAP_HEIGHT - 1 || col == MapConstants.MAP_WIDTH - 1) {
					grid[row][col].setVirtualWall(true);
				}

			}
		}
	}
	
	//Add the Detected Image to Map's image collections to track image location and direction
	public boolean detectedImg(Point pos, Direction dir) {
		if(checkValidCell(pos.y, pos.x) && grid[pos.y][pos.x].isObstacle() && detectedImg.indexOf(pos) == -1) {
			detectedImg.add(pos);
			imgDir.put(pos, dir);
			return true;
		}
		return false;
	}
	
	//Check if image has been detected before
	public boolean isImgDetected(Point pos, Direction dir) {
		return imgDir.containsKey(pos);
	}
	
	//to String Image location and direction
	public String detectedImgToString() {
		if(detectedImg.isEmpty())
			return null;
		else
		{
			String detected = "";
			for(Point pos: detectedImg) {
				detected += "(x="+pos.x+" y="+pos.y+" "+imgDir.get(pos).name().charAt(0)+"),";
			}
			return detected;
		}
	}
	
	//Set all cells as explored based on boolean true is all explord false is all unexplored
	public void setAllExplored(boolean explored) {
		for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
			for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
				grid[row][col].setExplored(explored);
			}
		}
	}
	
	//Set all cells as explored based on boolean true is all explord false is all unexplored
	public void setAllMoveThru(boolean moveThru) {
		for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
			for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
				grid[row][col].setMoveThru(moveThru);
			}
		}
	}

	// Returns the Cell
	public Cell getCell(int row, int col) {
		return grid[row][col];
	}

	// Returns the Cell based on Point
	public Cell getCell(Point pos) {
		return grid[pos.y][pos.x];
	}
	
	// Returns the nearest unexplored cell to the loc
	public Cell nearestUnexp(Point loc) {
		Cell cell, nearest = null;
		double distance = 1000;
		
		//Check for nearest unexplored 
		for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
			for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
				cell = grid[row][col];
				if(!cell.isExplored() && distance > loc.distance(cell.getPos()))
				{
					nearest = cell;
					distance = loc.distance(cell.getPos());
				}
			}
		}
		return nearest;
	}
	
	//Returns the nearest explored cell to the loc
	public Cell nearestExp(Point loc, Point botLoc) {
		Cell cell, nearest = null;
		double distance = 1000;
		
		//Check for nearest unexplored 
		for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
			for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
				cell = grid[row][col];
				if(checkValidMove(row,col) && clearForRobot(row,col) && areaMoveThru(row,col))
//				if(checkValidMove(row,col) && clearForRobot(row,col) && moveThru(row,col))
				{
					if((distance > loc.distance(cell.getPos())&& cell.getPos().distance(botLoc)>0)){
						nearest = cell;
						distance = loc.distance(cell.getPos());
					}
				}
			}
		}
		return nearest;
	}
	
	//Check if the entire area is moveThru
	public boolean areaMoveThru(int row, int col) {
		for(int r=row-1; r<= row+1; r++) {
			for(int c=col-1; c<=col+1; c++) {
				if(!grid[r][c].isMoveThru())
					return true;
			}
		}
		return false;
	}
	
	//Make sure the robot can move to the row, and col
	public boolean clearForRobot(int row, int col) {
		for(int r=row-1; r<= row+1; r++) {
			for(int c=col-1; c<=col+1; c++) {
				if(!checkValidCell(r,c)||!grid[r][c].isExplored()||grid[r][c].isObstacle())
					return false;
			}
		}
		return true;
	}
	
	//See if cell has unmoved place that is not completely moved thru
	public boolean moveThru(int row, int col) {
		for(int r=row-1; r<= row+1; r++) {
			for(int c=col-1; c<=col+1; c++) {
				if(!grid[r][c].isMoveThru())
					return true;
			}
		}
		return false;
	}

	// Check if the row and col is within the map boundary
	public boolean checkValidCell(int row, int col) {
		return row >= 0 && col >= 0 && row < MapConstants.MAP_HEIGHT && col < MapConstants.MAP_WIDTH;
	}

	// Check if valid to move there cannot move to virtual wall
	public boolean checkValidMove(int row, int col) {
		return checkValidCell(row, col) && !getCell(row, col).isVirtualWall() && !getCell(row, col).isObstacle() && getCell(row,col).isExplored();
	}
	
	// Check if valid to move there cannot move to virtual wall
	public boolean wayPointClear(int row, int col) {
		return checkValidCell(row, col) && !getCell(row, col).isVirtualWall() && !getCell(row, col).isObstacle();
	}

	// Reset Map
	public void resetMap() {
		initMap();
	}
	
	//Set the area under the robot as passed thru
	public void passThru(int row,int col) {
		for(int r= row-1; r<=row+1; r++) {
			for(int c= col-1; c<=col+1; c++) {
				grid[r][c].setMoveThru(true);
			}
		}
	}
	
	// Reinit virtual walls around obstacle
	public void reinitVirtualWall() {
		for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
			for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
				// Init Virtual wall
				if (row == 0 || col == 0 || row == MapConstants.MAP_HEIGHT - 1 || col == MapConstants.MAP_WIDTH - 1) {
					grid[row][col].setVirtualWall(true);
				}
				if (grid[row][col].isObstacle()) {
					for (int r = row - 1; r <= row + 1; r++)
						for (int c = col - 1; c <= col + 1; c++)
							if (checkValidCell(r, c))
								grid[row][col].setVirtualWall(true);
				}
			}
		}
	}

	// Returns the Percentage Explored
	public double exploredPercentage() {
		double total = MapConstants.MAP_HEIGHT * MapConstants.MAP_WIDTH;
		double explored = 0;

		for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
			for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
				if (grid[row][col].isExplored())
					explored++;
			}
		}
		return explored / total * 100;
	}

	public ArrayList<Cell> getNeighbours(Cell c) {
		ArrayList<Cell> neighbours = new ArrayList<Cell>();

		// UP
		if (checkValidMove(c.getPos().y + 1, c.getPos().x)) {
			neighbours.add(getCell(c.getPos().y + 1, c.getPos().x));
		}
		// DOWN
		if (checkValidMove( c.getPos().y - 1, c.getPos().x)) {
			neighbours.add(getCell(c.getPos().y - 1, c.getPos().x));
		}

		// RIGHT
		if (checkValidMove(c.getPos().y, c.getPos().x + 1)) {
			neighbours.add(getCell(c.getPos().y, c.getPos().x + 1));
		}

		// LEFT
		if (checkValidMove( c.getPos().y, c.getPos().x - 1)) {
			neighbours.add(getCell(c.getPos().y, c.getPos().x - 1));
		}

		return neighbours;
	}
	
	//Remove existing cells with path
	public void removePaths() {
		for(int r=0; r<MapConstants.MAP_HEIGHT; r++) {
			for(int c=0; c<MapConstants.MAP_WIDTH; c++) {
				grid[r][c].setPath(false);
			}
		}
	}
}
