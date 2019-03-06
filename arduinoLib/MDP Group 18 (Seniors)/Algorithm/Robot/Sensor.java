package Robot;

import java.awt.Point;
import Robot.RobotConstants.Direction;
import Map.*;

/**
 * @author Saklani Pankaj
 *
 */

public class Sensor {
	
	//Ranges of the Sensors
	private String id;
	private int minRange;
    private int maxRange;
    private double prevData;
    private double prevRawData;

    // Sensor's position on the map
    private Point pos;

    private Direction sensorDir;

    public Sensor(String id, int minRange, int maxRange, int sensorPosRow, int sensorPosCol, Direction sensorDirection) {
        this.id = id;
    	this.minRange = minRange;
        this.maxRange = maxRange;
        this.pos = new Point(sensorPosCol, sensorPosRow);
        this.sensorDir = sensorDirection;
        this.prevData = 9;
        this.prevRawData = 99;
    }

	public String getId() {
		return id;
	}

	public void setId(String id) {
		this.id = id;
	}

	public int getMinRange() {
		return minRange;
	}

	public void setMinRange(int minRange) {
		this.minRange = minRange;
	}

	public int getMaxRange() {
		return maxRange;
	}

	public void setMaxRange(int maxRange) {
		this.maxRange = maxRange;
	}

	public Point getPos() {
		return pos;
	}
	
	public int getRow() {
		return pos.y;
	}
	
	public int getCol() {
		return pos.x;
	}

	public void setPos(int col, int row) {
		this.pos.setLocation(col, row);
	}

	public Direction getSensorDir() {
		return sensorDir;
	}

	public void setSensorDir(Direction sensorDir) {
		this.sensorDir = sensorDir;
	}
    
	//Detect method for simulator
	//Senses where the obstacle is of the sensor if none detected return -1
	//Detected within the min and max range of sensor
	public int detect(Map map) {
		// Checking the range
		for (int cur = minRange; cur <= maxRange; cur++) {
			switch (sensorDir) {
			case UP:
				if (pos.y + cur > MapConstants.MAP_HEIGHT - 1)
					return -1;
				else if (map.getCell(pos.y + cur, pos.x).isObstacle())
					return cur;
				break;
			case RIGHT:
				if (pos.x + cur > MapConstants.MAP_WIDTH - 1)
					return -1;
				else if (map.getCell(pos.y, pos.x + cur).isObstacle())
					return cur;
				break;
			case DOWN:
				if (pos.y - cur < 0)
					return -1;
				else if (map.getCell(pos.y - cur, pos.x).isObstacle())
					return cur;
				break;
			case LEFT:
				if (pos.x - cur < 0)
					return -1;
				else if (map.getCell(pos.y, pos.x - cur).isObstacle())
					return cur;
				break;
			}
		}
		return -1;
	}

	public double getPrevData() {
		return prevData;
	}

	public void setPrevData(double prevData) {
		this.prevData = prevData;
	}

	public double getPrevRawData() {
		return prevRawData;
	}

	public void setPrevRawData(double prevRawData) {
		this.prevRawData = prevRawData;
	}
}
