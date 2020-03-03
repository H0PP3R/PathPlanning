import java.util.*;
import java.lang.Math.*;
import static java.util.stream.Collectors.*;
import static java.util.Map.Entry.*;
PrintWriter output;
//------------------------------------------------------------------------------
final int MAX_X = 20; 
final int MAX_Y = 20;
final int PIXELS_PER_CELL = 25; 
int world[][] = new int[MAX_Y][MAX_X];
final int CELL_EMPTY    = 0;  // an empty cell
final int CELL_OBSTACLE = -1; // a cell with an obstacle in it
int NUM_OBSTACLES = int(( MAX_X * MAX_Y ) * 0.25 );
//------------------------------------------------------------------------------
PVector R = new PVector( 0, 0 ); 
final PVector NORTH = new PVector( 0, -1 );
final PVector WEST  = new PVector( -1, 0 );
final PVector SOUTH = new PVector( 0, +1 );
final PVector EAST  = new PVector( +1, 0 );
final PVector NEAST = new PVector( +1, -1);
final PVector SEAST = new PVector( +1, +1);
final PVector NWEST = new PVector( -1, -1);
final PVector SWEST = new PVector( -1, +1);
// The robot has 4 range sensors which detect things relative to the robot in 
// each compass direction.

// Collection of all the ways a robot can move
PVector directions[] = {NORTH, NWEST, WEST, SWEST, SOUTH, SEAST, EAST, NEAST};
/**
 *set directions as a treemap so I would know how to orient the robot
 */
TreeMap<Integer, PVector> directions2 = new TreeMap();
void setDirections2(){
  directions2.put(0,NORTH);
  directions2.put(1,NWEST);
  directions2.put(2,WEST);
  directions2.put(3,SWEST);
  directions2.put(4,SOUTH);
  directions2.put(5,SEAST);
  directions2.put(6,EAST);
  directions2.put(7,NEAST);
}
//---------------------------------------------------------------------------
/**
 *Finds and returns the direction the robot would move if rotated left
 *@param p the direction to find the left rotation of
 *@return the direction that is to the rotational left
 */
PVector getLeft(PVector p) {
  for (int i=0; i<directions2.size(); i++) {
    if (directions2.get(i).x == p.x && directions2.get(i).y == p.y) {
      if (i == directions2.size()-1) {
        return directions2.get(0);
      } else {
        return directions2.get(i+1);
      }
    }
  }
  return null;
}

/**
 *Finds and returns the direction the robot would move if rotated right
 *@param p the direction to find the right rotation of
 *@return the direction that is to the rotational right
 */
PVector getRight(PVector p) {
  for (int i=0; i<directions2.size(); i++) {
    if (directions2.get(i).x == p.x && directions2.get(i).y == p.y) {
      if (i == 0) {
        return directions2.get(directions2.size()-1);
      } else {
        return directions2.get(i-1);
      }
    }
  }
  return null;
}

int sensors[] = new int[8];
//------------------------------------------------------------------------------
PVector T = new PVector( 0, 0 );
//------------------------------------------------------------------------------
// Define a set of colours for filling in cells in the robot's world when drawn.
final int COLOUR_EMPTY    = #ffffff;
final int COLOUR_GRID     = #999999;
final int COLOUR_OBSTACLE = #666666;
final int COLOUR_PATH     = #ffff00;
final int COLOUR_ROBOT    = #000099;
final int COLOUR_TARGET   = #990000;

//------------------------------------------------------------------------------
// Define a font size that will let us put labels in grid cells 
final int FONT_SIZE = 14;
//------------------------------------------------------------------------------
/**
 * setup()
 * This function is called automatically by Processing when a sketch starts to run.
 */
void setup() { //<>//
  c = '@'; // the config counter representative - as it increments it will turn into 'A', then 'B' etc.
  // set size of robot's world in pixels, which translates to MAX_X x MAX_Y in cells
  // 500 x 500 => (MAX_X * PIXELS_PER_CELL) x (MAX_Y * PIXELS_PER_CELL)
  size( 500, 500 );
  // set font size for labelling cells
  textSize( FONT_SIZE );
  setDirections2();  // populate directions2 treemap
  // initialise locations and robot's world
  reset();
} // end of setup()

//------------------------------------------------------------------------------
/**
 * initWorld()
 * This function initialises the robot's grid world.
 */
void initWorld() {
  // initialise the world to all empty cells
  for ( int y=0; y<MAX_Y; y++ ) {
    for ( int x=0; x<MAX_X; x++ ) {
      world[y][x] = CELL_EMPTY;
    }
  }
} // end of initWorld()
//------------------------------------------------------------------------------
/**
 * pickRandomCell()
 * This function picks a random cell within the robot's world and returns it as a vector.
 * Note that the value returned is in grid cell coordinates.
 */
PVector pickRandomCell() {
  PVector tmp = new PVector();
  tmp.x = int( random( MAX_X ));
  tmp.y = int( random( MAX_Y ));
  return( tmp );
} // end of pickRandomCell()
//------------------------------------------------------------------------------
/**
 * findEmptyCell()
 * This function returns a vector with the coordinates of a randomly chosen empty 
 * cell in the robot's world.
 * Note that the value returned is in grid cell coordinates.
 * Also note that this function is far from perfect (!) for the following reasons:
 * (1) It can fail if there are no empty cells left in the grid.
 * (2) It can take a long time to return if cells are not empty.
 */
PVector findEmptyCell() {
  PVector tmp = pickRandomCell();
  boolean found = false;
  while ( ! found ) {
    if ( world[int(tmp.y)][int(tmp.x)] == CELL_EMPTY ) {
      found = true;
    } else {
      tmp = pickRandomCell();
    }
  } // end while
  return( tmp );
} // end of findEmptyCell()
//------------------------------------------------------------------------------
/**
 * reset()
 * This function (re-)initialises the robot's world.
 */
void reset() {
  PVector tmp;
  // set the trajectory tracker to 0 and increment the config tracker
  c += 1;
  t = 0;
  state = 0;  // set state of robot to 0
  path.clear(); // clear memory of robot path taken
  // Initialise the robot's world
  initWorld();
  // Define locations for obstacles in randomly chosen empty cells
  for ( int num=0; num < NUM_OBSTACLES; num++ ) {
    tmp = findEmptyCell();
    world[int(tmp.y)][int(tmp.x)] = CELL_OBSTACLE;
  }
  // Define a location for the target in a randomly chosen empty cell
  T = findEmptyCell();
  // Define the starting location for robot in a randomly chosen empty cell
  R = findEmptyCell(); //<>//
  storeConfig();  // store the new map config
  // record the old map config, old R and T positions
  setOldWorld(oldWorld, world);
  oldR = new PVector(R.x,R.y);
  oldT = new PVector(T.x,T.y);
  // clear the data structures holding info for A* algorithm
  open.clear();
  closed.clear();
  // activate the robot's sonar sensors
  sense();
  System.out.println("--------------------------");
} // end of reset()
//----------------------------------------------------------------------------
/**
 * Java does not have a perfect clone constructor
 * thus this method clones one 2D int array newW into another 2D int array oldW
 * @params oldW where the previous map config will be saved
 * @params newW the map config to be saved
 * @return the newly populated oldW
 */
int[][] setOldWorld(int[][] oldW, int[][] newW) {
  for (int i=0; i<newW.length; i++) {
    for (int j=0; j<newW.length; j++) {
      oldW[i][j] = newW[i][j];
    }
  }
  return oldW;
}
//---------------------------------------------------------------------------
int t = 0;  // trajectory tracker for the number of times replay() has been called doing the same algorithm
/**
 * Writes to new file the coordinates the robot has taken to reach the goal
 */
void storeTrajectory() {
  if (!prevAlgo.equals(algorithm)) { t = 0; }
  System.out.println("./config"+c+"/"+algorithm+"/trajectory"+t+".csv");
   output = createWriter("./config"+c+"/"+algorithm+"/trajectory"+t+".csv");
   System.out.println("path: "+path);
   for (int i=1; i<path.size(); i++) {
     output.println(path.get(i).x+","+path.get(i).y);
   }
   output.flush();
   output.close();
   t += 1;
}
char c = '@';  // config tracker for the number of times reset() has been called
/**
 * Writes to a new csv file a list of coordinates in the map that have an obstacle, the starting position of R and position of T
 */
void storeConfig() {
  System.out.println(c);
  output = createWriter("./config"+c+"/config.csv");
  output.println("Obstacles: ");
  for (int i=0; i<world.length; i++) {
    for (int j=0; j<world.length; j++) {
      if (world[i][j] == CELL_OBSTACLE) {
        output.println(i+","+j);
      }
    }
  }
  output.println("\n");
  output.println("R: "+R.x+","+R.y);
  output.println("T: "+T.x+","+T.y);
  output.flush();
  output.close();
}
//------------------------------------------------------------------------------
int oldWorld[][] = new int[world.length][world[0].length];
PVector oldR;
PVector oldT;
// Replay function to allow the map to return to when the reset first happened
// so different algorithms can be tried out on the same map
/**
 * Replay function to allow the map to return to when the reset first happened
 * so different algorithms can be tried out on the same map
 */
void replay() {
  storeTrajectory();
  world = setOldWorld(world, oldWorld);
  R = new PVector(oldR.x,oldR.y);
  T = new PVector(oldT.x,oldT.y);
  path.clear();
  open.clear();
  closed.clear();
  sense();
}
//---------------------------------------------------------------------------
/**
 * validCell()
 * This function returns true if the argument (x,y) coordinates are valid within 
 * the robot's grid world coordinate system.
 */
boolean validCell( int x, int y ) {
  if (( x >= 0 ) && ( x < MAX_X ) && ( y >= 0 ) && ( y < MAX_Y )) {
    if (world[y][x] == CELL_EMPTY) {
      return( true );
    } else { return false; }
  } else {
    return( false );
  }
} // end of validCell()
boolean inBoard( int x, int y ) {
  if (( x >= 0 ) && ( x < MAX_X ) && ( y >= 0 ) && ( y < MAX_Y )) {
    return( true );
  } else {
    return( false );
  }
} // end of validCell()
//------------------------------------------------------------------------------
/**
 * sense()
 * This function emulates the operation of the robot's sensors.
 * Each sensor is set equal to the distance (in cells) to the closest obstacle, 
 * within the sensor range (1 cell), for each of the 8 compass directions:
 * 1 0 7
 * 2 R 6
 * 3 4 5
 */
void sense() {
  int cell_x, cell_y;
  // check the cell to the north of the robot
  cell_x = int( R.x + NORTH.x );
  cell_y = int( R.y + NORTH.y );
  for (int i=0; i<sensors.length; i++) {
    sensors[i] = 0;
  }
  while (validCell(cell_x, cell_y) && world[cell_y][cell_x] == CELL_EMPTY) {
    sensors[0] += 1;
    cell_x = int(cell_x + NORTH.x);
    cell_y = int(cell_y + NORTH.y);
  }
  cell_x = int( R.x + NWEST.x );
  cell_y = int( R.y + NWEST.y );
  while (validCell(cell_x, cell_y) && world[cell_y][cell_x] == CELL_EMPTY) {
    sensors[1] += 1;
    cell_x = int(cell_x + NWEST.x);
    cell_y = int(cell_y + NWEST.y);
  }
  // check the cell to the west of the robot
  cell_x = int( R.x + WEST.x );
  cell_y = int( R.y + WEST.y );
  while (validCell(cell_x, cell_y) && world[cell_y][cell_x] == CELL_EMPTY) {
    sensors[2] += 1;
    cell_x = int(cell_x + WEST.x);
    cell_y = int(cell_y + WEST.y);
  }
  cell_x = int( R.x + SWEST.x );
  cell_y = int( R.y + SWEST.y );
  while (validCell(cell_x, cell_y) && world[cell_y][cell_x] == CELL_EMPTY) {
    sensors[3] += 1;
    cell_x = int(cell_x + SWEST.x);
    cell_y = int(cell_y + SWEST.y);
  }
  // check the cell to the south of the robot
  cell_x = int( R.x + SOUTH.x );
  cell_y = int( R.y + SOUTH.y );
  while (validCell(cell_x, cell_y) && world[cell_y][cell_x] == CELL_EMPTY) {
    sensors[4] += 1;
    cell_x = int(cell_x + SOUTH.x);
    cell_y = int(cell_y + SOUTH.y);
  }
  cell_x = int( R.x + SEAST.x );
  cell_y = int( R.y + SEAST.y );
  while (validCell(cell_x, cell_y) && world[cell_y][cell_x] == CELL_EMPTY) {
    sensors[5] += 1;
    cell_x = int(cell_x + SEAST.x);
    cell_y = int(cell_y + SEAST.y);
  }
  // check the cell to the east of the robot
  cell_x = int( R.x + EAST.x );
  cell_y = int( R.y + EAST.y );
  while (validCell(cell_x, cell_y) && world[cell_y][cell_x] == CELL_EMPTY) {
    sensors[6] += 1;
    cell_x = int(cell_x + EAST.x);
    cell_y = int(cell_y + EAST.y);
  }
  cell_x = int( R.x + NEAST.x );
  cell_y = int( R.y + NEAST.y );
  while (validCell(cell_x, cell_y) && world[cell_y][cell_x] == CELL_EMPTY) {
    sensors[7] += 1;
    cell_x = int(cell_x + NEAST.x);
    cell_y = int(cell_y + NEAST.y);
  }
} // end of sense()
//------------------------------------------------------------------------------
/**
 * moveRobot()
 * This function moves the robot in the specified direction.
 * It updates the robot's world accordingly.
 */
void moveRobot( PVector dir ) {
  // Move the robot in the direction specified, as long as it will end up in a 
  // valid cell (i.e., doesn't run off the end of the world). //<>//
  path.add(new PVector(R.x, R.y));
  int move_to_x = int( R.x + dir.x );
  int move_to_y = int( R.y + dir.y );
  if ( validCell( move_to_x, move_to_y )) {
    // Prints out onto console whenever the robot is getting closer to its goal
    if ((T.x - R.x > T.x - move_to_x) && (R.x - T.x < 0)) {
      System.out.println("yippee");
    } else if ((T.x - R.x < T.x - move_to_x) && (R.x - T.x > 0)) {
      System.out.println("yippee");
    } else if ((T.y - R.y > T.y - move_to_y) && (R.y - T.y < 0)) {
      System.out.println("yippee");
    } else if ((T.y - R.y < T.y - move_to_y) && (R.y - T.y > 0)) {
      System.out.println("yippee");
    }
    if (world[move_to_y][move_to_x] == CELL_EMPTY) {
      R.x = move_to_x;
      R.y = move_to_y;
      // Update the robot's sensors from its new location.
      sense();
    } else {
      System.out.println("ouch");  // prints out to console whenever the robot bumps into a wall
    }
  }
} // end of moveRobot()
//------------------------------------------------------------------------------
/**
 * Line of Sight algorithm
 * The robot travels towards the object until it hits an obstacle
 * there it calculates how many turns it will have to make until it can get to a valid square to move onto
 * the side that takes the least amount of turns will then be taken
 * until the line of sight is met again
 */
void lineOfSight() {
  float newx, newy;
  if (R.x != T.x || R.y != T.y) {
    if (R.x > T.x) {
      newx = R.x -1;
    } else if (R.x < T.x) {
      newx = R.x + 1;
    } else {
      newx = R.x;
    }
    if (R.y > T.y) {
      newy = R.y -1;
    } else if (R.y < T.y) {
      newy = R.y + 1;
    } else {
      newy = R.y;
    }
    if (validCell((int)newx, (int)newy)) {
      moveRobot(new PVector(newx-R.x,newy-R.y));
    } else if (inBoard((int)newx, (int)newy)) {
      moveRobot(obstacleTrace(newx-R.x,newy-R.y));
    }
  }
}
/**
 * Direct the robot to move an empty cell around the object
 * @param x x coordinate of the direction the robot would ideally take
 * @param y y coordinate of the direction the robot would ideally take
 * @return a PVector coordinate of the direction to take
 */
PVector obstacleTrace(float x, float y) {
  PVector temp = new PVector(x,y);
  PVector R2 = new PVector(R.x,R.y);
  if (isRight(temp)) {
    PVector right = getRight(temp);
    temp.x = right.x;
    temp.y = right.y;
    R2.x = R.x+right.x;
    R2.y = R.y+right.y;
    while (!validCell((int)(R2.x), (int)(R2.y))) {
      right = getRight(right);
      temp.x = right.x;
      temp.y = right.y;
      R2.x = R.x+right.x;
      R2.y = R.y+right.y;
    }
  } else {
    PVector left = getLeft(temp);
    temp.x = left.x;
    temp.y = left.y;
    R2.x = R.x+left.x;
    R2.y = R.y+left.y;
    while (!validCell((int)(R2.x), (int)(R2.y))) {
      left = getLeft(left);
      temp.x = left.x;
      temp.y = left.y;
      R2.x = R.x+left.x;
      R2.y = R.y+left.y;
    }
  }
  return temp;
}
/**
 * Decider for whether the robot should rotate right or left around the object
 * @param temp direction vector for where the robot would go if there were no obstacle
 * @return true if the amount of times to turn right until a valid empty cell is in front of the robot is less than vice versa
 */
boolean isRight(PVector temp) {
  int tLeft = 0;
  int tRight = 0;
  PVector R2 = new PVector(R.x,R.y);
  if (!inBoard((int)(R.x+getLeft(temp).x), (int)(R.y+getLeft(temp).y))) {
    return true;
  } else if (!inBoard((int)(R.x+getRight(temp).x), (int)(R.y+getRight(temp).y))) {
    return false;
  }
  PVector dir = getLeft(temp);
  while (!validCell((int)(R2.x+dir.x), (int)(R2.y+dir.y))) {
    dir = getLeft(dir);
    R2.x = R.x;
    R2.y = R.y;
    tLeft += 1;
  } 
  dir = getRight(temp);
  while (!validCell((int)(R2.x+dir.x), (int)(R2.y+dir.y))) {
    dir = getRight(dir);
    R2.x = R.x;
    R2.y = R.y;
    tRight += 1;
  }
  if (tRight < tLeft) {
    return true;
  } else { return false; }  // will go left if they are equal or if tLeft>tRight
}
//---------------------------------------------------------------------------
int state = 0;
HashMap<PVector, Integer> open = new HashMap();
ArrayList<PVector> closed = new ArrayList();
/**
 * A* algorithm
 */
void aStar() {
  if (R.x != T.x || R.y != T.y) { //<>//
    if (open.size() > 0) {
      open = sortByValue(open);
      Map.Entry<PVector, Integer> entry = open.entrySet().iterator().next();
      PVector k = entry.getKey();
      closed.add(k);
      open.remove(k);
      ArrayList<PVector> adj = getAdjacent();
      for (int i=0; i<adj.size(); i++) {
        PVector waypoint = adj.get(i);
        if (!open.containsValue(waypoint) &&
            !closed.contains(waypoint) &&
            validCell((int)waypoint.x,(int)waypoint.y)){
          open.put(waypoint, calcH(waypoint));
        }
      }
      PVector n = new PVector(k.x-R.x,k.y-R.y);
      if (n.x == 0 && n.y == 0) {} else { moveRobot(n); } //<>//
    } else {
      open.put(R, calcH(R));
      path.add(R);
    }
  }
}
/**
 * Calculate the H value and returns it
 * @param p PVector to calculate H value of
 * @return H value
 */
Integer calcH(PVector p) {
   return (int) max(abs(p.x-T.x), abs(p.y-T.y));
}
/**
 * Find out and return a list of coordinates in the world that the robot can move to
 * @return arraylist of valid world coords the robot can move to
 */
ArrayList<PVector> getAdjacent() {
  ArrayList<PVector> adj = new ArrayList();
  for (int i=0; i<directions.length; i++) {
    PVector n = new PVector(R.x+directions[i].x, R.y+directions[i].y);
    if (validCell((int) (n.x), (int) (n.y))) {
      adj.add(n);
    }
  }
  return adj;
}
/**
 * Sort passed on HashMap by its value - in this case, sort the open list by its H value
 * @param hm hashmap to be sorted
 * @return sorted hashmap 
 */
HashMap<PVector, Integer> sortByValue(HashMap<PVector, Integer> hm) {
    List<Map.Entry<PVector, Integer> > list = 
           new LinkedList<Map.Entry<PVector, Integer> >(hm.entrySet());
    Collections.sort(list, new Comparator<Map.Entry<PVector, Integer> >() { 
        public int compare(Map.Entry<PVector, Integer> o1,  
                           Map.Entry<PVector, Integer> o2) { 
            return (o1.getValue()).compareTo(o2.getValue()); 
        } 
    }); 
    // put data from sorted list to hashmap  
    HashMap<PVector, Integer> temp = new LinkedHashMap<PVector, Integer>(); 
    for (Map.Entry<PVector, Integer> aa : list) { 
        temp.put(aa.getKey(), aa.getValue()); 
    } 
    return temp; 
} 
//------------------------------------------------------------------------------
/**
 * Bug Algorithm 2
 */
void BugAlgo2() {
  float newx, newy;
  if (R.x != T.x || R.y != T.y) {
    if (R.x > T.x) {
      newx = R.x -1;
    } else if (R.x < T.x) {
      newx = R.x + 1;
    } else {
      newx = R.x;
    }
    if (R.y > T.y) {
      newy = R.y -1;
    } else if (R.y < T.y) {
      newy = R.y + 1;
    } else {
      newy = R.y;
    }
    if (validCell((int)newx, (int)newy)) {
      moveRobot(new PVector(newx-R.x,newy-R.y));
    } else if (inBoard((int)newx, (int)newy)) {
      moveRobot(BugTurn(new PVector(newx-R.x,newy-R.y)));
    }
  }
}
/**
 * Decides the direction of the robot when navigating around the obstacle
 * @param temp the direction the robot would take if there were no objects
 * @return direction the robot will take
 */
PVector BugTurn(PVector temp) {
  PVector R2 = new PVector(R.x,R.y);
  PVector left = getLeft(temp);
  temp.x = left.x;
  temp.y = left.y;
  R2.x = R.x+left.x;
  R2.y = R.y+left.y;
  while (!validCell((int)(R2.x), (int)(R2.y))) {
    //System.out.println("left: "+left);
    left = getLeft(left);
    temp.x = left.x;
    temp.y = left.y;
    R2.x = R.x+left.x;
    R2.y = R.y+left.y;
  }
  return temp;
}
//---------------------------------------------------------------------------
/**
 * keyPressed()
 * This function is called when any key is pressed and do the following:
 *  'q' or 'Q'  : sketch quits (exits)
 *  ' ' (space) : world resets
 *  up-arrow    : robot moves up (north)
 *  down-arrow  : robot moves down (south)
 *  left-arrow  : robot moves left (west)
 *  right-arrow : robot moves right (east)
 */
String algorithm = "";
String prevAlgo = "";  // Stores previous algorithm used so writing to a new folder is tracked
void keyPressed() {
  if (( key == 'q' ) || ( key == 'Q' )) { // q/Q pressed, meaning "quit"
    exit();
  } else if ( key == ' ' ) { // space pressed, meaning "reset"
    reset();
  } else if ((key == 'l') || (key =='L')) {
    if (algorithm.length() > 0) {
      prevAlgo = new String(algorithm);
    }
    algorithm = "lineOfSight";
    state = 2;  // algorithm used are represented by states
  } else if ((key == 'a') || (key =='A')) {
    if (algorithm.length() > 0) {
      prevAlgo = new String(algorithm);
    }
    algorithm = "aStar"; 
    state = 1;
  } else if ((key == 'b') || (key =='B')) {
    if (algorithm.length() > 0) {
      prevAlgo = new String(algorithm);
    }
    algorithm = "bug2";
    state = 3;
  } else if ((key == 'r') || (key =='R')) {
    state = 0;
    replay();
  } else if (key == CODED) {
    state = 0;
    if ( keyCode == UP ) { // up arrow pressed
      moveRobot( NORTH );
    } else if ( keyCode == DOWN ) { // down arrow pressed
      moveRobot( SOUTH );
    } else if ( keyCode == LEFT ) { // left arrow pressed
      moveRobot( WEST );
    } else if ( keyCode == RIGHT ) { // right arrow pressed
      moveRobot( EAST );
    }
  }
} // end of keyPressed()

//------------------------------------------------------------------------------

/**
 * drawGrid()
 * This function draws the robot's world as a 2D grid.
 * Note that the grid is drawn with all cells empty.
 */
void drawGrid() {
  int pixel_x, pixel_y;
  int pixel_x_max = MAX_X * PIXELS_PER_CELL;
  int pixel_y_max = MAX_Y * PIXELS_PER_CELL;
  background( COLOUR_EMPTY );
  stroke( COLOUR_GRID );
  for ( int x=0; x<MAX_X; x++ ) {
    pixel_x = x * PIXELS_PER_CELL;
    line( pixel_x, 0, pixel_x, pixel_x_max );
  }
  for ( int y=0; y<MAX_Y; y++ ) {
    pixel_y = y * PIXELS_PER_CELL;
    line( 0, pixel_y, pixel_x_max, pixel_y );
  }
} // end of drawGrid()

//------------------------------------------------------------------------------

/**
 * fillCell()
 * This function fills the cell located at (x,y) in the grid cell coordinate system.
 */
void fillCell( int x, int y, int colour ) {
  fill( colour );
  int pixel_x = x * PIXELS_PER_CELL;
  int pixel_y = y * PIXELS_PER_CELL;
  rect( pixel_x, pixel_y, PIXELS_PER_CELL, PIXELS_PER_CELL );
} // end of fillCell()

//------------------------------------------------------------------------------

/**
 * drawSensors()
 * This function draws labels in the cells that are sensed by the robot's sensors.
 */
void drawSensors() {
  int cell_x, cell_y, pixel_x, pixel_y;
  String label;
  float labelWidth, labelHeight = FONT_SIZE/2;
  fill( COLOUR_ROBOT );
  // NORTH sensor
  label = "^";
  labelWidth = textWidth( label );
  cell_x = int( R.x + NORTH.x );
  cell_y = int( R.y - sensors[0] + NORTH.y);
  pixel_x = int( cell_x * PIXELS_PER_CELL + ( PIXELS_PER_CELL - labelWidth ) / 2 );
  pixel_y = int( cell_y * PIXELS_PER_CELL + ( PIXELS_PER_CELL + labelHeight ) / 2 );
  text( label, pixel_x, pixel_y ); 
  // SOUTH sensor
  label = "v";
  labelWidth = textWidth( label );
  cell_x = int( R.x + SOUTH.x );
  cell_y = int( R.y + sensors[4] + SOUTH.y);
  pixel_x = int( cell_x * PIXELS_PER_CELL + ( PIXELS_PER_CELL - labelWidth ) / 2 );
  pixel_y = int( cell_y * PIXELS_PER_CELL + ( PIXELS_PER_CELL + labelHeight ) / 2 );
  text( label, pixel_x, pixel_y ); 
  // EAST sensor
  label = ">";
  labelWidth = textWidth( label );
  cell_x = int( R.x + sensors[6] + EAST.x );
  cell_y = int( R.y + EAST.y );
  pixel_x = int( cell_x * PIXELS_PER_CELL + ( PIXELS_PER_CELL - labelWidth ) / 2 );
  pixel_y = int( cell_y * PIXELS_PER_CELL + ( PIXELS_PER_CELL + labelHeight ) / 2 );
  text( label, pixel_x, pixel_y ); 
  // WEST sensor
  label = "<";
  labelWidth = textWidth( label );
  cell_x = int( R.x - sensors[2] + WEST.x );
  cell_y = int( R.y + WEST.y );
  pixel_x = int( cell_x * PIXELS_PER_CELL + ( PIXELS_PER_CELL - labelWidth ) / 2 );
  pixel_y = int( cell_y * PIXELS_PER_CELL + ( PIXELS_PER_CELL + labelHeight ) / 2 );
  text( label, pixel_x, pixel_y );
} // end of drawSensors()

//------------------------------------------------------------------------------

/**
 * draw()
 * This function is called repeatedly by the Processing draw loop and is used to 
 * display the robot's world and everything in it, including the robot.
 */
ArrayList<PVector> path = new ArrayList();
void draw() {
  if (state == 1){   // state value decides which algorithm to use when drawing
    aStar();
  } else if (state == 2) {
    lineOfSight();
  } else if (state == 3) {
    BugAlgo2();
  }
  // Draw robot's world as a grid
  drawGrid();
  // Fill in the cells in the robot's world, according to what is in each cell.
  // Note that we don't need to bother to fill the "empty" cells. 
  for ( int y=0; y<MAX_Y; y++ ) {
    for ( int x=0; x<MAX_X; x++ ) {
      if (( x == R.x ) && ( y == R.y )) {
        fillCell( x, y, COLOUR_ROBOT );
        String label = "O";
        float labelWidth = textWidth( label );
        float labelHeight = FONT_SIZE/2;
        int pixel_x = int( R.x * PIXELS_PER_CELL + ( PIXELS_PER_CELL - labelWidth ) / 2 );
        int pixel_y = int( R.y * PIXELS_PER_CELL + ( PIXELS_PER_CELL + labelHeight ) / 2 );
        fill( #ffffff );
        text( label, pixel_x, pixel_y );
      } else if (path.contains(new PVector(x,y))) {
        fillCell(x,y, COLOUR_PATH);
      } else if (( x == T.x ) && ( y == T.y )) {
        fillCell( x, y, COLOUR_TARGET );
      } else if ( world[y][x] == CELL_OBSTACLE ) {
        fillCell( x, y, COLOUR_OBSTACLE );
      }
    } // end for x
  } // end for y
  // Draw the robot's sensors
  drawSensors();
} // end of draw()
