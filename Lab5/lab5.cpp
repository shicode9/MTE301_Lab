#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <utility>
#include <vector>
#include <queue>

#include "utils.h"
#include "render.h"

using namespace std;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++Modify my_robot class here+++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// my_robot sub-class
// modify here so it inherits from the Object class from utils.h
class my_robot: public Object {
// define any private or protected members here
private:
int lidar_range{40};
int radius{10};
queue<vector<int>> paths_queue;
int direction = 1;
bool clockwise{true};
grid_util* true_grid_ptr = nullptr;
std::vector<std::vector<int>> grid;

public:
my_robot(int width, int height, const int env_width, const int env_height, int range, int tol, int min_y_spawn, int max_y_spawn) : Object(width, height, env_width, min_y_spawn, max_y_spawn, tol)
{
grid = std::vector<std::vector<int>>(800, std::vector<int>(800, -1));
this->lidar_range = range;
this->radius = width / 2;
}
void setGrid(grid_util& grid) {
true_grid_ptr = &grid;
}

std::vector<std::vector<int>> getGrid(){
return (this->grid);
}

grid_util& true_grid() {
return *true_grid_ptr;
}

//task 1 part 1 code
void lidarCircle(grid_util& true_grid, int env_width, int env_height);

// Detect walls around robot
vector<int> detect_walls() {
vector<int> direction(4, 0);
int cx = x + radius;
int cy = y + radius;
int tol = radius + 10;
const int tol_45 = tol * cos(45*M_PI/180);

if (this->grid[cx][cy - tol] == 1){
direction[0] = 1; // Top Wall
}
else if (this->grid[cx][cy + tol] == 1) {
direction[0] = -1; // Bottom wall
}
if (this->grid[cx + tol][cy] == 1) {
direction[1] = 1; // Right wall
}
else if (this->grid[cx - tol][cy] == 1) {
direction[1] = -1; // Left wall
}
if (this->grid[cx - tol_45][cy - tol_45] == 1) {
direction[2] = 1; // Top-left diagonal
}
else if (this->grid[cx + tol_45][cy + tol_45] == 1) {
direction[2] = -1; // Bottom-right diagonal
}
if (this->grid[cx + tol_45][cy - tol_45] == 1) {
direction[3] = 1; // Top-right diagonal
}
else if (this->grid[cx - tol_45][cy + tol_45] == 1) {
direction[3] = -1;// Bottom-left diagonal
}
return direction;
};

// Check for obstacle collision
bool detect_obstacle() {
int cx = x + radius;
int cy = y + radius;
int check_radius = radius + 8;
for (int dx = -check_radius; dx <= check_radius; dx++) {
for (int dy = -check_radius; dy <= check_radius; dy++) {
int check_x = cx + dx;
int check_y = cy + dy;
if (check_x >= 0 && check_x < (int)grid.size() &&
check_y >= 0 && check_y < (int)grid[0].size()) {
if (this->grid[check_x][check_y] == 2) {
return true;
}
}
}
}
return false;
}

// Determine movement direction for wall following
void find_dir(const vector<int>& direction) {
vector<int> rWall(2, 0);

if (direction[0] == 0 && direction[1] == 0 && direction[2] == 0 && direction[3] == 0) {
rWall = {-1, 0};
} else {
if (clockwise) {
if (direction[2] == 1) {
rWall = {1, -1};
}
if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0) {
rWall = {1, 0};
}
if (direction[3] == 1) {
rWall = {1, 1};
}
if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0) {
rWall = {0, 1};
}
if (direction[2] == -1) {
rWall = {-1, 1};
}
if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0) {
rWall = {-1, 0};
}
if (direction[3] == -1 && direction[2] != 1) {
rWall = {-1, -1};
}
if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0) {
rWall = {0, -1};
}
}
else {
if (direction[2] == 1) {
rWall = {-1, 1};
}
if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0) {
rWall = {0, 1};
}
if (direction[3] == -1) {
rWall = {1, 1};
}
if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0) {
rWall = {1, 0};
}
if (direction[2] == -1) {
rWall = {1, -1};
}
if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0) {
rWall = {0, -1};
}
if (direction[3] == 1 && direction[2] != 1) {
rWall = {-1, -1};
}
if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0) {
rWall = {-1, 0};
}
}
}
paths_queue.push(rWall);
}

void move() {
if (!paths_queue.empty()) {
vector<int> move1 = paths_queue.front();
paths_queue.pop();
this->x += move1[0];
this->y += move1[1];
}
};

void switch_dir() {
if(clockwise == true){
clockwise = false;
} else{
clockwise = true;
}
};

bool get_dir() {
return clockwise;
}

// save grid
void save_grid_csv() {
std::string filename = "grid_pred.csv";
std::ofstream file(filename);

if (!file.is_open()) {
std::cerr << "Error: Could not open file " << filename << std::endl;
return;
}

// determine the maximum row size by finding the size of the longest inner vector
size_t maxRowSize = 0;
for (const auto& col : grid) {
if (col.size() > maxRowSize) {
maxRowSize = col.size();
}
}

// output the grid in transposed form (columns become rows in CSV)
for (size_t row = 0; row < maxRowSize; ++row) {
for (size_t col = 0; col < grid.size(); ++col) {
if (row < grid[col].size()) {
file << grid[col][row];
}
if (col < grid.size() - 1) {
file << ","; // Add comma except after the last element
}
}
file << "\n"; // New line after each row
}

file.close();
std::cout << "Robot's grid written to " << filename << std::endl;
};
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//===== Main parameters =====
const int env_width {800}, env_height {800}; //Width and height of the environment
const int radius {10}; //Radius of the robot's circular body
const int min_obj_size {30}; //Maximum object dimension
const int max_obj_size {40}; //Maximum object dimension
const int occupancy_tol {35}; //Minimum distance between all objects that spawn
int lidar_range{40}; //Lidar range, radiating from center of robot
int tol{5+radius}; //How much closer from farthest lidar range should robot stop in front of obstacle?
int num_objects {6}; //Number of objects in environment

// Grid utility class
grid_util grid(env_width, env_height, min_obj_size, max_obj_size, radius, tol);

// Random generator
random_generator rand_gen;

// Vector of robot positions
std::vector<std::vector<int>> robot_pos;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++DEFINE ANY GLOBAL VARIABLES/FUNCTIONS HERE+++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void my_robot::lidarCircle(grid_util& true_grid, int env_width, int env_height){
int xc = this->x + radius;
int yc = this->y + radius;
int range = this->lidar_range;

for(int i = max(0, xc - range); i <= min(env_width - 1, xc + range); ++i){
for(int j = max(0, yc - range); j <= min(env_height - 1, yc + range); ++j) {
if((i - xc)*(i - xc) + (j - yc)*(j - yc) <= range*range){
int v = this->grid_value(true_grid, this, i, j, range);
if (v == 0 || v == 1) {
this->grid[i][j] = v;
}
}
}
}
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char const *argv[])
{

//==========CREATE ROBOT, GOAL, OBJECTS==========

// read config file
std::tuple<std::string, bool, int, int> config = read_csv();

// create the walls
std::vector<Object*> objects;
// normal perpendicular walls
if (std::get<3>(config) == 4) {
objects = grid.create_walls(std::get<0>(config));
}
// angled walls
else {
objects = grid.create_angled_walls(std::get<0>(config));
}

// create the goal
Object* goal = grid.spawn_object(rand_gen, occupancy_tol, 2);
goal->val = 2;

int min_y_spawn = grid.get_min_y();
int max_y_spawn = grid.get_max_y();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++

// create the objects
std::vector<Object *> obstacles = grid.create_objects_wall(rand_gen, occupancy_tol, tol, radius, num_objects);

// push obstacles to the list of objects
objects.insert(objects.end(), obstacles.begin(), obstacles.end());
// insert goal to list of objects
objects.push_back(goal);

// clear the grid of -1 tolerance values
grid.clear_tol();

// Uncomment this line to write the grid to csv to see the grid as a csv
// grid.writeGridToCSV("grid.csv");

// create robot with range sensor of range 40
my_robot robot(2*radius, 2*radius, env_width, env_height,
lidar_range, tol, min_y_spawn, max_y_spawn);

robot.setGrid(grid);

// create a copy
my_robot robot_init = robot;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++DEFINE ANY LOCAL VARIABLES HERE+++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
vector<int> dir1(4, 0);
int y_ref; // for sweeping
bool sweep {false};
const std::vector<int> dir2(4, 0);
const int strip_height = 50; // vertical spacing between sweep passes
int max_y {0}, min_y {800};
bool avoiding_obstacle = false;
int obstacle_y_ref = 0;
bool go_to_goal = false;
bool obstacle_mode = false;
int obstacle_follow_count = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

robot_pos.push_back({robot.x, robot.y});
int limit_count = 0;

// run the program indefinitely until robot hits the goal or an obstacle
while (true)
{
limit_count++;

//+++++++++++++++WRITE YOUR MAIN LOOP CODE HERE++++++++++++++++++++++

robot.lidarCircle(grid, env_width, env_height);
// task 2
if (go_to_goal) {
int goal_x = goal->x + goal->width / 2;
int goal_y = goal->y + goal->height / 2;
int robot_center_x = robot.x + radius;
int robot_center_y = robot.y + radius;
int dx = robot_center_x - goal_x;
int dy = robot_center_y - goal_y;
if (sqrt(dx*dx + dy*dy) <= radius + 10) {
std::cout << "Goal reached!" << std::endl;
break;
}
// task 1
if (obstacle_mode) {
obstacle_follow_count++;
if (obstacle_follow_count % 20 == 0) {
int test_x = robot.x;
int test_y = robot.y;
if (abs(robot_center_y - goal_y) > abs(robot_center_x - goal_x)) {
test_y += (robot_center_y < goal_y) ? 5 : -5;
} else {
test_x += (robot_center_x < goal_x) ? 5 : -5;
}
int old_x = robot.x, old_y = robot.y;
robot.x = test_x;
robot.y = test_y;
if (!robot.detect_obstacle()) {
obstacle_mode = false;
obstacle_follow_count = 0;
robot.x = old_x;
robot.y = old_y;
} else {
robot.x = old_x;
robot.y = old_y;
}
}
if (obstacle_mode) {
robot.find_dir(robot.detect_walls());
robot.move();
}
} else {

bool moved = false;

if (abs(robot_center_y - goal_y) > 2) {
if (robot_center_y < goal_y) {
robot.y += 1;
} else {
robot.y -= 1;
}
moved = true;
}

else if (abs(robot_center_x - goal_x) > 2) {
if (robot_center_x < goal_x) {
robot.x += 1;
} else {
robot.x -= 1;
}
moved = true;
}

if (moved && robot.detect_obstacle()) {

if (abs(robot_center_y - goal_y) > 2) {
if (robot_center_y < goal_y) {
robot.y -= 1;
} else {
robot.y += 1;
}
} else {
if (robot_center_x < goal_x) {
robot.x -= 1;
} else {
robot.x += 1;
}
}

obstacle_mode = true;
obstacle_follow_count = 0;
std::cout << "Obstacle detected, switching to wall-following mode" << std::endl;
}
}
} else if (!sweep) {

robot.find_dir(robot.detect_walls());
robot.move();

if (robot.y > max_y) {
max_y = robot.y;
}
if (robot.y < min_y) {
min_y = robot.y;
}
if (grid.wall_accuracy(robot.getGrid()) >= 0.80) {

if (robot.y <= min_y + 5 && limit_count > 100) {
y_ref = robot.y;
sweep = true;
}
}
} else {
if (avoiding_obstacle) {
if (robot.y > obstacle_y_ref) {
robot.y = robot.y - 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
robot.lidarCircle(grid, env_width, env_height);
if (robot.y == obstacle_y_ref && !robot.detect_obstacle()) {
avoiding_obstacle = false;
}
continue;
}
}

if (robot.detect_obstacle() && !avoiding_obstacle) {
avoiding_obstacle = true;
obstacle_y_ref = robot.y;

while (robot.detect_obstacle()) {
robot.y = robot.y - 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
robot.lidarCircle(grid, env_width, env_height);
if (limit_count >= 10000) break;
}
continue;
}
robot.find_dir(robot.detect_walls());
if (dir1 == dir2 && robot.detect_walls() != dir2) {
robot.switch_dir();
}
robot.move();
if (robot.y >= y_ref + strip_height) {
if (robot.get_dir()) {
// Moving left
while (robot.x > radius + 10) {
robot.lidarCircle(grid, env_width, env_height);

int cx = robot.x + radius;
int cy = robot.y + radius;
auto robot_grid = robot.getGrid();
if (robot_grid[cx - (radius + 10)][cy] == 1) {
break;
}

if (robot.detect_obstacle()) {
obstacle_y_ref = robot.y;
avoiding_obstacle = true;

while (robot.detect_obstacle()) {
robot.y = robot.y - 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
robot.lidarCircle(grid, env_width, env_height);
if (limit_count >= 10000) break;
}

while (robot.detect_walls() != dir2 && robot.y > obstacle_y_ref) {
robot.lidarCircle(grid, env_width, env_height);
robot.x = robot.x - 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
if (limit_count >= 10000) break;
}

while (robot.y > obstacle_y_ref) {
robot.y = robot.y + 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
robot.lidarCircle(grid, env_width, env_height);
if (limit_count >= 10000) break;
}
avoiding_obstacle = false;
if (limit_count >= 10000) break;
}
robot.x = robot.x - 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
if (limit_count >= 10000) break;
}
y_ref += strip_height;
robot.switch_dir();
} else {
// Moving right
while (robot.detect_walls() != dir2) {
robot.lidarCircle(grid, env_width, env_height);

if (robot.detect_obstacle()) {
obstacle_y_ref = robot.y;
avoiding_obstacle = true;

while (robot.detect_obstacle()) {
robot.y = robot.y + 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
robot.lidarCircle(grid, env_width, env_height);
if (limit_count >= 10000) break;
}

while (robot.detect_walls() != dir2 && robot.y > obstacle_y_ref) {
robot.lidarCircle(grid, env_width, env_height);
robot.x = robot.x + 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
if (limit_count >= 10000) break;
}

while (robot.y > obstacle_y_ref) {
robot.y = robot.y - 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
robot.lidarCircle(grid, env_width, env_height);
if (limit_count >= 10000) break;
}
avoiding_obstacle = false;
if (limit_count >= 10000) break;
}
robot.x = robot.x + 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
if (limit_count >= 10000) break;
}
if (limit_count < 10000) {
while (robot.detect_walls() == dir2) {
robot.lidarCircle(grid, env_width, env_height);
robot.x = robot.x + 1;
robot_pos.push_back({robot.x, robot.y});
limit_count++;
if (limit_count >= 10000) break;
}
}
y_ref += strip_height;
robot.switch_dir();
}
}

if (limit_count % 3 == 0) {
robot.lidarCircle(grid, env_width, env_height);
}
// for checking if sweeping is complete
if (robot.y >= max_y) {
// if sweeping is complete move on to task 2
go_to_goal = true;
}
dir1 = robot.detect_walls();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
robot_pos.push_back({robot.x, robot.y});

if (limit_count >= 10000) {
std::cout << "====Program terminated after " << limit_count << " iterations====" << std::endl;
break;
}

}

float wall_accuracy = grid.wall_accuracy(robot.getGrid()); // for task 1: outer walls
float accuracy = grid.grid_accuracy(robot.getGrid()); // for task 2: entire environment inside walls
std::cout << std::fixed << std::setprecision(2); // set precision for printing
std::cout << "Percent of walls correctly mapped: " << wall_accuracy*100.0 << "%" << std::endl;
std::cout << "Percent of environment correctly mapped: " << accuracy*100.0 << "%" << std::endl;
if (std::get<1>(config)){
render_window(robot_pos, objects, robot_init, env_width, env_height, std::get<2>(config));
}
render_grid(robot_init, robot_pos, robot.getGrid(), env_width, env_height, radius, lidar_range, std::get<2>(config));
return 0;
}
