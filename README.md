# swarm_intelligance
Dissertation on "Software development for drone swarm navigation using digital twins"
# Robomaster EP Swarm Navigation

Welcome to the Robomaster EP Swarm Navigation project! This project focuses on the development and implementation of software to control a swarm of Robomaster EP drones. The system has been tested in the educational environment of the "Kvantorium" Pedagogical Technopark named after V.G. Razumovsky, located at the Glazov State Engineering and Pedagogical University named after V.G. Korolenko.

## Getting Started

To get started with connecting the robots, you need to connect them via "sta" (Wi-Fi) using the code provided in the Robomaster SDK - `05_conn_sta_helper.py`. Follow the steps below to set up and run the system.

## Step 1: Establish Wi-Fi Connection

Connect your Robomaster EP drones to the Wi-Fi network using the provided helper script:

### ```python
### Example usage of the connection script
### python 05_conn_sta_helper.py

## Step 2: Create a Map of the Area

Create a matrix to represent the area where the robots will navigate. In this matrix:
0 represents a navigable space.
1 represents an obstacle.
The mapping of the area should reflect the real-world layout where the robots will operate.

## Step 3: Define Starting Positions
Determine the initial positions (i, j) for your robots within the matrix. Update the code with these starting positions. Refer to Example Figure 1 for guidance.
### 
# Example of setting start positions
start_positions = [(i1, j1), (i2, j2), ...]

## Step 4: Place ArUco Markers
Randomly place ArUco markers within the area to assist with navigation and localization. Position the robots at their defined start locations.

## Step 5:
After setting up the environment and placing the robots, execute the program to start the swarm navigation.

# Run the navigation script
python swarm_navigation.py

## Testing Environment
This project has been rigorously tested within the facility of the "Kvantorium" Pedagogical Technopark named after V.G. Razumovsky, ensuring a reliable and educational experience.

## Example Figure 1
Unable to render image

This figure illustrates how to set up the initial positions and map layout for the robots.
Thank you for using the Robomaster EP Swarm Navigation project. We hope this guide helps you set up your swarm and explore the possibilities of synchronized multi-robot navigation.

Feel free to customize the text and layout to better suit your project needs!
