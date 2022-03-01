Matlab Project

Organization
------------
The skeleton code consists of following files:
1. runSimulation(s1, t1, s2, t2, timeout): Runs the simulation for given source and destination locations of aircraft until they reach their destination or timeout happens. 
2. controller(in, state): Computes the controller output based on the current state of the aircraft. A sample controller is provided. The code must be replaced with the actual controller. 
3. safetyMonitor(in1, in2): It returns true if the aircraft avoid collision avoidance. Must be implemented. 

Usage
-----
To run the simulation use the following command
runSimulation(s1, t1, s2, t2, timeout)

For example, say A goes from (0, 0) -> (0, 10) and B goes from (-5, 5) -> (5, 5). To run the simulation for at most 30 steps the following command is used. 
runSimulation([0, 0], [0, 10], [-5, 5], [5, 5], 40)



Test Cases: 
-----------
Use the following test cases to test your code:
runSimulation([0, 0], [0, 10], [5, 10], [10, 5], 40)
runSimulation([0, 0], [0, 10], [-5, 5], [5, 5], 40)
runSimulation([0, 0], [0, 10], [10, 0], [0, 0], 40)
runSimulation([0, 0], [10, 10], [10, 0], [0, 10], 40)
runSimulation([0, 5], [15, 5], [5, 10], [15, 0], 40)




