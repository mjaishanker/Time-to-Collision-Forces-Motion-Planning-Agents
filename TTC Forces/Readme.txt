Mani Jaishanker
Anand Saral
Motion Planning Assignment 2
TTC Force Based Motion Navigation

* To run the program use mini conda to run simulator file using "python simulator.py"
* To change each senario of different agents, change the scenarioFile to different csv files in the simulator.py
* Three csv files to experiment with, 3_agents, 8_agents, crossing_agents
* The senarios to work with are regular ttc based, isotropic ttc based, Force Power law ttc, and relative displacement ttc
* All the calculations for each senarios are done in four different ttc functions
* To run each senarios, change the function call on line 200 of agent.py to ttc, ttcForce, ttcIso, ttcDis
* This will run all the senarios including the extra credit
* Parameters used for each senarios are: epsilon = 0.2 or 0, k = 15, tO = 3, m = 2, e = 2.718, and nu = 0.1
* Epsilon is used for ttc, isotropic, relative displacement, and power law
* K, tO, m, e, and nu parameters are used strictly for the power law implementation