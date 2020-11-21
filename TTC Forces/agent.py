# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from math import sqrt


class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=5, goalRadiusSq=1, maxF = 10):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq = goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = dhor # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent

    def computeForces(self, neighbors=[]):
        """ 
            Your code to compute the forces acting on the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """     

        valid_neighbor = []
        
        ep = 0 
        m = 2
        k = 15
        tau0 = 3
        e = 2.718
        vu = 0        
        use_power_law = True
        
        ep2 = ep * ep
        
        def calc_mag(v):
            return sqrt(v[0]*v[0] + v[1]*v[1])
        
        def calc_ttc_force(a1,a2):      
            
            r = vu * sqrt(np.random.uniform(0,1))
            angle = np.random.uniform(0,2) * np.pi
            eta = [r * np.cos(angle), r * np.sin(angle)]
        
            x = [a1.pos[0] - a2.pos[0], a1.pos[1] - a2.pos[1]]
            r = a1.radius + a2.radius
            c = (np.dot(x, x)) - (r * r) 
            if c < 0:
                print("COLLISION!")
                return np.zeros(2)
            
            v = [a1.vel[0] - a2.vel[0], a1.vel[1] - a2.vel[1]]
            v = v + eta
            v = v - ep * (x/calc_mag(x))
            b = np.dot(x, v) - ep * r
            if b > 0:
                return np.zeros(2)
           
            a = np.dot(v, v) - ep * ep
            d = (b * b) - a*c           
            if d <= 0:
                return 0
            tau = c/(-b + sqrt(d))
            if tau < 0:
               return np.zeros(2)   
            

            col_dir = x + np.dot(v,tau)
            col_dir = col_dir/calc_mag(col_dir)
            
            f = max(self.timehor - tau,0)/tau
            
            return col_dir*f 
            
            
        def calc_powerlaw_force(a1,a2):           
            x = [a1.pos[0] - a2.pos[0], a1.pos[1] - a2.pos[1]]
            r = a1.radius + a2.radius
            c = (np.dot(x, x)) - (r * r) 
            if c < 0:
                print("COLLISION!")
                return 0
            
            v = [a1.vel[0] - a2.vel[0], a1.vel[1] - a2.vel[1]]
            b = np.dot(x, v) - ep * r
            if b > 0:
                return 0
           
            a = np.dot(v, v) - ep * ep
            d = (b * b) - a*c           
            if d <= 0:
                return 0
            tau = (-b - sqrt(d))/a
            if tau < 0:
               return 0           
             
            if(tau0 != 0):
                f =(k * pow(e,-tau/tau0 )/(pow(tau,m+1)))*(m + tau/tau0) * ((x+np.dot(v,tau))/sqrt(d))
            else:
                f = 2 * k * ((x+np.dot(v,tau))/sqrt(d)/t**(m+1))
            
            return f
        
        for n in neighbors:
            if self.id != n.id and (sqrt((n.pos[0] - n.goal[0])**2 + (n.pos[1] - n.goal[1])**2) > 1) and n.atGoal == False:
                dist =  sqrt((self.pos[0] - n.pos[0])**2 + (self.pos[1] - n.pos[1])**2)
               # print(dist," ",self.dhor)
                if dist <= self.dhor:
                    valid_neighbor.append(n)
        
        fg = (self.gvel - self.vel)/self.ksi
       
       # print(valid_neighbor)
        for n in valid_neighbor:
            if(use_power_law == True):
                fg = fg + calc_powerlaw_force(self,n)
            else:
                fg = fg + calc_ttc_force(self,n)
        
        
        if not self.atGoal:
            self.F = fg
            

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        
        """
        mag = sqrt(self.F[0] * self.F[0] + self.F[1] * self.F[1])
        if  mag > self.maxF:
            self.F = (self.F/mag) * self.maxF
        
        if not self.atGoal:
            self.vel += self.F*dt     # update the velocity
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  
            
            
  