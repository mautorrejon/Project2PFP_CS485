#include "RigidBodyPlanner.hpp"

#include<iostream>
#include<limits>
#include<time.h>

void wait()
{
	std::cin.ignore(1);
}

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;   
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator

}


RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
{
	RigidBodyMove move;
	
	int verts = m_simulator->GetNrRobotVertices();
	const double* vertices = m_simulator->GetRobotVertices();
	double theta = m_simulator->GetRobotTheta();
	double x = m_simulator->GetRobotX();
	double y = m_simulator->GetRobotY();
	double gx = m_simulator->GetGoalCenterX();
	double gy = m_simulator->GetGoalCenterY();
	int obs = m_simulator->GetNrObstacles();
	//COMMENT!!!
	
	double scale_cs = .025;
	double scale_att = .1; //arbitrarily chosen
	double scale_rep = 3;
	double threshold = 0.5; //arbitrarily chosen threshold distance for Urep
	srand(time(NULL)); //seed random number generator to spike uRep after sufficient amount of time
	double spike = 10.0; //amplifies uRep at random while near obstacles
	int chance = 10; //changes probability of "spiking" uRep

	//initialize default values for gradients
	double uAtt[2] = {0,0};
	double uRep[2] = {0,0};
	double uCS[3] = {0,0,0};

	double Jacobian[2][3];
	//set columns 0 and 1 on all Jacobians
	for(int i=0;i<verts;i++)
	{
		Jacobian[0][0] = 1;
		Jacobian[0][1] = 0;
		Jacobian[0][2] = 0;
		Jacobian[1][0] = 0;
		Jacobian[1][1] = 1;
		Jacobian[1][2] = 0;
	}
	Point oi;

	for(int i=0;i<verts;i++)
	{
		//calc U.att(q)
		uAtt[0] += (vertices[2*i] - gx)*scale_att;
		uAtt[1] += (vertices[2*i+1] - gy)*scale_att;

		//calc U.rep(q) 
		for(int j=0;j<obs;j++)
		{
			oi = m_simulator->ClosestPointOnObstacle(j,vertices[2*i],vertices[2*i+1]);
			
			double xdist = oi.m_x - vertices[2*i];
			double ydist = oi.m_y - vertices[2*i+1];

			double obs_dist = sqrt(pow(xdist,2) + pow(ydist,2));
			
			if(obs_dist < threshold) //if the obstacle is close enough, calc a repelling force for it
			{
				//set scale_rep based on distance
				scale_rep = 1/pow(obs_dist,2); //higher power of denominator increases scaling effect
				//set scale_cs when approaching obstacle, decreasing the robot's velocity
				scale_cs = .025*pow(obs_dist,2);

				uRep[0] += xdist*scale_rep;
				uRep[1] += ydist*scale_rep;

				//while near obstacle, 1-in-chance chance to add large repelling force
				if(rand()%chance==0) //add orthogonal vector with large magnitude to uRep
				{
					double temp = uRep[0];
					double orth_x = -uRep[1]*spike;
					double orth_y = temp*spike;
					if(rand()%2==0) //make the normal vector point in the opposite direction half the time
					{
						orth_x = -orth_x;
						orth_y = -orth_y;
					}

					uRep[0] += orth_x;
					uRep[1] += orth_y;
				}
			}			
		}

		//calc Jacobian
		Jacobian[0][2] += -vertices[2*i]*sin(theta) - vertices[2*i+1]*cos(theta);
		Jacobian[1][2] += vertices[2*i]*cos(theta) - vertices[2*i+1]*sin(theta);

		//calc partial gradient in configuration space
		uCS[0] += uAtt[0] + uRep[0];
		uCS[1] += uAtt[1] + uRep[1];
		uCS[2] += uAtt[0]*(Jacobian[0][2]) + uAtt[1]*(Jacobian[1][2]);
		uCS[2] += uRep[0]*(Jacobian[0][2]) + uRep[1]*(Jacobian[1][2]);

		//reset variables for calculation of partial uCS for next vertex
		Jacobian[0][2] = 0;
		Jacobian[1][2] = 0;
		uAtt[0] = 0;
		uAtt[1] = 0;
		uRep[0] = 0;
		uRep[1] = 0;
	}

	uCS[0] = uCS[0]*scale_cs;
	uCS[1] = uCS[1]*scale_cs;
	uCS[2] = uCS[2]*scale_cs*0.15;
	
	move.m_dx = -uCS[0];
	move.m_dy = -uCS[1];
	move.m_dtheta = -uCS[2];

	return move;
}
