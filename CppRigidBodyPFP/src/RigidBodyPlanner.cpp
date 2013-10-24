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
	
	//printf("Hello");
	//add your implementation
	
	//calculate the U(q) potential
	
	int verts = m_simulator->GetNrRobotVertices();
	//printf("%d",verts);
	const double* vertices = m_simulator->GetRobotVertices();
	double theta = m_simulator->GetRobotTheta();
	double x = m_simulator->GetRobotX();
	double y = m_simulator->GetRobotY();
	double gx = m_simulator->GetGoalCenterX();
	double gy = m_simulator->GetGoalCenterY();
	int obs = m_simulator->GetNrObstacles();
	
	double scale_cs = .025;
	double scale_att = .1; //arbitrarily chosen
	double scale_rep = 3;
	double threshold = 0.5; //arbitrarily chosen threshold distance for Urep
	srand(time(NULL)); //seed random number generator to spike uRep after sufficient amount of time
	double spike = 20.0; //amplifies uRep at random while near obstacles
	int chance = 10; //changes probability of "spiking" uRep

	//initialize default values for gradients
	double uAtt[2] = {0,0};
	double uRep[2] = {0,0};
	double uCS[3] = {0,0,0};

	//printf("uAtt.x = %f, uAtt.y = %f\nuRep.x = %f, uRep.y = %f",uAtt[0],uAtt[1],uRep[0],uRep[1]);

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
	//wait(); //HERE IS THE KEY THE KEY THE KEY THE KEY
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
			//printf("\nobs_dist = %f",obs_dist);
			
			if(obs_dist < threshold) //if the obstacle is close enough, calc a repelling force for it
			{
				//set scale_rep based on distance
				scale_rep = 2/pow(obs_dist,2);

				scale_cs = .025*pow(obs_dist,2);

				uRep[0] += xdist*scale_rep;
				uRep[1] += ydist*scale_rep;

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

				//spike = 1; //reset spike value

				printf("\nspike = %f",spike);
			}
			//double disty = uRep[1]/scale_rep;
			//printf("\ndistx = %f\ndisty = %f",distx,disty);
		}

		spike = 1; //reset spike value

		//printf("\nuAtt.x = %f, uAtt.y = %f\nuRep.x = %f, uRep.y = %f\noi.x = %f, oi.y = %f",uAtt[0],uAtt[1],uRep[0],uRep[1],oi.m_x,oi.m_y);
		//wait(); //HERE IS THE KEY HERE IS THE KEY HERE IS THE KEY HERE IS THE KEY 

		//calc Jacobian
		Jacobian[0][2] += -vertices[2*i]*sin(theta) - vertices[2*i+1]*cos(theta);
		Jacobian[1][2] += vertices[2*i]*cos(theta) - vertices[2*i+1]*sin(theta);
		//printf("\nvertices[2*i] = %f, vertices[2*i+1] = %f",vertices[2*i],vertices[2*i+1]);
		//printf("\nJaco[0][2] = %f, Jaco[1][2] = %f",Jacobian[0][2],Jacobian[1][2]);

		//wait();
		//calc partial gradient in configuration space
		uCS[0] += uAtt[0] + uRep[0];
		uCS[1] += uAtt[1] + uRep[1];
		uCS[2] += uAtt[0]*(Jacobian[0][2]) + uAtt[1]*(Jacobian[1][2]);
		uCS[2] += uRep[0]*(Jacobian[0][2]) + uRep[1]*(Jacobian[1][2]);
		//printf("\nuCS[0] = %f\nuCS[1] = %f\nuCS[2] = %f",uCS[0],uCS[1],uCS[2]);

		//reset variables for calculation of partial uCS for next vertex
		Jacobian[0][2] = 0;
		Jacobian[1][2] = 0;
		uAtt[0] = 0;
		uAtt[1] = 0;
		uRep[0] = 0;
		uRep[1] = 0;
	}
	//printf("REACHED");
	uCS[0] = uCS[0]*scale_cs;
	uCS[1] = uCS[1]*scale_cs;
	uCS[2] = uCS[2]*scale_cs*0.15;
	
	move.m_dx = -uCS[0];
	move.m_dy = -uCS[1];
	move.m_dtheta = -uCS[2];

	printf("\n\t%f\t%f\t%f",move.m_dx,move.m_dy,move.m_dtheta);

	//wait();
	return move;
}
