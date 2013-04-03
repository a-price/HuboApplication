#include <iostream>
#include <Hubo_Control.h>

double compareT(Eigen::Isometry3d a,Eigen::Isometry3d b,Eigen::VectorXd weight)
{
	Eigen::Quaterniond qa(a.rotation());
	Eigen::Quaterniond qb(b.rotation());
	Eigen::Vector3d pa=a.translation();
	Eigen::Vector3d pb=b.translation();
	Eigen::VectorXd va(7),vb(7),verr(7),vScaled(7);
	va<<pa,qa;
	vb<<pb,qb;
	verr=pb-pa;
	vScaled=weight.transpose()*Eigen::MatrixXd::Identity(7,7)*verr;
	return vScaled.squaredNorm();
	
	

}
int main(int argc, char** argv)
{
	Hubo_Control hubo;
	Vector6d R1,R2,L1,L2,Rc,Lc; //vectors go here.
	Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
	hubo.getArmAngles(RIGHT, Rc);//double this for left
	hubo.getArmAngles(LEFT, Lc);
	Eigen::Isometry3d Rin,Lin,Rout,Lout;
	Eigen::VectorXd weight(7);
	weight<<1,1,1,1,1,1,1;//change this for weights!
	//disregard sixth and thirteenth
	double mins[14];
	double maxs[14];
	double rando[14];
	int counter=0;
	for(int i=RSP;i<=LWP;i++)
	{
		mins[counter]=hubo.getJointAngleMin(i);
		maxs[counter]=hubo.getJointAngleMax(i);
		int range=(maxs[counter]-mins[counter]);
		int r=rand();
		rando[counter]=r%range+mins[counter];
		counter++;
	}
	R1 <<rando[0],rando[1],rando[2],rando[3],rando[4],rando[6];//these might be off by one, so drop 4 instead of 5 and corresponding l1
	L1 <<rando[7],rando[8],rando[9],rando[10],rando[11],rando[13];
	
	for(int j=0;j<1;j++)//can't handle more than one loop yet
	{
		hubo.huboArmFK(Rin,R1,RIGHT);//takes in r1 angles fro rand
		//and gives Rd, the coords.
		hubo.huboArmFK(Lin,L1,LEFT);
		hubo.huboArmIK(R2,Rin,Rc,RIGHT);
		hubo.huboArmIK(L2,Lin,Lc,LEFT);//R1 starts out
		//IK takes in a desited pose and outputs joint angles
		

	}
	hubo.huboArmFK(Rout,R1,RIGHT);
	hubo.huboArmFK(Lout,L1,LEFT);
	//loop takes in Rin and produces R
	double Left=compareT(Lin,Lout,weight);
	double Right=compareT(Rin,Rout,weight);
	//for loop to go only once
	//compare [333555]
	//quality(F,F')
	std::cout <<"Left "<< Left << " Right "<< Right<<std::endl;
	
	
}

