/**
 * \file TestIK.cpp
 * \brief Unit tests for forward and Inverse Kinematics
 *
 * \author Andrew Price
 */

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <iostream>
#include <hubo.h>
#include "HuboKin.h"

typedef Eigen::Matrix< double, 6, 1 > Vector6d;
//#include <Hubo_Control.h>

/**
 * \class TestIK
 * \brief Unit tests for HuboKin FK/IK.
 */
class TestIK : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestIK );
	CPPUNIT_TEST(TestIKMain);
	CPPUNIT_TEST_SUITE_END();

public:

	/**
	 * \fn setUp
	 * \brief Initializes test context for following unit tests for Fastrak class.
	 */
	void setUp()
	{

	}

	/**
	 * \fn tearDown
	 * \brief Cleans up test context for unit tests for Fastrak class.
	 */
	void tearDown()
	{

	}
	
	double compareT(Eigen::Isometry3d a,Eigen::Isometry3d b,Eigen::VectorXd weight)
	{
		Eigen::Quaterniond qa(a.rotation());
		Eigen::Quaterniond qb(b.rotation());
		Eigen::Vector3d pa=a.translation();
		Eigen::Vector3d pb=b.translation();
		Eigen::VectorXd va(7),vb(7),verr(7),vScaled(7);
		va<<pa,qa.x(), qa.y(), qa.z(), qa.w();
		vb<<pb,qb.x(), qb.y(), qb.z(), qb.w();
		verr=vb-va;
		vScaled=weight.cwiseProduct(verr);
		return vScaled.squaredNorm();
	}

	void TestIKMain()
	{
		HK::HuboKin hubo;
		Vector6d R1,R2,L1,L2;  //,Rc,Lc; //vectors go here.
		Eigen::Isometry3d ident = Eigen::Isometry3d::Identity();
		//hubo.getArmAngles(RIGHT, Rc);//double this for left
		//hubo.getArmAngles(LEFT, Lc);
		Eigen::Isometry3d Rin,Lin,Rout,Lout;
		Eigen::VectorXd weight(7);
		weight<<1,1,1,1,1,1,1;//change this for weights!
		//disregard sixth and thirteenth
		double mins[14];
		double maxs[14];
		double rando[14];
		int counter=0;

		HK::Matrix62d lLimits = hubo.kc.getArmLimits(LEFT);
		HK::Matrix62d rLimits = hubo.kc.getArmLimits(RIGHT);

		// Load right arm limits
		for (int i = 0; i < 7; i++)
		{
			if (i == 5) {continue;}
			mins[i] = rLimits(counter,0);
			maxs[i] = rLimits(counter,1);

			mins[i+7] = lLimits(counter,0);
			maxs[i+7] = lLimits(counter,1);

			counter++;
		}

		counter = 0;
		for(int i=RSP;i<=LWP;i++)
		{
			//mins[counter]=hubo.kc.arm_limits.getJointAngleMin(i);
			//maxs[counter]=hubo.getJointAngleMax(i);
			int range=(maxs[counter]-mins[counter]);
			int r=rand();
			rando[counter]=r%range+mins[counter];
			counter++;
		}
		R1 <<rando[0],rando[1],rando[2],rando[3],rando[4],rando[6];//these might be off by one, so drop 4 instead of 5 and corresponding l1
		L1 <<rando[7],rando[8],rando[9],rando[10],rando[11],rando[13];

		for(int j=0;j<1;j++)//can't handle more than one loop yet
		{
			hubo.armFK(Rin,R1,RIGHT);//takes in r1 angles fro rand
			//and gives Rd, the coords.
			hubo.armFK(Lin,L1,LEFT);
			hubo.armIK(R2,Rin,Vector6d::Zero(),RIGHT);
			hubo.armIK(L2,Lin,Vector6d::Zero(),LEFT);//R1 starts out
			//IK takes in a desited pose and outputs joint angles

		}
		hubo.armFK(Rout,R1,RIGHT);
		hubo.armFK(Lout,L1,LEFT);
		//loop takes in Rin and produces R
		double Left=compareT(Lin,Lout,weight);
		double Right=compareT(Rin,Rout,weight);
		//for loop to go only once
		//compare [333555]
		//quality(F,F')
		CPPUNIT_ASSERT(Left < 0.005);
		CPPUNIT_ASSERT(Right < 0.005);
		std::cout <<"Left "<< Left << " Right "<< Right<<std::endl;

	}

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestIK);
