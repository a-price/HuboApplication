/**
 * \file TestFastrak.cpp
 * \brief Unit tests for Fastrak class
 * 
 * \author Andrew Price
 */

#include "TestFastrak.h"

CPPUNIT_TEST_SUITE_REGISTRATION(TestFastrak);

void TestFastrak::setUp() 
{
	std::cerr << "Setting up...";
	ach_create("fastrak", 10, 3000, NULL);
	//ach_open(chan, "fastrak", NULL);
	//ach_chmod(chan, 0666);
	std::cerr << "Steady...";
	goodFastrak = new Fastrak("fastrak");
	badFastrak = new Fastrak("kittens");
	std::cerr << "Done" << std::endl;
}
void TestFastrak::tearDown() 
{
	std::cerr << "Tidying Up...";
	delete goodFastrak;
	delete badFastrak;
	//ach_close(chan);
	//ach_unlink("fastrak");
	std::cerr << "Done" << std::endl;
}

void TestFastrak::TestScaleGetSet()
{
	std::cerr << "Testing get/set...";
	double scale = 5;
	goodFastrak->setFastrakScale(scale);
	CPPUNIT_ASSERT(goodFastrak->getFastrakScale() == scale);

	scale = -1.2;
	goodFastrak->setFastrakScale(scale);
	CPPUNIT_ASSERT(goodFastrak->getFastrakScale() == scale);

	goodFastrak->setFastrakScale(0);
	CPPUNIT_ASSERT(goodFastrak->getFastrakScale() == scale);
	std::cerr << "Done" << std::endl;
}

void TestFastrak::TestSensorOOB()
{
	std::cerr << "Testing Sensors...";
	Eigen::Isometry3d pose;
	ft_flag_t r;

	std::cerr << "Testing Lower Bound...";
	r = goodFastrak->getPose(pose, -1);
	CPPUNIT_ASSERT(r == SENSOR_OOB);

	std::cerr << "Testing Good...";
	r = goodFastrak->getPose(pose, goodFastrak->getNumChannels()-1);
	CPPUNIT_ASSERT(r == SUCCESS || r == FASTRAK_STALE);

	std::cerr << "Testing Upper Bound...";
	r = goodFastrak->getPose(pose, goodFastrak->getNumChannels()+1);
	CPPUNIT_ASSERT(r == SENSOR_OOB);
	std::cerr << "Done" << std::endl;
}

void TestFastrak::TestAchChannelName()
{
	std::cerr << "Testing Fail on Open...";
	ft_flag_t r = badFastrak->initFastrak();
	CPPUNIT_ASSERT(r == CHAN_OPEN_FAIL);
	std::cerr << "Done" << std::endl;
}

void TestFastrak::TestPoseConversions()
{

}

