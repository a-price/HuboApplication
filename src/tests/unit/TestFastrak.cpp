/**
 * \file TestFastrak.cpp
 * \brief Unit tests for Fastrak class
 * 
 * \author Andrew Price
 */

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "hubo_vision/Fastrak.h"
#include <ach.h>

/**
 * \class TestFastrak
 * \brief Unit tests for Fastrak class.
 */
class TestFastrak : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestFastrak );
	CPPUNIT_TEST(TestScaleGetSet);
	CPPUNIT_TEST(TestSensorOOB);
	CPPUNIT_TEST(TestAchChannelName);
	CPPUNIT_TEST(TestPoseConversions);
	CPPUNIT_TEST_SUITE_END();

public:
	/**
	 * \fn setUp
	 * \brief Initializes test context for following unit tests for Fastrak class.
	 */
	void setUp()
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

	/**
	 * \fn tearDown
	 * \brief Cleans up test context for unit tests for Fastrak class.
	 */
	void tearDown()
	{
		std::cerr << "Tidying Up...";
		delete goodFastrak;
		delete badFastrak;
		//ach_close(chan);
		//ach_unlink("fastrak");
		std::cerr << "Done" << std::endl;
	}

	/**
	 * \fn TestScaleGetSet
	 * \brief Verifies correct behavior of Scale get & set.
	 * Checks get/set correctness for positive and negative ints and doubles.
	 * Checks that 0 scale factors are ignored.
	 */
	void TestScaleGetSet()
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

	/**
	 * \fn TestSensorOOB
	 * \brief Verifies correct behavior of requests for in- and out-of-bounds sensor indices
	 * Checks return values for negative, correct, and big indices.
	 */
	void TestSensorOOB()
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

	/**
	 * \fn TestAchChannelName
	 * \brief Verifies that CHAN_OPEN_FAIL is returned for invalid channel names.
	 */
	void TestAchChannelName()
	{
		std::cerr << "Testing Fail on Open...";
		ft_flag_t r = badFastrak->initFastrak();
		CPPUNIT_ASSERT(r == CHAN_OPEN_FAIL);
		std::cerr << "Done" << std::endl;
	}

	/**
	 * \fn TestPoseConversions
	 * \brief Verifies that overloaded getPose calls return the same basic information.
	 * Not yet implemented
	 */
	void TestPoseConversions()
	{

	}

private:
	ach_channel_t* chan;
	Fastrak* goodFastrak;
	Fastrak* badFastrak;
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestFastrak);

