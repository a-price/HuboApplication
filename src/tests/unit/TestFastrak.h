/**
 * \file TestFastrak.h
 * \brief Unit tests for Fastrak class.
 *
 * \author Andrew Price
 */

#ifndef TEST_FASTRAK_H
#define TEST_FASTRAK_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "Fastrak.h"
#include "ft_flag_t.h"
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
	void setUp();

	/**
	 * \fn tearDown
	 * \brief Cleans up test context for unit tests for Fastrak class.
	 */
	void tearDown();

	/**
	 * \fn TestScaleGetSet
	 * \brief Verifies correct behavior of Scale get & set.
	 * Checks get/set correctness for positive and negative ints and doubles.
	 * Checks that 0 scale factors are ignored.
	 */
	void TestScaleGetSet();

	/**
	 * \fn TestSensorOOB
	 * \brief Verifies correct behavior of requests for in- and out-of-bounds sensor indices
	 * Checks return values for negative, correct, and big indices.
	 */
	void TestSensorOOB();

	/**
	 * \fn TestAchChannelName
	 * \brief Verifies that CHAN_OPEN_FAIL is returned for invalid channel names.
	 */
	void TestAchChannelName();

	/**
	 * \fn TestPoseConversions
	 * \brief Verifies that overloaded getPose calls return the same basic information.
	 * Not yet implemented
	 */
	void TestPoseConversions();

private:
	ach_channel_t* chan;
	Fastrak* goodFastrak;
	Fastrak* badFastrak;
};

#endif // TEST_FASTRAK_H
