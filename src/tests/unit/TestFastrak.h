/**
 * \file TestFastrak.h
 * \brief Unit tests for Fastrak class
 */

#ifndef TEST_FASTRAK_H
#define TEST_FASTRAK_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "Fastrak.h"
#include "ft_flag_t.h"
#include <ach.h>

class TestFastrak : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestFastrak );
	CPPUNIT_TEST(TestScaleGetSet);
	CPPUNIT_TEST(TestSensorOOB);
	//CPPUNIT_TEST(TestAchChannelName);
	//CPPUNIT_TEST(TestPoseConversions);
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void TestScaleGetSet();
	void TestSensorOOB();
	void TestAchChannelName();
	void TestPoseConversions();

private:
	ach_channel_t* chan;
	Fastrak* goodFastrak;
	Fastrak* badFastrak;
};

#endif // TEST_FASTRAK_H
