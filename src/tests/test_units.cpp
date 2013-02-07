/**
 * \file test_units.cpp
 * \brief Runs all unit tests defined for HuboApplication
 *
 * \author Andrew Price
 */

#include <cppunit/XmlOutputter.h>
#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TextTestRunner.h>

#include <boost/thread/thread.hpp>
#include <Eigen/Geometry>

#include <iostream>
#include <string>

#include "Fastrak.h"

int main()
{
	// Get the top level suite from the test registry
	CppUnit::Test* suite = CppUnit::TestFactoryRegistry::getRegistry().makeTest();

	// Add the test to the list of tests to run
	CppUnit::TextTestRunner runner;
	runner.addTest(suite);

	// Send the output to cerr when the compiler runs
	runner.setOutputter(new CppUnit::CompilerOutputter(&runner.result(), std::cerr));

	// Actually run the thing.
	bool successful = runner.run();

	// Return result
	return (successful ? 0 : 1);
}
