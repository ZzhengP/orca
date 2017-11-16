/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/ModelIO/URDFGenericSensorsImport.h>
#include <iDynTree/Sensors/Sensors.h>
#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
using namespace iDynTree;

void checkURDF(std::string fileName,
               unsigned int expectedNrOfAccelerometers,
               unsigned int expectedNrOfGyroscopes)
{
    iDynTree::SensorsList sensorList;
    iDynTree::sensorsFromURDF(fileName,sensorList);

    std::cout<<"Sensor list created from URDF. num accel : "<<sensorList.getNrOfSensors(iDynTree::ACCELEROMETER)
    <<", num gyro : "<<sensorList.getNrOfSensors(iDynTree::GYROSCOPE)<<std::endl;

    ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::ACCELEROMETER),expectedNrOfAccelerometers);
    ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::GYROSCOPE),expectedNrOfGyroscopes);

    // Load the reduced model without any joint (this will clamp all the link in one big link)
    iDynTree::ModelLoader mdlLoader;
    std::vector<std::string> consideredJoints;
    bool ok = mdlLoader.loadReducedModelFromFile(fileName,consideredJoints);

    ASSERT_IS_TRUE(ok);

    // The number of gyro and accelerometers (and in general of link sensors) should be converved
    ASSERT_EQUAL_DOUBLE(mdlLoader.sensors().getNrOfSensors(iDynTree::ACCELEROMETER),expectedNrOfAccelerometers);
    ASSERT_EQUAL_DOUBLE(mdlLoader.sensors().getNrOfSensors(iDynTree::GYROSCOPE),expectedNrOfGyroscopes);
}


int main()
{
    std::cout<<"Generic Sensor test running (one Link):\n";
    checkURDF(getAbsModelPath("oneLink.urdf"),2,1);

    std::cout<<"Generic Sensor test running (two Link):\n";
    checkURDF(getAbsModelPath("/twoLinks.urdf"),2,1);

    std::cout <<"Generic Sensor test just ran\n";
    return 0;
}
