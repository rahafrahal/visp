/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 *   Test Franka robot behavior
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testFrankaGetForce.cpp

  Test Panda robot from Franka Emika getting measured robot force/torque.
*/


#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_FRANKA)

#include <visp3/robot/vpRobotFranka.h>
#include <visp3/gui/vpPlot.h>


int main(int argc, char **argv)
{
  std::string robot_ip = "192.168.1.1";
  bool opt_plot = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] << " [--ip 192.168.1.1] [--plot] [--help] [-h]"
                           << "\n";
      return EXIT_SUCCESS;
    }
  }

  try {
    std::cout << "Start force/torque measurements test" << std::endl;
    vpRobotFranka robot;
    robot.connect(robot_ip);

    vpColVector tau;

    for (unsigned int i = 0; i < 10; i++) {
      robot.getForceTorque(vpRobot::JOINT_STATE, tau);
      std::cout << "Joint torque: " << tau.t() << std::endl;
      vpTime::wait(10);
    }

    bool end = false;

    vpPlot *plotter = NULL;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, 250 * 2, 500, 10, 10, "Real time force/torque plotter");
      plotter->setTitle(0, "Force N");
      plotter->setTitle(1, "Torque Nm");
      plotter->initGraph(0, 3);
      plotter->initGraph(1, 3);
      plotter->setLegend(0, 0, "Fx");
      plotter->setLegend(0, 1, "Fy");
      plotter->setLegend(0, 2, "Fz");
      plotter->setLegend(1, 0, "Tx");
      plotter->setLegend(1, 1, "Ty");
      plotter->setLegend(1, 2, "Tz");

      vpColVector force_torque;
      while (! end) {
        robot.getForceTorque(vpRobot::END_EFFECTOR_FRAME, force_torque);
        vpColVector force(3);
        vpColVector torque(3);
        for (unsigned int i = 0; i < 3; i++) {
          force[i]  = force_torque[i];
          torque[i] = force_torque[i+3];
        }

        plotter->plot(0, iter_plot, force);
        plotter->plot(1, iter_plot, torque);
        iter_plot++;

        if (vpDisplay::getClick(plotter->I, false)) {
          end = true;
        }
      }
      delete plotter;
    }
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch(const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command"
              << " line option set by default to 192.168.1.1. " << std::endl;
     return EXIT_FAILURE;
  }
  catch(const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "ViSP is not build with libfranka..." << std::endl;
}
#endif
