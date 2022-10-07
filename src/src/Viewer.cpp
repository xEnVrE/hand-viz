/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/iCubCamera.h>

#include <RobotsViz/VtkContainer.h>
#include <RobotsViz/VtkContent.h>
#include <RobotsViz/VtkPointCloud.h>
#include <RobotsViz/VtkiCubHand.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace RobotsIO::Camera;
using namespace RobotsViz;


int main(int argc, char** argv)
{
    if ((argc < 10) || ((std::string(argv[9]) == "true") && (argc != 11)))
    {
        std::cout << "Synopsis: robmo-icub-viz <blocking> <robot_name> <hand_laterality> <use_fingers> <use_analogs> <hand_fk> <hand_aruco> <hand_debug> <point_cloud> [<camera>]" << std::endl;
        std::cout << "          Camera name <camera> is required only if <point_cloud> = true." << std::endl;
        return EXIT_FAILURE;
    }
    bool blocking = false;
    bool use_fingers = false;
    bool use_analogs = false;
    bool show_hand_fk = false;
    bool show_hand_aruco = false;
    bool show_hand_debug = false;
    bool show_point_cloud = false;

    if (std::string(argv[1]) == "true")
        blocking = true;
    const std::string robot_name = std::string(argv[2]);
    const std::string hand_laterality = std::string(argv[3]);
    if (std::string(argv[4]) == "true")
        use_fingers = true;
    if (std::string(argv[5]) == "true")
        use_analogs = true;
    if (std::string(argv[6]) == "true")
        show_hand_fk = true;
    if (std::string(argv[7]) == "true")
        show_hand_aruco = true;
    if (std::string(argv[8]) == "true")
        show_hand_debug = true;
    if (std::string(argv[9]) == "true")
        show_point_cloud = true;
    std::string camera_name = "";
    if (show_point_cloud)
        camera_name = std::string(argv[10]);

    double fps = 30.0;
    VtkContainer container(1.0 / fps, 600, 600, blocking);

    /* Show hand according to forward kinematics. */
    if (show_hand_fk)
    {
        std::unique_ptr<VtkContent> hand = std::unique_ptr<VtkiCubHand>
        (
            new VtkiCubHand(robot_name, hand_laterality, "test-visualization/hand_fk", use_fingers, use_analogs, {100.0 / 255.0, 160 / 255.0, 255.0 / 255.0}, 1.0)
        );
        container.add_content("hand_fk", std::move(hand));
    }

    /* Show additional hand for debugging purposes. */
    // if (show_hand_debug)
    // {
    //     std::unique_ptr<VtkContent> hand_debug = std::unique_ptr<VtkiCubHand>
    //     (
    //         new VtkiCubHand(robot_name, hand_laterality, "test-visualization/hand_debug", use_fingers, use_analogs, {60.0 / 255.0, 180.0 / 255.0, 60.0 / 255.0}, 1.0)
    //     );

    //     container.add_content("hand_aruco_debug", std::move(hand_debug));
    // }

    container.run();

    return EXIT_SUCCESS;
}
