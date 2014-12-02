# Modified by
# Andre Santos, November 2014

import query_git as qg
import db_extract as dbe
import time

import package
from sys import exit


# Retrieves all people of every package
def allPkgPpl(packages):
    mntnrs = reduce(set.union, [p.maintainers.people for p in packages.values()])
    authors = reduce(set.union, [p.authors.people for p in packages.values()])
    people = mntnrs.union(authors)

    return people


# Data retrieval to build tables...

# Tables with only primary key, no foreign key/direct linking:

def getPkgInfo(packages, info):
    # Returns a list of tuples, each tuple describes a single package
    lot = []
    for idx,p in enumerate(packages.values(), start=1):
        l = [idx] # starts as a list, will be converted to tuple and appended
        for inf in info:
            l.append(eval('p.'+inf))
        lot.append(tuple(l))
    return lot

def getPplNames(people):
    # Retrieves general information of people based on package dict (maintainers 
    # or authors), as a list of tuples; each tuple describes a specific person
    lot = []
    for idx,p in enumerate(people, start=1):
        lot.append((idx, p.name))
    return lot

def getPplInfo(packages, people, info):
    # Retrieves general information of people based on package dict (maintainers 
    # or authors), as a list of tuples; each tuple describes a specific person
    lot = []
    for idx,p in enumerate(people, start=1):
        l = [idx]
        for i in info:
            try:
                l.append(eval('p.'+i))
            except:
                l.append(None) # append None if attribute is absent
        lot.append(tuple(l))
    return lot

def getLicenses(packages):
    lot = []
    c = 1
    for idx,p in enumerate(packages.values(), start=1):
        for license in p.licenses:
            if not any(license in t for t in lot):
                lot.append((c,license))
                c += 1
    return lot

def getRepoNames(packages):
    # Creates a list of the unique repository names of all the packages
    l = []
    for p in packages.values():
        for repo_name in p.repo.repo_names:
            if repo_name not in l:
                l.append(repo_name)
    return l

def getRepoInfo(repo_names, repo_info_keys):
    # Retrieves the information for each repository's dictionary based on provided keys
    lot = []
    c = 1
    for repo_name in repo_names:
        repo_info = qg.getRepoInfo(repo_name, repo_info_keys)
        if repo_info.count(None) == len(repo_info_keys):
            continue # If this repo name is invalid it returns a list of None's
        t = [c, repo_name] + repo_info
        c += 1
        lot.append(t)
    return lot

def getGitEmails(git_names):
    lot = []
    for idx,git_name in enumerate(git_names, start=1):
        git_row = list(git_name)
        git_row.insert(0,idx)
        git_row.append(qg.getUserEmail(git_name[0]))
        lot.append(tuple(git_row))
    return lot



# Linked tables, both a primary key and a foreign key (many to one)

def getIssuesLinkRepos(repo_ids, issues_info_keys):
    lot = []
    c = 1
    for repo_id,repo_name in repo_ids:
        issues_info = qg.getIssuesInfo(repo_name, issues_info_keys)
        for issue_info in issues_info:
            if issue_info == None: # This repo name is invalid, don't append anything
                print "   No Issues Found  :  ",repo_id, repo_name
                break
            issue_info.insert(0,c) # Id inserted in the front of the row
            c += 1
            issue_info.append(repo_id)
            lot.append(issue_info)
    return lot

def getIssuesLabels(issue_cols, issues_info):
    label_ids = []
    issues_labels = []
    c = 0
    label_idx = issue_cols.index('labels')
    del issue_cols[label_idx]
    for idx,issue_info in enumerate(issues_info):
        issue_id = issue_info[0]
        issue_labels = issue_info[label_idx]
        del issues_info[idx][label_idx]
        
        if issue_labels == None: continue
        issue_labels = issue_labels.split(',')
        
        for issue_label in issue_labels:
            if not any(issue_label in t for t in label_ids): # Add new label
                c += 1
                label_ids.append((c,issue_label))
            for label_id,label in label_ids:
                if label == issue_label:
                    issues_labels.append((issue_id,label_id))
    
    # issues_cols = column list for Issues tables after label removal
    # issues_info = info list for Issues tables after label removal
    # label_ids = id label pairing for Labels table
    # issues_labels = label id, issue id pairing for IssuesLabels map table
    return issue_cols, issues_info, label_ids, issues_labels


def getPkgFiles(pkg_ids, src_dict):
    lot = []
    idx = 1
    for pkg_id, pkg_name in pkg_ids:
        srcs = src_dict.get(pkg_name)
        if srcs is None:
            continue
        for src in srcs:
            lot.append((idx, pkg_id, src.name, src.path))
            idx += 1
    return lot


# Mapping Tables, only foreign keys

def mapPkgDeps(pkg_ids, dep_type_ids, packages):
    # Retrieves package dependencies as a list of tuples, each tuple describes a direct
    # dependency relation. Many to many
    lot = []
    for pkg_id, pkg_name in pkg_ids:
        for dep_type_id,dep_type in dep_type_ids:
            deps = eval("packages['"+pkg_name+"']."+dep_type)
            for dep in deps:
                dep_ids = [p[0] for p in pkg_ids if p[1] == dep]
                for dep_id in dep_ids:
                    lot.append((pkg_id,dep_id,dep_type_id))
    return lot

def mapPkgLicenses(pkg_ids, license_ids, packages):
    # Maps the many to many relation of package and license type
    lot = []
    for pkg_id,pkg_name in pkg_ids:
        for license_id,license_name in license_ids:
            for pkg_license in packages[pkg_name].licenses:
                if pkg_license == license_name:
                    lot.append((pkg_id,license_id))
    return lot


def mapPkgAMs(pkg_ids, ppl_ids, packages):
    # Retrieves package maintainers/authors as a list of tuples, each tuple 
    # describes a package maintainer/author. Many to many
    a_lot = []
    m_lot = []
    for pkg_id,pkg_name in pkg_ids:
        for ppl_id,ppl_name in ppl_ids:
            if packages[pkg_name].authors.find(ppl_name) != None:
                a_lot.append((pkg_id,ppl_id))
            if packages[pkg_name].maintainers.find(ppl_name) != None:
                m_lot.append((pkg_id,ppl_id))
    return a_lot, m_lot

def mapPkgRepos(pkg_ids, repo_ids, packages):
    # Retrieves package repository relations as a list of tuples, each tuple
    # describes a package repository relation. Many to many
    lot = []
    for pkg_id, pkg_name in pkg_ids:
        for repo_id, repo_name in repo_ids:
            if repo_name in packages[pkg_name].repo.repo_names:
                lot.append((pkg_id,repo_id))
    return lot

def mapGitPpl(git_emails, ppl_emails):
    # Maps matching github account emails with people's listed emails in
    # package manifests
    lot = []
    idx = 1
    for git_id, git_email in git_emails:
        for ppl_id, ppl_email in ppl_emails:
            if git_email == ppl_email and git_email != None:
                lot.append((idx, git_id,ppl_id))
                idx += 1
    
    return lot


if __name__ == '__main__':
    # Repos_dict = repo.makeReposDict('distribution.yaml')
    # packages = package.makePackageDict('./manifests', Repos_dict)
    # pkg_ids = [(1, 'rosconsole'), (2, 'sr_gui_motor_resetter'), (3, 'angles'), (4, 'calibration_estimation'), (5, 'rosgraph'), (6, 'orocos_kinematics_dynamics'), (7, 'rocon_python_comms'), (8, 'test_bond'), (9, 'ros'), (10, 'flir_ptu_driver'), (11, 'rocon_gateway_utils'), (12, 'mjpeg_server'), (13, 'nmea_comms'), (14, 'libccd'), (15, 'tf'), (16, 'rqt_publisher'), (17, 'desire_description'), (18, 'smach_ros'), (19, 'timestamp_tools'), (20, 'diagnostic_aggregator'), (21, 'rocon_service_pair_msgs'), (22, 'object_recognition_tod'), (23, 'ecl_streams'), (24, 'rqt_topic'), (25, 'cob_srvs'), (26, 'ecl_io'), (27, 'rosboost_cfg'), (28, 'concert_scheduling'), (29, 'rqt_service_caller'), (30, 'sr_ronex_launch'), (31, 'freenect_camera'), (32, 'concert_tutorial'), (33, 'sr_gui_muscle_driver_bootloader'), (34, 'jaco_description'), (35, 'kobuki_dashboard'), (36, 'yocs_velocity_smoother'), (37, 'rosserial_windows'), (38, 'rqt_pose_view'), (39, 'nao_robot'), (40, 'rocon_tutorials'), (41, 'concert_schedulers'), (42, 'hector_imu_attitude_to_tf'), (43, 'ros_ethercat_hardware'), (44, 'pcl_conversions'), (45, 'geometry_tutorials'), (46, 'arbotix_msgs'), (47, 'humanoid_nav_msgs'), (48, 'rqt_image_view'), (49, 'sr_hardware_interface'), (50, 'rqt_console'), (51, 'sicktoolbox'), (52, 'ecl_sigslots'), (53, 'test_roscpp'), (54, 'nav2d_tutorials'), (55, 'ecto'), (56, 'rosh_desktop'), (57, 'brics_actuator'), (58, 'image_transport'), (59, 'rqt_gui'), (60, 'sr_ronex_utilities'), (61, 'test_rosbag_storage'), (62, 'qt_gui_py_common'), (63, 'rosserial_embeddedlinux'), (64, 'ipa_canopen_core'), (65, 'prosilica_gige_sdk'), (66, 'diagnostic_common_diagnostics'), (67, 'rostopic'), (68, 'visualization_msgs'), (69, 'message_generation'), (70, 'camera_handler'), (71, 'ecto_image_pipeline'), (72, 'audio_common'), (73, 'kobuki_gazebo'), (74, 'polled_camera'), (75, 'rocon_gateway_tutorials'), (76, 'ecl_threads'), (77, 'interval_intersection'), (78, 'ecl_core'), (79, 'rospy'), (80, 'hector_marker_drawing'), (81, 'roscpp_tutorials'), (82, 'ecl_build'), (83, 'sr_visualization'), (84, 'driver_common'), (85, 'python_orocos_kdl'), (86, 'm4atx_battery_monitor'), (87, 'robot_model'), (88, 'kobuki_qtestsuite'), (89, 'visp_hand2eye_calibration'), (90, 'rqt_common_plugins'), (91, 'microstrain_3dmgx2_imu'), (92, 'concert_service_admin'), (93, 'rocon_python_utils'), (94, 'rosmsg'), (95, 'arbotix_controllers'), (96, 'rocon_unreliable_experiments'), (97, 'audio_capture'), (98, 'gazebo_ros_pkgs'), (99, 'roslib'), (100, 'hector_map_tools'), (101, 'moveit_planners_ompl'), (102, 'moveit_simple_grasps'), (103, 'diagnostic_updater'), (104, 'rosapi'), (105, 'rgbd_launch'), (106, 'rocon_tools'), (107, 'rqt_runtime_monitor'), (108, 'velocity_controllers'), (109, 'ecl_eigen'), (110, 'vision_opencv'), (111, 'visp_camera_calibration'), (112, 'hector_trajectory_server'), (113, 'korg_nanokontrol'), (114, 'yocs_controllers'), (115, 'unique_identifier'), (116, 'topic_tools'), (117, 'interactive_marker_tutorials'), (118, 'rqt_robot_plugins'), (119, 'stage_ros'), (120, 'libntcan'), (121, 'rqt_robot_steering'), (122, 'hardware_interface'), (123, 'geographic_msgs'), (124, 'ar_track_alvar_msgs'), (125, 'arbotix'), (126, 'sensor_msgs'), (127, 'roscreate'), (128, 'rqt_launch'), (129, 'settlerlib'), (130, 'rocon_python_wifi'), (131, 'roseus_msgs'), (132, 'ecl_core_apps'), (133, 'gazebo_ros'), (134, 'sr_moveit_config'), (135, 'statistics_msgs'), (136, 'ecl_lite'), (137, 'shape_msgs'), (138, 'opencv_candidate'), (139, 'rospy_message_converter'), (140, 'rosh_robot_plugins'), (141, 'ecl_math'), (142, 'mk'), (143, 'executive_smach_visualization'), (144, 'rocon_app_platform'), (145, 'vision_visp'), (146, 'ros_ethercat_model'), (147, 'master_sync_fkie'), (148, 'joint_states_settler'), (149, 'ecl_linear_algebra'), (150, 'rail_maps'), (151, 'ipa_canopen'), (152, 'multimaster_msgs_fkie'), (153, 'rocon_semantic_version'), (154, 'camera_info_manager_py'), (155, 'concert_service_manager'), (156, 'robot_localization'), (157, 'nav2d_msgs'), (158, 'sr_standalone'), (159, 'wiimote'), (160, 'test_package_for_rapps'), (161, 'move_slow_and_clear'), (162, 'osm_cartography'), (163, 'pcl_ros'), (164, 'rqt_srv'), (165, 'rqt_moveit'), (166, 'route_network'), (167, 'ecl_ipc'), (168, 'qt_gui_core'), (169, 'pkg_with_duplicate_names'), (170, 'sr_edc_ethercat_drivers'), (171, 'kobuki_safety_controller'), (172, 'object_recognition_reconstruction'), (173, 'hector_geotiff_plugins'), (174, 'zeroconf_avahi_suite'), (175, 'robot_base_pkg'), (176, 'uvc_camera'), (177, 'manipulator_handler'), (178, 'freenect_launch'), (179, 'diff_drive_controller'), (180, 'moveit_visual_tools'), (181, 'gmapping'), (182, 'libpointmatcher'), (183, 'rocon_bubble_icons'), (184, 'rocon_tf_reconstructor'), (185, 'laser_assembler'), (186, 'urg_c'), (187, 'test_rosparam'), (188, 'sr_gui_hand_calibration'), (189, 'a'), (190, 'rqt_tf_tree'), (191, 'gazebo_ros_control'), (192, 'joint_trajectory_controller'), (193, 'collada_parser'), (194, 'euslisp'), (195, 'smach'), (196, 'yocs_virtual_sensor'), (197, 'sr_edc_controller_configuration'), (198, 'rocon_concert'), (199, 'sr_ronex_msgs'), (200, 'media_export'), (201, 'geometry_msgs'), (202, 'interactive_marker_twist_server'), (203, 'missing_interface_pkg'), (204, 'rosh_common'), (205, 'sr_hand'), (206, 'rocon_launch'), (207, 'sr_example'), (208, 'test_roslaunch'), (209, 'urdf'), (210, 'resource_retriever'), (211, 'yujin_maps'), (212, 'rqt_graph'), (213, 'rotate_recovery'), (214, 'fake_localization'), (215, 'qt_gui'), (216, 'filters'), (217, 'ompl'), (218, 'gazebo2rviz'), (219, 'flir_ptu_description'), (220, 'sr_edc_muscle_tools'), (221, 'concert_scheduler_requests'), (222, 'perception_pcl'), (223, 'jsk_roseus'), (224, 'roscpp_serialization'), (225, 'ps3joy'), (226, 'wpi_jaco_wrapper'), (227, 'default_cfg_fkie'), (228, 'object_recognition_core'), (229, 'kobuki_driver'), (230, 'rqt_web'), (231, 'bond_core'), (232, 'kobuki_gazebo_plugins'), (233, 'mavros'), (234, 'object_recognition_msgs'), (235, 'ecl_mpl'), (236, 'shadow_robot'), (237, 'rqt_action'), (238, 'xacro'), (239, 'diagnostic_msgs'), (240, 'teleop_twist_joy'), (241, 'gripper_action_controller'), (242, 'rqt_robot_dashboard'), (243, 'pluginlib'), (244, 'rosdoc_lite'), (245, 'kobuki_testsuite'), (246, 'xmlrpcpp'), (247, 'dynamic_edt_3d'), (248, 'node_manager_fkie'), (249, 'bond'), (250, 'self_test'), (251, 'image_geometry'), (252, 'nao_msgs'), (253, 'rospack'), (254, 'quadrotor_handler'), (255, 'image_rotate'), (256, 'jsk_pr2eus'), (257, 'nao_driver'), (258, 'arbotix_firmware'), (259, 'hector_geotiff'), (260, 'rosserial_server'), (261, 'rviz_plugin_tutorials'), (262, 'tf2_msgs'), (263, 'camera_calibration'), (264, 'qt_gui_app'), (265, 'rostime'), (266, 'rviz'), (267, 'rocon_master_info'), (268, 'common_tutorials'), (269, 'rocon_device_msgs'), (270, 'driver_base'), (271, 'interaction_cursor_msgs'), (272, 'rocon_msgs'), (273, 'laser_filters'), (274, 'interaction_cursor_demo'), (275, 'concert_utilities'), (276, 'yujin_ocs'), (277, 'rocon_gateway'), (278, 'sr_gui_joint_slider'), (279, 'bride_templates'), (280, 'pepperl_fuchs_r2000'), (281, 'ecl'), (282, 'rosservice'), (283, 'sr_ronex_transmissions'), (284, 'rocon_ebnf'), (285, 'turtle_tf'), (286, 'razer_hydra'), (287, 'tf2_geometry_msgs'), (288, 'depthimage_to_laserscan'), (289, 'visualization_marker_tutorials'), (290, 'audio_play'), (291, 'geometry'), (292, 'pluginlib_tutorials'), (293, 'roslz4'), (294, 'rqt_dep'), (295, 'rosh_robot'), (296, 'rqt_robot_monitor'), (297, 'rosh_visualization'), (298, 'nav2d_navigator'), (299, 'no_provider_pkg'), (300, 'stereo_msgs'), (301, 'wpi_jaco_msgs'), (302, 'kobuki_capabilities'), (303, 'trajectory_msgs'), (304, 'roscpp_core'), (305, 'visp_tracker'), (306, 'ecl_errors'), (307, 'rocon_gateway_tests'), (308, 'libfreenect'), (309, 'nolangs'), (310, 'tf2'), (311, 'position_controllers'), (312, 'test_rosmaster'), (313, 'velodyne_msgs'), (314, 'chatter_concert'), (315, 'vrep_ros_bridge'), (316, 'rostest'), (317, 'ecl_time'), (318, 'drums_ros'), (319, 'concert_service_turtlesim'), (320, 'cpp_common'), (321, 'sound_play'), (322, 'quux_user'), (323, 'concert_conductor'), (324, 'demo_pioneer'), (325, 'kobuki_softapps'), (326, 'compressed_image_transport'), (327, 'zeroconf_msgs'), (328, 'concert_simple_scheduler'), (329, 'rocon'), (330, 'turtle_concert'), (331, 'ecl_mobile_robot'), (332, 'jpeg_streamer'), (333, 'qt_dotgraph'), (334, 'openni2_launch'), (335, 'rosbag_storage'), (336, 'ecl_navigation'), (337, 'ecl_utilities'), (338, 'hokuyo_node'), (339, 'b'), (340, 'tf2_web_republisher'), (341, 'nodelet'), (342, 'sr_gui_change_muscle_controllers'), (343, 'rqt_logger_level'), (344, 'ecl_concepts'), (345, 'badly_specified_changelog'), (346, 'openni_launch'), (347, 'sr_visualization_icons'), (348, 'ecto_openni'), (349, 'tf2_kdl'), (350, 'monocam_settler'), (351, 'slam_karto'), (352, 'collada_urdf'), (353, 'uuid_msgs'), (354, 'sr_tactile_sensors'), (355, 'rosserial'), (356, 'minimal_pkg'), (357, 'ecl_type_traits'), (358, 'theora_image_transport'), (359, 'tf2_tools'), (360, 'actionlib'), (361, 'kobuki_soft'), (362, 'concert_service_utilities'), (363, 'nao_pose'), (364, 'rocon_app_utilities'), (365, 'base_local_planner'), (366, '%(package)s'), (367, 'visp'), (368, 'roslisp'), (369, 'gazebo_msgs'), (370, 'visp_ros'), (371, 'rosserial_arduino'), (372, 'octovis'), (373, 'yocs_cmd_vel_mux'), (374, 'sr_gui_movement_recorder'), (375, 'clear_costmap_recovery'), (376, 'octomap_msgs'), (377, 'interaction_cursor_rviz'), (378, 'bride_tutorials'), (379, 'fcl'), (380, 'rqt_py_common'), (381, 'rocon_icons'), (382, 'compressed_depth_image_transport'), (383, 'rocon_test'), (384, 'hector_nav_msgs'), (385, 'ros_controllers'), (386, 'kobuki_dock_drive'), (387, 'rosserial_msgs'), (388, 'kobuki_keyop'), (389, 'urdf_parser_plugin'), (390, 'diagnostics'), (391, 'kobuki_rviz_launchers'), (392, 'orocos_kdl'), (393, 'sr_cyberglove_config'), (394, 'imu_handler'), (395, 'carrot_planner'), (396, 'roshlaunch'), (397, 'serial_utils'), (398, 'nodelet_topic_tools'), (399, 'nodelet_core'), (400, 'nodelet_tutorial_math'), (401, 'test_rosgraph'), (402, 'joystick_drivers'), (403, 'ecl_exceptions'), (404, 'laser_cb_detector'), (405, 'nav2d'), (406, 'cv_bridge'), (407, 'rosparam'), (408, 'geneus'), (409, 'image_pipeline'), (410, 'bfl'), (411, 'ecto_opencv'), (412, 'wfov_camera_msgs'), (413, 'roswtf'), (414, 'geometric_shapes'), (415, 'map_server'), (416, 'imu_pipeline'), (417, 'ecl_containers'), (418, 'test_nodelet'), (419, 'ecl_license'), (420, 'axis_camera'), (421, 'raw_description'), (422, 'smach_msgs'), (423, 'hector_slam_launch'), (424, 'sr_ethercat_hand_config'), (425, 'sr_gui_grasp_controller'), (426, 'visp_bridge'), (427, 'test_rosbag'), (428, 'rigid_body_handler'), (429, 'rqt_rviz'), (430, 'rosgraph_msgs'), (431, 'image_cb_detector'), (432, 'ecl_manipulators'), (433, 'kobuki_core'), (434, 'ecto_pcl'), (435, 'rqt_msg'), (436, 'nao_extras'), (437, 'ar_track_alvar'), (438, 'sr_kinematics'), (439, 'master_discovery_fkie'), (440, 'rosnode'), (441, 'rocon_app_manager'), (442, 'rocon_apps'), (443, 'ros_web_video'), (444, 'nmea_navsat_driver'), (445, 'cob_common'), (446, 'global_planner'), (447, 'depth_image_proc'), (448, 'libuvc_camera'), (449, 'nao_bringup'), (450, 'python_qt_binding'), (451, 'kobuki_bumper2pc'), (452, 'test_find_tinyxml'), (453, 'bride'), (454, 'rqt_gui_cpp'), (455, 'concert_master'), (456, 'roscpp_traits'), (457, 'rqt_capabilities'), (458, 'yocs_diff_drive_pose_controller'), (459, 'object_recognition_transparent_objects'), (460, 'multimaster_fkie'), (461, 'nao_path_follower'), (462, 'image_common'), (463, 'interaction_cursor_3d'), (464, 'joy'), (465, 'kobuki_controller_tutorial'), (466, 'navfn'), (467, 'kdl_conversions'), (468, 'cob_extern'), (469, 'actionlib_tutorials'), (470, 'nav2d_karto'), (471, 'map_msgs'), (472, 'urdfdom_py'), (473, 'python_ethernet_rmp'), (474, 'turtle_actionlib'), (475, 'common_msgs'), (476, 'kobuki_auto_docking'), (477, 'c'), (478, 'tf2_bullet'), (479, 'ecl_config'), (480, 'forward_command_controller'), (481, 'gateway_msgs'), (482, 'navigation_capability'), (483, 'sr_gui_self_test'), (484, 'tutorial_pioneer_node'), (485, 'ecl_converters'), (486, 'arbotix_sensors'), (487, 'turtlesim'), (488, 'effort_controllers'), (489, 'diagnostic_analysis'), (490, 'view_controller_msgs'), (491, 'camera1394'), (492, 'sr_robot_msgs'), (493, 'ecl_geometry'), (494, 'rosconsole_bridge'), (495, 'concert_service_teleop'), (496, 'sr_gui_bootloader'), (497, 'geographic_info'), (498, 'slam_gmapping'), (499, 'ros_ethernet_rmp'), (500, 'robot_pose_publisher'), (501, 'rqt_reconfigure'), (502, 'rosh_desktop_plugins'), (503, 'prosilica_camera'), (504, 'rocon_python_redis'), (505, 'freenect_stack'), (506, 'visp_auto_tracker'), (507, 'calibration_msgs'), (508, 'sentis_tof_m100'), (509, 'sr_ronex_external_protocol'), (510, 'rosauth'), (511, 'roseus_smach'), (512, 'tf2_py'), (513, 'message_runtime'), (514, 'std_msgs'), (515, 'concert_service_link_graph'), (516, 'sr_edc_launch'), (517, 'sr_robot_lib'), (518, 'concert_service_gazebo'), (519, 'test_nodelet_topic_tools'), (520, 'libg2o'), (521, 'bondcpp'), (522, 'rocon_hub'), (523, 'roslang'), (524, 'interactive_marker_proxy'), (525, 'yocs_waypoints_navi'), (526, 'kobuki_softnode'), (527, 'joint_limits_interface'), (528, 'joint_state_publisher'), (529, 'bride_plugin_source'), (530, 'stereo_image_proc'), (531, 'jaco_interaction'), (532, 'sr_config'), (533, 'kobuki_description'), (534, 'rviz_animated_view_controller'), (535, 'ecl_formatters'), (536, 'robot_upstart'), (537, 'class_loader'), (538, 'shadow_robot_ethercat'), (539, 'concert_msgs'), (540, 'ecl_sigslots_lite'), (541, 'unique_id'), (542, 'nao_teleop'), (543, 'sr_ronex'), (544, 'nav_msgs'), (545, 'costmap_2d'), (546, 'ecl_command_line'), (547, 'sr_ronex_drivers'), (548, 'rocon_app_manager_tutorials'), (549, 'threemxl'), (550, 'mavlink'), (551, 'jaco_sdk'), (552, 'tf2_ros'), (553, 'vrep_ros_plugin'), (554, 'librviz_tutorial'), (555, 'sr_ronex_controllers'), (556, 'octomap_rviz_plugins'), (557, 'ros_control'), (558, 'wpi_jaco'), (559, 'sr_mechanism_model'), (560, 'hector_compressed_map_transport'), (561, 'rosbridge_suite'), (562, 'ros_comm'), (563, 'eigen_conversions'), (564, 'test_diagnostic_aggregator'), (565, 'xdot'), (566, 'rosbridge_library'), (567, 'octomap_ros'), (568, 'ax2550'), (569, 'sr_ronex_test'), (570, 'battery_monitor_rmp'), (571, 'pkg_with_invalid_specs'), (572, 'tf_conversions'), (573, 'roslaunch'), (574, 'visualization_tutorials'), (575, 'laser_pipeline'), (576, 'navigation'), (577, 'kobuki_random_walker'), (578, 'openslam_gmapping'), (579, 'kdl_parser'), (580, 'imu_sensor_controller'), (581, 'sr_movements'), (582, 'rosserial_xbee'), (583, 'qt_build'), (584, 'rocon_interactions'), (585, 'carl_estop'), (586, 'hector_mapping'), (587, 'sr_self_test'), (588, 'ecl_statistics'), (589, 'moveit_python'), (590, 'kobuki_apps'), (591, 'catkin'), (592, 'hector_slam'), (593, 'image_view'), (594, 'rqt_py_console'), (595, 'test_osm'), (596, 'pr2eus'), (597, 'hector_imu_tools'), (598, 'cob_description'), (599, 'rocon_tutorial_msgs'), (600, 'object_recognition_ros'), (601, 'object_recognition_capture'), (602, 'controller_manager_tests'), (603, 'sr_ronex_hardware_interface'), (604, 'sr_ronex_examples'), (605, 'open_karto'), (606, 'rosh'), (607, 'concert_resource_pool'), (608, 'dwa_local_planner'), (609, 'smclib'), (610, 'ecl_devices'), (611, 'capabilities'), (612, 'rosbuild'), (613, 'qt_gui_cpp'), (614, 'rocon_hub_client'), (615, 'dynamic_reconfigure'), (616, 'differential_mobile_base_capability'), (617, 'force_sensor_handler'), (618, 'libpcan'), (619, 'urg_node'), (620, 'geodesy'), (621, 'genlisp'), (622, 'interactive_markers'), (623, 'robot_state_publisher'), (624, 'depthcloud_encoder'), (625, 'concert_service_msgs'), (626, 'laser_proc'), (627, 'ackermann_msgs'), (628, 'rqt_top'), (629, 'rosbridge_server'), (630, 'executive_smach'), (631, 'noproject'), (632, 'd'), (633, 'move_base'), (634, 'genmsg'), (635, 'control_toolbox'), (636, 'turtle_tf2'), (637, 'rocon_interaction_msgs'), (638, 'velodyne'), (639, 'rosserial_python'), (640, 'rosmaster'), (641, 'qt_create'), (642, 'python_test_a'), (643, 'transmission_interface'), (644, 'sicktoolbox_wrapper'), (645, 'ecl_converters_lite'), (646, 'rqt'), (647, 'catkin_test'), (648, 'rosserial_client'), (649, 'rocon_std_msgs'), (650, 'opencv_tests'), (651, 'sr_utilities'), (652, 'ntpd_driver'), (653, 'spacenav_node'), (654, 'laser_geometry'), (655, 'qt_tutorials'), (656, 'geometry_experimental'), (657, 'rqt_bag'), (658, 'sr_mechanism_controllers'), (659, 'graph_msgs'), (660, 'sr_gazebo_plugins'), (661, 'rosout'), (662, 'camera_calibration_parsers'), (663, 'camera_umd'), (664, 'libuvc_ros'), (665, 'nmea_msgs'), (666, 'sr_description'), (667, 'dummy_slam_broadcaster'), (668, 'rqt_ez_publisher'), (669, 'rqt_nav_view'), (670, 'roslint'), (671, 'ecl_time_lite'), (672, 'qt_ros'), (673, 'rosunit'), (674, 'pysdf'), (675, 'move_base_msgs'), (676, 'stage'), (677, 'moveit_core'), (678, 'um6'), (679, 'camera_info_manager'), (680, 'smach_viewer'), (681, 'flir_ptu_viz'), (682, 'nav2d_operator'), (683, 'ardrone_autonomy'), (684, 'calibration_launch'), (685, 'rocon_uri'), (686, 'controller_interface'), (687, 'yocs_math_toolkit'), (688, 'pcl_msgs'), (689, 'usb_cam'), (690, 'test_tf2'), (691, 'argos3d_p100'), (692, 'xsens_driver'), (693, 'rviz_python_tutorial'), (694, 'velodyne_height_map'), (695, 'ecl_tools'), (696, 'kobuki_desktop'), (697, 'image_exposure_msgs'), (698, 'velodyne_pointcloud'), (699, 'libnabo'), (700, 'rosbash'), (701, 'rqt_bag_plugins'), (702, 'scheduler_msgs'), (703, 'rqt_plot'), (704, 'arbotix_python'), (705, 'control_msgs'), (706, 'test_rospy'), (707, 'actionlib_msgs'), (708, 'kobuki_msgs'), (709, 'shared_serial'), (710, 'pocketsphinx'), (711, 'sr_external_dependencies'), (712, 'controller_manager_msgs'), (713, 'rqt_shell'), (714, 'openni_camera'), (715, 'sr_gui_controller_tuner'), (716, 'joint_state_controller'), (717, 'quadrotor_tk_handler'), (718, 'test_rosservice'), (719, 'audio_common_msgs'), (720, 'force_torque_sensor_controller'), (721, 'image_transport_plugins'), (722, 'std_srvs'), (723, 'rviz_fixed_view_controller'), (724, 'yocs_msgs'), (725, 'ros_ethercat_eml'), (726, 'sr_gui_change_controllers'), (727, 'octomap'), (728, 'amcl'), (729, 'ros_ethercat_loop'), (730, 'libphidgets'), (731, 'bondpy'), (732, 'humanoid_msgs'), (733, 'ros_tutorials'), (734, 'bride_compilers'), (735, 'genpy'), (736, 'libuvc'), (737, 'cmake_modules'), (738, 'nav_core'), (739, 'nao_description'), (740, 'openni2_camera'), (741, 'ipa_canopen_ros'), (742, 'kobuki_ftdi'), (743, 'test_roslib_comm'), (744, 'nav2d_remote'), (745, 'rocon_console'), (746, 'robot_pose_ekf'), (747, 'ros_user'), (748, 'graft'), (749, 'nav2d_exploration'), (750, 'quux_msgs'), (751, 'image_proc'), (752, 'roscpp'), (753, 'controller_manager'), (754, 'calibration'), (755, 'zeroconf_avahi'), (756, 'ecl_manipulation'), (757, 'rosh_core'), (758, 'velodyne_driver'), (759, 'kobuki_node'), (760, 'rosclean'), (761, 'nao_dashboard'), (762, 'lms1xx'), (763, 'kobuki'), (764, 'rqt_marble'), (765, 'rosmake'), (766, 'rocon_multimaster'), (767, 'pointgrey_camera_driver'), (768, 'message_filters'), (769, 'nav2d_localizer'), (770, 'concert_services'), (771, 'rosh_geometry'), (772, 'realtime_tools'), (773, 'rospy_tutorials'), (774, 'ros_ethercat'), (775, 'gazebo_plugins'), (776, 'rqt_gui_py'), (777, 'voxel_grid'), (778, 'serial'), (779, 'moveit_planners'), (780, 'grasping_msgs'), (781, 'ecto_ros'), (782, 'roseus'), (783, 'no_default_provider_pkg'), (784, 'gencpp'), (785, 'rocon_app_manager_msgs'), (786, 'rqt_controller_manager'), (787, 'hector_map_server'), (788, 'zeroconf_avahi_demos'), (789, 'rosbag')]
    # mapPkgDeps(pkg_ids, packages)


    git_names = dbe.getUnique(cur, 'Issues', ['created_by'])
    for name in git_names:
         print qg.getUserEmail(name[0])
