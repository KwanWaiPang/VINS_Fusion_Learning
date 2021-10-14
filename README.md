Self learning VINS-FUSION  (https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
adding EVIO fusion  
运行是需要在后面加入参数rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml   
rosrun vins vins_node ~/catkin_ws_dvs/src/VINS_Fusion_Learning/config/davis_346/davis_346_imu_config.yaml   

1. 加入Arc*包
2. feature_tracker.ccp先做改进
3. rosNodeTest.cpp加入该加入的依赖以及topic
4. estimator.cpp






# 运行命令： #
1. roslaunch dvs_renderer davis_testing.launch
2. 
# rostopic list #
/camera_info
/camera_info_num
/davis/camera_info
/davis_ros_driver/parameter_descriptions
/davis_ros_driver/parameter_updates
/dvs/calibrate_imu
/dvs/events
/dvs/exposure
/dvs/image_raw
/dvs/imu
/dvs/reset_timestamps
/dvs/trigger_snapshot
/dvs_accumulated_events
/dvs_accumulated_events/compressed
/dvs_accumulated_events/compressed/parameter_descriptions
/dvs_accumulated_events/compressed/parameter_updates
/dvs_accumulated_events/compressedDepth
/dvs_accumulated_events/compressedDepth/parameter_descriptions
/dvs_accumulated_events/compressedDepth/parameter_updates
/dvs_accumulated_events/theora
/dvs_accumulated_events/theora/parameter_descriptions
/dvs_accumulated_events/theora/parameter_updates
/dvs_accumulated_events_edges
/dvs_accumulated_events_edges/compressed
/dvs_accumulated_events_edges/compressed/parameter_descriptions
/dvs_accumulated_events_edges/compressed/parameter_updates
/dvs_accumulated_events_edges/compressedDepth
/dvs_accumulated_events_edges/compressedDepth/parameter_descriptions
/dvs_accumulated_events_edges/compressedDepth/parameter_updates
/dvs_accumulated_events_edges/theora
/dvs_accumulated_events_edges/theora/parameter_descriptions
/dvs_accumulated_events_edges/theora/parameter_updates
/dvs_rendering
/dvs_rendering/compressed
/dvs_rendering/compressed/parameter_descriptions
/dvs_rendering/compressed/parameter_updates
/dvs_rendering/compressedDepth
/dvs_rendering/compressedDepth/parameter_descriptions
/dvs_rendering/compressedDepth/parameter_updates
/dvs_rendering/theora
/dvs_rendering/theora/parameter_descriptions
/dvs_rendering/theora/parameter_updates
/dvs_undistorted
/dvs_undistorted/compressed
/dvs_undistorted/compressed/parameter_descriptions
/dvs_undistorted/compressed/parameter_updates
/dvs_undistorted/compressedDepth
/dvs_undistorted/compressedDepth/parameter_descriptions
/dvs_undistorted/compressedDepth/parameter_updates
/dvs_undistorted/theora
/dvs_undistorted/theora/parameter_descriptions
/dvs_undistorted/theora/parameter_updates
/events_off_mean_1
/events_off_mean_5
/events_on_mean_1
/events_on_mean_5
/image
/rosout
/rosout_agg
