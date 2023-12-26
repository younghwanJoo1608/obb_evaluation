void WaitTcws(const ros::NodeHandle& nh,
              const ObbParam& param,
                const std::vector<int>& cameras,
                ros::Rate& rate,
                EigenMap<int, g2o::SE3Quat >& Tcws,
                EigenMap<int, g2o::SE3Quat >& Tcps
                ){
    int ntry = 0;
    while(!ros::isShuttingDown()){
        tf::TransformListener listener;
        for(int i : cameras){
        std::string rgb_tf_name;
        std::string depth_tf_name;
    
        if (param.sensor_model == ObbParam::SENSORMODEL::K4A)
        {
            // rgb_tf_name = "/cam" + std::to_string(i) + "_rgb_camera_link";
            // depth_tf_name = "/cam" + std::to_string(i) + "_depth_camera_link";
            rgb_tf_name = "/cam" + std::to_string(i) + "_rgb_camera_link";
            depth_tf_name = "/cam" + std::to_string(i) + "_depth_camera_link";
        }
        else
        {
            rgb_tf_name = "/optical_frame_4104429722";
            depth_tf_name = "/robot";
        }

        tf::StampedTransform base_to_rgb;

        g2o::SE3Quat base_T_rgb;
        try{
            listener.waitForTransform("/robot", rgb_tf_name, ros::Time(0), ros::Duration(0.5));
            listener.lookupTransform("/robot", rgb_tf_name,  
                                    ros::Time(0), base_to_rgb);

            Eigen::Vector3d translation;
            tf::vectorTFToEigen(base_to_rgb.getOrigin(), translation);
            Eigen::Quaterniond rotation;
            tf::quaternionTFToEigen(base_to_rgb.getRotation(), rotation);
            base_T_rgb = g2o::SE3Quat(rotation, translation);
        }
        catch (tf::TransformException ex){
            ROS_WARN("No tf from /robot to %s", rgb_tf_name.c_str());
            break;
        }

        tf::StampedTransform depth_to_rgb;

        g2o::SE3Quat depth_T_rgb;
        try
        {
            listener.waitForTransform(depth_tf_name, rgb_tf_name, ros::Time(0), ros::Duration(0.5));
            listener.lookupTransform(depth_tf_name, rgb_tf_name,
                                    ros::Time(0), depth_to_rgb);

            Eigen::Vector3d translation;
            tf::vectorTFToEigen(depth_to_rgb.getOrigin(), translation);
            Eigen::Quaterniond rotation;
            tf::quaternionTFToEigen(depth_to_rgb.getRotation(), rotation);
            depth_T_rgb = g2o::SE3Quat(rotation, translation);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("No tf from %s, to %s", depth_tf_name.c_str(), rgb_tf_name.c_str());
            break;
        }

        if(param.sensor_model == ObbParam::SENSORMODEL::K4A){
            // Hardcoded from ../../unloader_calib/config/calib_info_for_tf.yaml

            Eigen::Quaterniond q(1.,0.,0.,0.);
            Eigen::Vector3d t(0.0, 0.0, 0.0);
            // std::cout << depth_T_rgb.to_homogeneous_matrix() << std::endl;
            Tcps[i] = g2o::SE3Quat(q, t); // Identity for k4a.
            Tcws[i] = base_T_rgb.inverse();
            // TODO Tcps[i] : Transformation "camera (i.e. k4a rgb)" <- "Points (i.e. k4a depth)"
            // TODO Tcws[i] : Transformation "camera (i.e. k4a rgb)" <- "World (i.e. base)"
        }
        else {
            Tcps[i] = depth_T_rgb.inverse(); // Identity for k4a.
            Tcws[i] = base_T_rgb.inverse();
        }
        }
        if(Tcws.size() == cameras.size() && Tcps.size() == cameras.size() )
        break;
        rate.sleep();
        ros::spinOnce();
    }

    return;
}
