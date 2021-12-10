XmlRpc::XmlRpcValue pos_list1;
        nh.getParam("/aruco_lookup_locations/target_1", pos_list1);
        ROS_ASSERT(pos_list1.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int32_t i = 0; i < pos_list1.size(); ++i)
        {
            ROS_ASSERT(pos_list1[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            a.at(i) = static_cast<double>(pos_list1[i]);
        }