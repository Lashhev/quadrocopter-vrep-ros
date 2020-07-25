local Quaternion = require('quaternion')
local RosInterface = {}
RosInterface.__index = RosInterface

function RosInterface:new(frame_id, topic_prefix, is_stereo, is_lidar)
    local self = setmetatable({}, RosInterface)
    local moduleName=0
    local index=0
    local rosInterfacePresent=false
    while moduleName do
        moduleName=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            rosInterfacePresent=true
        end
        index=index+1
    end
    self.is_stereo = is_stereo
    self.is_lidar = is_lidar
    -- if rosInterfacePresent == true then
    self.local_pose_publisher = simROS.advertise(frame_id .. "/Local_pose", "geometry_msgs/PoseStamped", 10)
    self.odometry_publisher = simROS.advertise(frame_id .. "/odometry", "nav_msgs/Odometry", 10)
    if self.is_stereo then
      self.left_image_publisher = simROS.advertise(topic_prefix .. "/left/image_raw", "sensor_msgs/Image", 10)
      self.right_image_publisher = simROS.advertise(topic_prefix .. "/right/image_raw", "sensor_msgs/Image", 10)
      self.right_camera_info_publisher = simROS.advertise(topic_prefix .. "/right/camera_info", "sensor_msgs/CameraInfo", 10)
      self.left_camera_info_publisher = simROS.advertise(topic_prefix .. "/left/camera_info", "sensor_msgs/CameraInfo", 10)
      self.left_depth_publisher = simROS.advertise(topic_prefix .. "/left/depth", "sensor_msgs/Image", 10)
      self.right_depth_publisher = simROS.advertise(topic_prefix .. "/right/depth", "sensor_msgs/Image", 10)
      simROS.publisherTreatUInt8ArrayAsString(self.left_image_publisher)
      simROS.publisherTreatUInt8ArrayAsString(self.right_image_publisher)
      simROS.publisherTreatUInt8ArrayAsString(self.left_depth_publisher)
      simROS.publisherTreatUInt8ArrayAsString(self.right_depth_publisher)
      self.left_image_msg = {}
      self.right_image_msg = {}
      self.tf_msg = {}
      self.left_camera_info_msg = {}
      self.right_camera_info_msg = {}
      self.left_depth_msg = {}
      self.right_depth_msg = {}
    end
    self.right_imu_publisher = simROS.advertise(topic_prefix .. "/Imu", "sensor_msgs/Imu", 10)
    if self.is_lidar then
      self.lidar_publisher = simROS.advertise(topic_prefix .. "/Lidar", "sensor_msgs/PointCloud", 10)
      simROS.publisherTreatUInt8ArrayAsString(self.lidar_publisher)
      self.lidar_msg = {}
    end
    self.local_pos_msg = {}
    self.odometry_msg = {}
    self.imu_msg = {}
    self.seq = 0
    self.frame_id = frame_id
    return self
  end

function RosInterface:getHeaderMsg(frame_id)
    return {stamp = sim.getSimulationTime(), frame_id = frame_id, seq = self.seq}
end

function RosInterface:getLocalPosMsg(position_vector, orientation_vector)
    local header = self:getHeaderMsg(self.frame_id)
    local position = {x=position_vector[1], y=position_vector[2], z=position_vector[3]}
    local orientation = Quaternion:new(orientation_vector[3], orientation_vector[2], orientation_vector[1])
    self.local_pos_msg = {header = header, pose = {position = position, orientation = orientation}}
end

function RosInterface:getOdometryMsg(position_vector, orientation_vector)
  local header = self:getHeaderMsg(self.frame_id)
  local position = {x=position_vector[1], y=position_vector[2], z=position_vector[3]}
  local orientation = Quaternion:new(orientation_vector[3], orientation_vector[2], orientation_vector[1])
  local pose1 = {position = position, orientation = orientation}
  local pose2 = {}
  pose2["pose"] = pose1
  self.odometry_msg = {header = header, pose = pose2, twist = {}}
end

function RosInterface:getTransformStamped(objHandle,name,relTo,relToName)
  p=sim.getObjectPosition(objHandle,relTo)
  o=sim.getObjectQuaternion(objHandle,relTo)
  return {
      header={
          stamp=t,
          frame_id=relToName
      },
      child_frame_id=name,
      transform={
          translation={x=p[1],y=p[2],z=p[3]},
          rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
      }
  }
end

function RosInterface:getImageMsg(vision_sensor)
  local header = self:getHeaderMsg(vision_sensor.name)
  local buffer, resolution_x, resolution_y = vision_sensor:getSensorImage(1)
  sim.transformImage(buffer,{resolution_x,resolution_y},4)
  self.left_image_msg = {header = header, data = buffer, height = resolution_y, width = resolution_x, encoding = "rgb8", is_bigendian = 1, step = resolution_x*1*3}
  header = self:getHeaderMsg(vision_sensor.name)
  buffer, resolution_x, resolution_y = vision_sensor:getSensorImage(2)
  sim.transformImage(buffer,{resolution_x,resolution_y},4)
  self.right_image_msg = {header = header, data = buffer, height = resolution_y, width = resolution_x, encoding = "rgb8", is_bigendian = 1, step = resolution_x*1*3}
end

function RosInterface:getCameraInfoMsg(vision_sensor)
  local header = self:getHeaderMsg(vision_sensor.name)
  self.left_camera_info_msg = self:fillCameraInfo(vision_sensor, header, 1)
  header = self:getHeaderMsg(vision_sensor.name)
  self.right_camera_info_msg = self:fillCameraInfo(vision_sensor, header, 2)
end

function RosInterface:getDepthMsg(vision_sensor)
  self.left_depth_msg = self:fillDepthMsg(vision_sensor, 1)
  self.right_depth_msg = self:fillDepthMsg(vision_sensor, 2)
end

function RosInterface:fillDepthMsg(vision_sensor, number)
  msg = {}
  msg["header"] = self:getHeaderMsg(vision_sensor.name)
  local resolution = vision_sensor:getSensorResolution(number)
  local result, near_clipping = vision_sensor:getNearClipping(number)
  local result, far_clipping = vision_sensor:getFarClipping(number)
  local buffer = vision_sensor:getDepthBuffer(number)
  --msg["data"], msg["width"], msg["height"] = vision_sensor:getSensorImage(number)
  buffer = sim.transformBuffer(buffer, sim.buffer_float, 1, 0, sim.buffer_uint8)
  msg["data"] = sim.transformBuffer(buffer, sim.buffer_float, far_clipping - near_clipping, near_clipping, sim.buffer_float)
  msg["width"] = resolution[1]
  msg["height"] = resolution[2]
  msg["is_bigendian"] = 1
  msg["encoding"] = "16UC1"
  msg["step"] = msg["width"] * 2
  return msg
end

function RosInterface:getPointCloudMsg(data)
  self.lidar_msg = self:fillPointCloudMsg(data)
end

function RosInterface:fillPointCloudMsg(data)
  pub_data={}
  pub_data['header']=self:getHeaderMsg('velodyneVPL')
  pub_data['points']=data
  return pub_data
end

function RosInterface:fillCameraInfo(vision_sensor, header, number)
  local info = {}
  local result, resolution, view_angle = vision_sensor:getSensorInfo(number)
  if result == 1 then
    info['width'] = resolution[1]
    info['height'] = resolution[2]
    local f_x = (info['width']/2) / math.tan(view_angle/2)
    local f_y = f_x
    info['D'] = {0, 0, 0, 0, 0}
    info['K'] = {f_x, 0, info['width']/2, 0, f_y, info['height']/2, 0, 0, 1}
    info['R'] = {1, 0, 0, 0, 1, 0, 0, 0, 1}
    info['P'] = {info['K'][1], 0, info['K'][3], 0, 0, info['K'][5], info['K'][6], 0, 0, 0, 1, 0}
    info['header'] = header
    info['distortion_model'] = "plumb_bob"
    info['binning_x'] = 1
    info['binning_y'] = 1
    info['roi']  = {x_offset = 0, y_offset = 0, width = 0, height = 0, do_rectify = false}
    return info
    -- return {header = header, width = width, height = height, distortion_model = "plumb_bob", D = D, K = K, R = R, P = P, binning_x = 0, binning_y = 0, roi = {x_offset = 0, y_offset = 0, width = 0, height = 0, do_rectify = 0}}
  else
    print("Could get CameraInfo data!!!")
    return nil
  end
end

function RosInterface:getImuMsg(orientation_vector, linear_accel, angular_velocity_vector)
    local header = self:getHeaderMsg(self.frame_id)
    local orientation = Quaternion:new(orientation_vector[3], orientation_vector[2], orientation_vector[1])
    local linear_acceleration = {x=linear_accel[1], y=linear_accel[2], z=linear_accel[2]}
    local angular_velocity = {x=angular_velocity_vector[1], y=angular_velocity_vector[2], z=angular_velocity_vector[2]}
    self.imu_msg = {header = header, angular_velocity = angular_velocity, orientation = orientation, linear_acceleration=linear_acceleration}

end

function RosInterface:publish()
    simROS.publish(self.local_pose_publisher, self.local_pos_msg)
    simROS.publish(self.odometry_publisher, self.odometry_msg)
    self.seq = self.seq + 1;
    if(self.is_stereo) then
      simROS.publish(self.left_image_publisher, self.left_image_msg)
      simROS.publish(self.right_image_publisher, self.right_image_msg)
      simROS.publish(self.right_camera_info_publisher, self.right_camera_info_msg)
      simROS.publish(self.left_camera_info_publisher, self.left_camera_info_msg)
      simROS.publish(self.left_depth_publisher, self.left_depth_msg)
      simROS.publish(self.right_depth_publisher, self.right_depth_msg)
    end
    if(self.is_lidar) then
      simROS.publish(self.lidar_publisher,self.lidar_msg)
    end
end

function RosInterface:sendTransform(objHandle,name,relTo,relToName)
    simROS.sendTransform(self:getTransformStamped(objHandle,name,relTo,relToName))
end
function RosInterface:clearup()
    simROS.shutdownPublisher(self.local_pose_publisher)
    simROS.shutdownPublisher(self.odometry_publisher)
    if(self.is_stereo) then
      simROS.shutdownPublisher(self.left_image_publisher)
      simROS.shutdownPublisher(self.right_image_publisher)
      simROS.shutdownPublisher(self.right_camera_info_publisher)
      simROS.shutdownPublisher(self.left_camera_info_publisher)
      simROS.shutdownPublisher(self.left_depth_publisher)
      simROS.shutdownPublisher(self.right_depth_publisher)
    end
    if(self.is_lidar) then
      simROS.shutdownPublisher(self.lidar_publisher)
    end
  end
return RosInterface