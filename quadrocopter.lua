local StereoCamera = require('stereo')
local RosInterface = require('ros_interface')
local Velodyne = require('velodyne')
local Vector = require('vector')

local Quadrocopter = {}
Quadrocopter.__index = Quadrocopter

function Quadrocopter:new(is_stereo, is_lidar)
    local self = setmetatable({}, Quadrocopter)
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    local v = sim.getInt32Parameter(sim.intparam_program_version)
    if (v < 20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end
    self.is_stereo = is_stereo
    self.is_lidar = is_lidar
    if (is_stereo) then
      self.stereo = StereoCamera:new()
    end
    if (is_lidar) then
      self.lidar = Velodyne:new()
    end
    self.ros_interface = RosInterface:new('Quadricopter_base', 'stereo', is_stereo, is_lidar)
    self.targetObj = sim.getObjectHandle('Quadricopter_target')
    if self.stereo == nil and self.ros_interface == nil and self.lidar == nil then
      print("Failed to initiate interfaces")
      return nil
    end
    sim.setObjectParent(self.targetObj,-1,true)
    --print(self.targetObj)
    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example
    self.d=sim.getObjectHandle('Quadricopter_base')
    self.targetObj_loc=sim.getObjectHandle('Quadricopter_target_loc')

    self.particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(self.particlesAreVisible))
    self.simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(self.simulateParticles))
    --print(test.x)
    self.propellerScripts = {-1,-1,-1,-1}
    for i=1,4,1 do
      self.propellerScripts[i] = sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    self.heli = sim.getObjectAssociatedWithScript(sim.handle_self)
    
    self.obsticle_handlers ={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, -1}
    for i=1,12,1 do
      self.obsticle_handlers[i] =sim.getObjectHandle('240cmHighPillar50cm'..i)
    end
    self.particlesTargetVelocities = {0,0,0,0}

    self.pParam=2
    self.iParam=0
    self.dParam=0
    self.vParam=-6

    self.cumul=0
    self.lastE=0
    self.pAlphaE=0
    self.pBetaE=0
    self.psp2=0
    self.psp1=0

    self.prevEuler=0
    
    self.nextPoint = Vector.new(0,0,0)
    self.targetPos_loc = sim.getObjectPosition(self.targetObj_loc, -1)
    self.K_Fo = 3
    self.K_t = 0.5
    self.min_dist = 1.5
    self.distances_to_obst = {0,0,0,0,0,0,0,0,0,0,0,0}

    self.shadowCont = {}
    self.fakeShadow = sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (self.fakeShadow) then
      self.shadowCont=sim.addDrawingObject(sim.drawing_discpoints + sim.drawing_cyclic + sim.drawing_25percenttransparency + sim.drawing_50percenttransparency + sim.drawing_itemsizes, 0.2, 0, -1, 1)
    end

    -- Prepare 2 floating views with the camera views:
    self.floorCam=sim.getObjectHandle('Quadricopter_floorCamera')
    self.frontCam=sim.getObjectHandle('Quadricopter_frontCamera')
    self.floorView=sim.floatingViewAdd(0.9, 0.9, 0.2, 0.2, 0)
    self.frontView=sim.floatingViewAdd(0.7, 0.9, 0.2, 0.2, 0)
    sim.adjustView(self.floorView, self.floorCam, 64)
    sim.adjustView(self.frontView, self.frontCam, 64)
    return self
  end

function Quadrocopter:fillLocalPoseMsg(position_vector, orientation_vector)
  self.ros_interface:getLocalPosMsg(position_vector, orientation_vector)
end

function Quadrocopter:fillOdometryMsg(position_vector, orientation_vector)
  self.ros_interface:getOdometryMsg(position_vector, orientation_vector)
end

function Quadrocopter:fillPointCloudMsg(data)
  self.ros_interface:getPointCloudMsg(data)
end

function Quadrocopter:fillImageMsg()
  self.ros_interface:getImageMsg(self.stereo)
end

function Quadrocopter:fillCameraInfoMsg()
  self.ros_interface:getCameraInfoMsg(self.stereo)
end
function Quadrocopter:fillDepthMsg()
  self.ros_interface:getDepthMsg(self.stereo)
end

function Quadrocopter:fillImuMsg(orientation_vector, linear_accel, angular_velocity_vector)
  self.ros_interface:getImuMsg(orientation_vector, linear_accel, angular_velocity_vector)
end

function Quadrocopter:getTransformStamped()
  local p = sim.getObjectPosition(self.stereo.vision_handler, self.heli)
  local o = sim.getObjectOrientation(self.stereo.vision_handler, self.heli)
  self.ros_interface:getTransformStamped(p, o,self.stereo.name,"world")
end

function Quadrocopter:publish()
  self.ros_interface:publish()
  if (self.is_lidar) then
    self.ros_interface:sendTransform(self.lidar.visionSensorHandles[1],'velodyneVPL',-1,'world')
  end
  if (self.is_stereo) then
    self.ros_interface:sendTransform(self.stereo.vision_sensor_handlers[1],self.stereo.name,-1,'world')
  end
  
end

function Quadrocopter:update()
  local position = {}
  local thrust = 0
  local alphaCorr = 0
  local betaCorr = 0
  local rotCorr = 0
  local orientation = 0
  self:update_next_point()
  -- Vertical control:
  position, thrust = self:vertical_control()
  -- Horizontal control:
  alphaCorr, betaCorr = self:horizontal_control()
  -- Rotational control:
  rotCorr, orientation = self:rotation_control()
  
  --Fill ROS msgs and publish them
  local linear_velocity, angular_velocity = sim.getObjectVelocity(self.d)
  self:fillLocalPoseMsg(position, orientation)
  self:fillOdometryMsg(position, orientation)
  if (self.is_stereo) then
    self:fillImageMsg()
    self:fillCameraInfoMsg()
    self:fillDepthMsg()
  end
  self:fillImuMsg(orientation, angular_velocity, angular_velocity)
  if (self.is_lidar) then
    local lidar_data = self.lidar:getRawData()
    self:fillPointCloudMsg(lidar_data)
  end
  self:publish()
  -- Decide of the motor velocities:
  self:update_speed(thrust, alphaCorr, betaCorr, rotCorr)
  self:updateFakeShadow(position)
end

function Quadrocopter:clearup()
  self.ros_interface:clearup()
  if (self.is_lidar) then
    self.lidar:clearup()
  end
  sim.removeDrawingObject(self.shadowCont)
  sim.floatingViewRemove(self.floorView)
  sim.floatingViewRemove(self.frontView)
end

function Quadrocopter:vertical_control()
  local targetPos = sim.getObjectPosition(self.targetObj_loc, -1)
  local pos = sim.getObjectPosition(self.d,-1)
  local l = sim.getVelocity(self.heli)
  local e=(targetPos[3]-pos[3])
  self.cumul=self.cumul+e
  local pv=self.pParam*e
  local thrust=5.335+pv+self.iParam*self.cumul+self.dParam*(e-self.lastE)+l[3]*self.vParam
  self.lastE=e
  return pos, thrust
end

function Quadrocopter:horizontal_control()
  -- Horizontal control:
  local sp = sim.getObjectPosition(self.targetObj_loc,self.d)
  local m = sim.getObjectMatrix(self.d,-1)
  local vx = {1,0,0}
  vx = sim.multiplyVector(m,vx)
  local vy = {0,1,0}
  vy = sim.multiplyVector(m,vy)
  local alphaE = (vy[3] - m[12])
  local alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - self.pAlphaE)
  local betaE=(vx[3] - m[12])
  local betaCorr = -0.25*betaE-2.1*(betaE-self.pBetaE)
  self.pAlphaE = alphaE
  self.pBetaE = betaE
  alphaCorr = alphaCorr+sp[2]*0.005+1*(sp[2]-self.psp2)
  betaCorr = betaCorr-sp[1]*0.005-1*(sp[1]-self.psp1)
  self.psp2=sp[2]
  self.psp1=sp[1]
  return alphaCorr, betaCorr
end

function Quadrocopter:rotation_control()
  local euler = sim.getObjectOrientation(self.d, self.targetObj_loc)
  local orientation = sim.getObjectOrientation(self.d, -1)
  local rotCorr = euler[3] * 0.15 + 2 * (euler[3]-self.prevEuler)
  self.prevEuler=euler[3]
  return rotCorr, orientation
end

function Quadrocopter:update_speed(thrust, alphaCorr, betaCorr, rotCorr)
  self.particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
  self.particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
  self.particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
  self.particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)
  -- Send the desired motor velocities to the 4 rotors:
  for i=1,4,1 do
    sim.setScriptSimulationParameter(self.propellerScripts[i],'particleVelocity',self.particlesTargetVelocities[i])
  end
end

function Quadrocopter:updateFakeShadow(position)
  s = sim.getObjectSizeFactor(self.d)
  if (self.fakeShadow) then
    local itemData = {position[1],position[2],0.002,0,0,1,0.2*s}
    sim.addDrawingObjectItem(self.shadowCont, itemData)
  end
end

function Quadrocopter:update_next_point()
  local Fres = self:get_resist_forces()
  -- local Fres = Vector.new(0,0,0)
  -- print("Fres=")
  -- print(Fres)
  local targetPos = sim.getObjectPosition(self.targetObj,-1)
  local targetPos_loc = sim.getObjectPosition(self.targetObj_loc,-1)
  local Pos = sim.getObjectPosition(self.d,-1)

  local main_target_vec = self:sub_arrays(targetPos, Pos)
  local local_target_vec = self:sub_arrays(targetPos, targetPos_loc)
  local d_to_loc_targ_vec = self:sub_arrays(targetPos_loc, Pos)
  local distance_to_targ = main_target_vec:length()
  local distance_loc = d_to_loc_targ_vec:length()

  if (distance_to_targ > 0.1 and distance_loc < 0.2) then
  -- if (distance_to_targ > 0.06) then  
    local relPose_v = (main_target_vec*self.K_t + Fres):normalized()
    local tPos_loc = Vector.new(self.targetPos_loc[1], self.targetPos_loc[2], self.targetPos_loc[3])  + relPose_v*0.01
    local targetPos_loc_past = self.targetPos_loc
    self.targetPos_loc = {tPos_loc.x, tPos_loc.y, tPos_loc.z}
    sim.setObjectPosition(self.targetObj_loc,-1, self.targetPos_loc)
    -- local z_angle = Vector.direction(Vector.new(0, 0, 0), relPose_v) 
    -- local orientation_loc = {0, 0, z_angle}
    -- sim.setObjectOrientation(self.targetObj_loc,-1, orientation_loc)
  end
end

function Quadrocopter:get_distances_to_obst()
  for i=1,12,1 do
    self.obsPos = sim.getObjectPosition(self.obsticle_handlers[i], -1)
    self.Pos = sim.getObjectPosition(self.d, -1)
    self.obsPos_loc = self:sub_arrays(self.obsPos, self.Pos)
    self.distances_to_obst[i] = Vector.new(self.obsPos_loc.x, self.obsPos_loc.y, 0)
    if(self.distances_to_obst[i]:length() > self.min_dist) then
      self.distances_to_obst[i]=Vector.new(0,0,0)
    end
  end
end

function Quadrocopter:get_resist_forces()
  self:get_distances_to_obst()
  forces = Vector.new(0,0,0)
  for i=1,12,1 do
    beta = 0.1
    force= Vector.new(0,0,0)
    if (self.distances_to_obst[i] == Vector.new(0,0,0)) then
    else
      force = self:calculate_res_force(i, self.K_Fo, beta)
    end

    forces = forces + force
    -- print("forces = ")
    if i ==6 then
    print("force["..i.."] = ")
    print(force)
    end
    -- print(forces)
  end
  return forces
end

function Quadrocopter:sub_arrays(lh, rh)
  local left = Vector.new(lh[1], lh[2],lh[3])
  local right = Vector.new(rh[1], rh[2],rh[3])
  local result = left - right
  x, y, z = result:unpack()
  return result
end

function Quadrocopter:add_arrays(lh, rh)
  local left = Vector.new(lh[1], lh[2],lh[3])
  local right = Vector.new(rh[1], rh[2],rh[3])
  local result = left + right
  x, y, z = result:unpack()
  return result
end

function Quadrocopter:calculate_pos_force(moving_object, target, k)
    vector_pos = target - moving_object
    pos_force = k*vector_pos*(1/vector_pos:length())
    return pos_force
end

function Quadrocopter:calculate_res_force(i, c, beta)
    q = self.distances_to_obst[i]:length()
    alfa = math.atan2(self.distances_to_obst[i].y, self.distances_to_obst[i].x)
    res_force = -c*math.exp(-beta*q)*Vector.new(math.cos(alfa), math.sin(alfa), 0)
    return res_force
end
return Quadrocopter

