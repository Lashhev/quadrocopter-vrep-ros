local StereoCamera = require('stereo')
local RosInterface = require('ros_interface')

local Vehicle = {}
Vehicle.__index = Vehicle

function Vehicle:new()
    local self = setmetatable({}, Vehicle)
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    local v = sim.getInt32Parameter(sim.intparam_program_version)
    if (v < 20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end
    self.stereo = StereoCamera:new()
    self.ros_interface = RosInterface:new('Quadricopter_base', 'stereo')
    self.targetObj = sim.getObjectHandle('Quadricopter_target')
    if self.stereo == nil and self.ros_interface == nil then
      print("Failed to initiate interfaces")
      return nil
    end
    sim.setObjectParent(self.targetObj,-1,true)
    --print(self.targetObj)
    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example
    self.d=sim.getObjectHandle('Quadricopter_base')
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

    self.particlesTargetVelocities = {0,0,0,0}

    self.pParam=2
    self.iParam=0
    self.dParam=0
    self.vParam=-2

    self.cumul=0
    self.lastE=0
    self.pAlphaE=0
    self.pBetaE=0
    self.psp2=0
    self.psp1=0

    self.prevEuler=0

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

function Vehicle:fillLocalPoseMsg(position_vector, orientation_vector)
  self.ros_interface:getLocalPosMsg(position_vector, orientation_vector)
end

function Vehicle:fillImageMsg()
  self.ros_interface:getImageMsg(self.stereo)
end

function Vehicle:fillCameraInfoMsg()
  self.ros_interface:getCameraInfoMsg(self.stereo)
end
function Vehicle:fillDepthMsg()
  self.ros_interface:getDepthMsg(self.stereo)
end

function Vehicle:getTransformStamped()
  local p = sim.getObjectPosition(self.stereo.vision_handler, self.heli)
  local o = sim.getObjectOrientation(self.stereo.vision_handler, self.heli)
  self.ros_interface:getTransformStamped(p, o,self.stereo.name,"world")
end

function Vehicle:publish()
  self.ros_interface:publish()
end

function Vehicle:update()
  local position = {}
  local thrust = 0
  local alphaCorr = 0
  local betaCorr = 0
  local rotCorr = 0
  local orientation = 0
  -- Vertical control:
  position, thrust = self:vertical_control()
  -- Horizontal control:
  alphaCorr, betaCorr = self:horizontal_control()
  -- Rotational control:
  rotCorr, orientation = self:rotation_control()
  --Fill ROS msgs and publish them
  self:fillLocalPoseMsg(position, orientation)
  self:fillImageMsg()
  self:fillCameraInfoMsg()
  self:fillDepthMsg()
  self:getTransformStamped()
  self:publish()
  -- Decide of the motor velocities:
  self:update_speed(thrust, alphaCorr, betaCorr, rotCorr)
  self:updateFakeShadow(position)
end

function Vehicle:clearup()
  self.ros_interface:clearup()
  sim.removeDrawingObject(self.shadowCont)
  sim.floatingViewRemove(self.floorView)
  sim.floatingViewRemove(self.frontView)
end

function Vehicle:vertical_control()
  --print(self.d)
  targetPos = sim.getObjectPosition(self.targetObj, -1)
  local pos = sim.getObjectPosition(self.d,-1)
  local l = sim.getVelocity(self.heli)
  local e=(targetPos[3]-pos[3])
  self.cumul=self.cumul+e
  local pv=self.pParam*e
  local thrust=5.335+pv+self.iParam*self.cumul+self.dParam*(e-self.lastE)+l[3]*self.vParam
  self.lastE=e
  return pos, thrust
end

function Vehicle:horizontal_control()
  -- Horizontal control:
  --print(self.d)
  local sp = sim.getObjectPosition(self.targetObj,self.d)
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

function Vehicle:rotation_control()
  local euler = sim.getObjectOrientation(self.d, self.targetObj)
  local orientation = sim.getObjectOrientation(self.d, -1)
  local rotCorr = euler[3] * 0.1 + 2 * (euler[3]-self.prevEuler)
  self.prevEuler=euler[3]
  return rotCorr, orientation
end

function Vehicle:update_speed(thrust, alphaCorr, betaCorr, rotCorr)
  self.particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
  self.particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
  self.particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
  self.particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)
  -- Send the desired motor velocities to the 4 rotors:
  for i=1,4,1 do
    sim.setScriptSimulationParameter(self.propellerScripts[i],'particleVelocity',self.particlesTargetVelocities[i])
  end
end

function Vehicle:updateFakeShadow(position)
  s = sim.getObjectSizeFactor(self.d)
  if (self.fakeShadow) then
    itemData = {position[1],position[2],0.002,0,0,1,0.2*s}
    sim.addDrawingObjectItem(self.shadowCont, itemData)
  end
end

return Vehicle

