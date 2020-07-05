local StereoCamera = {}
StereoCamera.__index = StereoCamera

function StereoCamera:new()
  local self = setmetatable({}, StereoCamera)
  self.vision_handler = sim.getObjectHandle('anaglyphStereoSensor')
  self.vision_sensor_handlers = {sim.getObjectHandle('anaglyphStereoSensor_leftSensor'), sim.getObjectHandle('anaglyphStereoSensor_rightSensor')}
  self.name = "anaglyphStereoSensor"
  return self
end

function StereoCamera:getSensorImage(number)
  local leftAndRightColors=nil
  local result = simVision.handleAnaglyphStereo(self.vision_handler,self.vision_sensor_handlers, leftAndRightColors)
  return sim.getVisionSensorCharImage(self.vision_sensor_handlers[number], 0, 0, 0)
end

function StereoCamera:getSensorInfo(number)
  local resolution = sim.getVisionSensorResolution(self.vision_sensor_handlers[number])
  local result, view_angle = sim.getObjectFloatParameter(self.vision_sensor_handlers[number], sim.visionfloatparam_perspective_angle)
  return result, resolution, view_angle
end

function StereoCamera:getDepthBuffer(number)
  local leftAndRightColors=nil
  local result = simVision.handleAnaglyphStereo(self.vision_handler, self.vision_sensor_handlers, leftAndRightColors)
  return sim.getVisionSensorDepthBuffer(self.vision_sensor_handlers[number] + sim.handleflag_codedstring)
end

function StereoCamera:getSensorResolution(number)
  return sim.getVisionSensorResolution(self.vision_sensor_handlers[number])
end

function StereoCamera:getNearClipping(number)
  return sim.getObjectFloatParameter(self.vision_sensor_handlers[number], sim.visionfloatparam_near_clipping)
end

function StereoCamera:getFarClipping(number)
  return sim.getObjectFloatParameter(self.vision_sensor_handlers[number], sim.visionfloatparam_far_clipping)
end

return StereoCamera
