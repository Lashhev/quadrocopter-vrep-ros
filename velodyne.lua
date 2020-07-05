local Velodyne = {}
Velodyne.__index = Velodyne

function Velodyne:new()
    local self = setmetatable({}, Velodyne)
    self.visionSensorHandles={}

    for i=1,4,1 do

        self.visionSensorHandles[i]=sim.getObjectHandle('velodyneVPL_16_sensor'..i)
    end

    self.ptCloudHandle=sim.getObjectHandle('velodyneVPL_16_ptCloud')

    self.frequency=5 -- 5 Hz

    self.options=2+8 -- bit0 (1)=do not display points, bit1 (2)=display only current points, bit2 (4)=returned data is polar (otherwise Cartesian), bit3 (8)=displayed points are emissive

    self.pointSize=2

    self.coloring_closeAndFarDistance={1,4}

    self.displayScaling=0.999 -- so that points do not appear to disappear in objects
    
	self.h=simVision.createVelodyneVPL16(self.visionSensorHandles,self.frequency,self.options,self.pointSize,self.coloring_closeAndFarDistance,self.displayScaling,self.ptCloudHandle)
  return self
end

function Velodyne:getRawData()
    data=simVision.handleVelodyneVPL16(self.h,sim.getSimulationTimeStep())
    local m=sim.getObjectMatrix(self.visionSensorHandles[1],-1)
    point32 = {}
    for i=0,#data/3-1,1 do
        d={data[3*i+1],data[3*i+2],data[3*i+3]}
        d=sim.multiplyVector(m,d)
        point32[i+1]={x=d[1], y=d[2], z=d[3]}
    end
    return point32
end

function Velodyne:clearup()
    simVision.destroyVelodyneVPL16(self.h)
end

return Velodyne
