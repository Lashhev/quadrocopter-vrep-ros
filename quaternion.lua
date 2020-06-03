local Quaternion = {}
Quaternion.__index = Quaternion

function Quaternion:new(yaw, pitch, roll)
    local self = setmetatable({}, Quaternion)
    local cy = math.cos(yaw * 0.5);
    local sy = math.sin(yaw * 0.5);
    local cp = math.cos(pitch * 0.5);
    local sp = math.sin(pitch * 0.5);
    local cr = math.cos(roll * 0.5);
    local sr = math.sin(roll * 0.5);

    self.w = cy * cp * cr + sy * sp * sr
    self.x = cy * cp * sr - sy * sp * cr
    self.y = sy * cp * sr + cy * sp * cr
    self.z = sy * cp * cr - cy * sp * sr
    return self
end
function Quaternion:quaternion(x, y, z, w)
    local self = setmetatable({}, Quaternion)
    self.w = w
    self.x = x
    self.y = y
    self.z = z
    return self
end
--TODO:
function Quaternion:copysign(dest, src)
    if src < 0 then
        dest = dest*(-1)
    end
    return dest
end
function Quaternion:toEuler()
    local euler = {}
    local sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z)
    local cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y)
    local roll = math.atan2(sinr_cosp, cosr_cosp)
    local pitch = 0.0
    local yaw = 0.0
    -- pitch (y-axis rotation)
    local sinp = 2.0 * (self.w * self.y - self.z * self.x)
    if math.abs(sinp) >= 1 then
        pitch = Quaternion:copysign(math.pi / 2, sinp) -- use 90 degrees if out of range
    else
        pitch = math.asin(sinp)
    end

    -- yaw (z-axis rotation)
    local siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
    local cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z) 
    yaw = math.atan2(siny_cosp, cosy_cosp)
    euler[1] = roll
    euler[2] = pitch
    euler[3] = yaw
    return euler;
end
return Quaternion