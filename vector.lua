local vector = {}
vector.__index = vector

local function is_vector(t)
    return getmetatable(t) == vector
end

function vector.new(x, y, z)
    return setmetatable({ x = x or 0, y = y or 0, z = z or 0 }, vector)
end

-- operator overloading
function vector.__add(lhs, rhs)
    assert(is_vector(lhs) and is_vector(rhs), "Type mismatch: vector expected.")
    return vector.new(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z)
end

function vector.__sub(lhs, rhs)
    assert(is_vector(lhs) and is_vector(rhs), "Type mismatch: vector expected.")
    return vector.new(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z)
end

function vector.__mul(lhs, rhs)
    local is_rhs_vector = is_vector(rhs)
    local is_lhs_vector = is_vector(lhs)
    if type(lhs) == "number" and is_rhs_vector then
        return vector.new(rhs.x * lhs, rhs.y * lhs, rhs.z * lhs)
    elseif type(rhs) == "number" and is_lhs_vector then
        return vector.new(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs)
    elseif is_rhs_vector and is_lhs_vector then
        return vector.new(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z)
    else
        error("Type mismatch: vector and/or number expected", 2)
    end
end

function vector.__unm(t)
    assert(is_vector(t), "Type mismatch: vector expected.")
    return vector.new(-t.x, -t.y, -t.z)
end

function vector:__tostring()
    return "("..self.x..", "..self.y..", "..self.z..")"
end

function vector.__eq(lhs, rhs)
    return lhs.x == rhs.x and lhs.y == rhs.y and lhs.z == rhs.z
end

function vector.__lt(lhs, rhs)
    return lhs.x < rhs.x or (not (rhs.x < lhs.x) and lhs.y < rhs.y)
end

function vector.__le(lhs, rhs)
    return lhs.x <= rhs.x or lhs.y <= rhs.y
end


-- actual functions
function vector:clone()
    return vector.new(self.x, self.y, self.z)
end

function vector:length()
    return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z )
end

function vector:length_squared()
    return self.x * self.x + self.y * self.y + self.z * self.z
end

function vector:is_unit()
    return self:length_squared() == 1
end

function vector:unpack()
    return self.x, self.y, self.y
end

function vector:normalize()
    local len = self:length()
    if len ~= 0 and len ~= 1 then
        self.x = self.x / len
        self.y = self.y / len
        self.z = self.z / len
    end
    return self
end

function vector:normalized()
    return self:clone():normalize()
end

function vector.dot(lhs, rhs)
    assert(is_vector(lhs) and is_vector(rhs), "Type mismatch: vector expected")
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z
end

function vector.distance(lhs, rhs)
    assert(is_vector(lhs) and is_vector(rhs), "Type mismatch: vector expected")
    local dx, dy, dz = lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)
end

function vector.distance_squared(lhs, rhs)
    assert(is_vector(lhs) and is_vector(rhs), "Type mismatch: vector expected")
    local dx, dy, dz = lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z
    return dx * dx + dy * dy + dz * dz
end

function vector.max(lhs, rhs)
    assert(is_vector(lhs) and is_vector(rhs), "Type mismatch: vector expected")
    local x = math.max(lhs.x, rhs.x)
    local y = math.max(lhs.y, rhs.y)
    local z = math.max(lhs.z, rhs.z)
    return vector.new(x, y, z)
end

function vector.min(lhs, rhs)
    assert(is_vector(lhs) and is_vector(rhs), "Type mismatch: vector expected")
    local x = math.min(lhs.x, rhs.x)
    local y = math.min(lhs.y, rhs.y)
    local z = math.min(lhs.z, rhs.z)
    return vector.new(x, y, z)
end

function vector.angle(from, to)
    assert(is_vector(from) and is_vector(to), "Type mismatch: vector expected")
    return math.acos(vector.dot(from, to) / (from:length() * to:length()))
end

function vector.direction(from, to)
    assert(is_vector(from) and is_vector(to), "Type mismatch: vector expected")
    return math.atan2(to.y - from.y, to.x - from.y, to.z - from.z)
end

function vector.lerp(from, to, t)
    assert(is_vector(from) and is_vector(to), "Type mismatch: vector expected")
    assert(type(t) == "number", "Type mismatch: number expected for t")
    return from * t + (to * (1 - t))
end


return vector