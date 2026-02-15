function sysCall_init()
    print("========== INIT START ==========")
    
    -- Parametri
    cubeCount = 20
    moveAmplitude = 0.5
    moveSpeed = 0.05
    area = 5
    
    print("cubeCount = " .. cubeCount)
    
    cubes = {}
    floorHandle = sim.getObjectHandle('Floor')
    
    local baseSize = {0.4, 0.2, 0.5}
    local baseCube = sim.createPureShape(0, 16, baseSize, 1, nil)
    
    print("Base cube created: " .. baseCube)

    local jsonData = {cubes = {}}
    
    for i = 1, cubeCount, 1 do
        print("Creating cube " .. i)
        
        local h = sim.copyPasteObjects({baseCube}, 0)[1]

        local x = (math.random()*2*area) - area
        local y = (math.random()*2*area) - area
        local z = baseSize[3]/2

        sim.setObjectPosition(h, floorHandle, {x, y, z})

        local phase = math.random()*2*math.pi
        local direction = math.random()*2*math.pi
        
        print(string.format("  Cube %d: x=%.2f, y=%.2f, phase=%.2f, dir=%.2f", i, x, y, phase, direction))
        
        cubes[i] = {
            handle = h,
            basePos = {x, y, z},
            phase = phase,
            direction = direction,
            velocity = {0, 0, 0}
        }

        -- Aggiungi al JSON
        table.insert(jsonData.cubes, {
            x = x,
            y = y,
            z = z,
            phase = phase,
            direction = direction
        })
    end

    sim.removeObject(baseCube)
    
    print("Number of cubes in jsonData: " .. #jsonData.cubes)
    
    -- Salva su file JSON (opzionale)
    local json = require('dkjson')
    local jsonString = json.encode(jsonData, {indent=true})
    
    print("JSON string length: " .. string.len(jsonString))
    print("JSON preview: " .. string.sub(jsonString, 1, 200))
    
    local file = io.open("/tmp/cube_init_poses.json", "w")
    if file then
        file:write(jsonString)
        file:close()
        print("? SUCCESS: Saved to /tmp/cube_init_poses.json")
    else
        print("? ERROR: Could not open file for writing")
    end
    
    -- MODIFIED: integrated CoppeliaSim obstacles - ROS2 publisher
    obstaclesPub = simROS2.createPublisher('/dynamic_obstacles', 'std_msgs/msg/String')
    print("ROS2 publisher created for /dynamic_obstacles")
    
    print("========== INIT END ==========")
end

function sysCall_actuation()
    local t = sim.getSimulationTime()

    for i = 1, #cubes, 1 do
        local c = cubes[i]
        local offset = moveAmplitude * math.sin(2*math.pi*moveSpeed*t + c.phase)
        
        -- MODIFIED: Calcola velocità (derivata della posizione)
        local vel_offset = moveAmplitude * 2*math.pi*moveSpeed * math.cos(2*math.pi*moveSpeed*t + c.phase)
        local vx = vel_offset * math.cos(c.direction)
        local vy = vel_offset * math.sin(c.direction)
        
        local offset_x = offset * math.cos(c.direction)
        local offset_y = offset * math.sin(c.direction)
        
        local p = {c.basePos[1] + offset_x, c.basePos[2] + offset_y, c.basePos[3]}
        sim.setObjectPosition(c.handle, floorHandle, p)
        
        -- Salva velocità
        c.velocity = {vx, vy, 0}
    end
    
    -- MODIFIED: Pubblica su ROS2
    local json = require('dkjson')
    local obstacles = {}
    for i = 1, #cubes, 1 do
        local c = cubes[i]
        local pos = sim.getObjectPosition(c.handle, -1)  -- -1 = global frame (importante!)
        table.insert(obstacles, {
            id = i,
            x = pos[1],
            y = pos[2],
            z = pos[3],
            vx = c.velocity[1],
            vy = c.velocity[2]
        })
    end
    
    local msgData = {data = json.encode({obstacles = obstacles})}
    simROS2.publish(obstaclesPub, msgData)
end

function sysCall_cleanup()
    -- Cleanup publisher
    if obstaclesPub then
        simROS2.shutdownPublisher(obstaclesPub)
        print("ROS2 publisher shutdown")
    end
end
