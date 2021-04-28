#!/usr/bin/julia


using RobotOS
using CARP

@rosimport geometry_msgs.msg: Vector3, Pose, PoseStamped
@rosimport carp_ros.msg: Ellipsoid, obstacleArray
@rosimport carp_ros.srv: CarpServicePoly
rostypegen()
import .geometry_msgs.msg: Vector3, Pose, PoseStamped
import .carp_ros.msg: Ellipsoid, obstacleArray
import .carp_ros.srv: CarpServicePoly, CarpServicePolyRequest, CarpServicePolyResponse


if has_param("order")
    order = get_param("order")
    order = parse(Float64, order)
else
    order = 3
end
println("order: ", order)

agent = AgentModel(order=order)
function carpServiceCB(req::CarpServicePolyRequest)
    # println("called")
    start = time()
    rsp = CarpServicePolyResponse()
    if length(req.obstacles.ellipsoids) == 0
        println("no obstacles returning goal")
        rsp.projection = req.goal
    else
        # build model
        # println("building problem")
        # set position
        pos  = [req.position.x, req.position.y, req.position.z]
        set_current_position!(agent, pos)
        # set velocity 
        vel = [req.velocity.x, req.velocity.y, req.velocity.z]
        set_current_velocity!(agent, vel)
        # set goal
        goalPt = [req.goal.x, req.goal.y, req.goal.z]
        set_goal_point!(agent, goalPt)
        for (name, ob) in zip(req.obstacles.names, req.obstacles.ellipsoids)
            ob_shape = reshape(ob.shape, (3, 3))
            set_ellipsoid!(agent, name, 
                           ob.center,
                           reshape(ob.shape, (3, 3)))
        end
        # println("solving problem")
        find_projection!(agent)
        rsp.success = agent.solved
        # unpack trajectory
        rsp.trajectory.x = agent.trajectory[1, :]
        rsp.trajectory.y = agent.trajectory[2, :]
        rsp.trajectory.z = agent.trajectory[3, :]
    end
    # println(time()-start)
    # println(rsp.projection)
    return rsp
end

function main()  
    init_node("carp_server")
    carpServiceServer = RobotOS.Service{CarpServicePoly}(
                            "carpService", carpServiceCB)
    println("CARP service online")
end


if ! isinteractive()
    try
        while ! is_shutdown()
            main()
            RobotOS.spin()
        end
    catch e
        println(e)
        bt = stacktrace(catch_backtrace())
        io = IOBuffer()
        showerror(io, e, bt)
        errstr = String(take!(io))
        println(errstr)
        RobotOS.error("Error: $errstr")
        exit()
    end
end