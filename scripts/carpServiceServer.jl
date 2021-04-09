#!/usr/bin/julia


using RobotOS
using CARP

@rosimport geometry_msgs.msg: Pose, PoseStamped
@rosimport carp_ros.msg: Ellipsoid, obstacleArray
@rosimport carp_ros.srv: CarpService
rostypegen()
import .geometry_msgs.msg: Pose, PoseStamped
import .carp_ros.msg: Ellipsoid, obstacleArray
import .carp_ros.srv: CarpService, CarpServiceRequest, CarpServiceResponse
function carpServiceCB(req::CarpServiceRequest)
    println("called")
    rsp = CarpServiceResponse()
    if length(req.obstacles.ellipsoids) == 0
        println("no obstacles returning goal")
        rsp.projection = req.goal
    else
        # build model
        agent = AgentModel()
        println("building problem")
        # set position
        pos  = [req.position.x, req.position.y, req.position.z]
        set_current_position!(agent, pos)
        # set goal
        goalPt = [req.goal.x, req.goal.y, req.goal.z]
        set_goal_point!(agent, goalPt)
        println(pos)
        println(goalPt)
        for (name, ob) in zip(req.obstacles.names, req.obstacles.ellipsoids)
            println(name)
            println(ob.center)
            println(ob.shape)
            ob_shape = reshape(ob.shape, (3, 3))
            set_ellipsoid!(agent, name, 
                           ob.center,
                           reshape(ob.shape, (3, 3)))
        end
        println("problem built")
        find_projection!(agent)
        rsp.success = agent.solved
        proj = agent.trajectory[:,end]
        rsp.projection.x = proj[1]
        rsp.projection.y = proj[2]
        rsp.projection.z = proj[3]
    end
    println(rsp.projection)
    return rsp
end

function main()  
    init_node("carp_server")
    carpServiceServer = RobotOS.Service{CarpService}(
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