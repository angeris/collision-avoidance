#!/home/kunal/julia/bin/julia


using RobotOS
# using CARP
println("hello")
@rosimport geometry_msgs.msg: Pose, PoseStamped
@rosimport carp_ros.msg: Ellipsoid, obstacleArray
@rosimport carp_ros.srv: CarpService
rostypegen()
import .geometry_msgs.msg: Pose, PoseStamped
import .carp_ros.msg: Ellipsoid, obstacleArray
import .carp_ros.srv: CarpService, CarpServiceRequest, CarpServiceResponse
println("he")
function carpServiceCB(req::CarpServiceRequest)
    println("running CARP service")
    println(req.position)
    println(req.goal)
    println(data)
    rsp = CarpServiceResponse()
    if length(req.obstacles.ellipsoids) == 0
        println("no obstacles returning goal")
        rsp.point = req.goal
    else
        # real code goes here
        for (name, ob) in zip(req.obstacles.names, req.obstacles.ellipsoids)
            println(name)
            println(ob.center)
            println(ob.shape)
        end
    end
    println(rsp.projection)
    return rsp
end

function main()
    init_node("carp_server")
    println("starting CARP service")
    carpServiceServer = RobotOS.Service{CarpService}("carpService", carpServiceCB)
end

if ! isinteractive()
    while ! is_shutdown()
        main()
        RobotOS.spin()
    end
end