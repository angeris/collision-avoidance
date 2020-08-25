#!/home/kunal/julia/bin/julia


using RobotOS

@rosimport geometry_msgs.msg: Pose, PoseStamped
@rosimport carp_ros.msg: Ellipsoid, EllipsoidArray
@rosimport carp_ros.srv: CarpService
rostypegen()
import .geometry_msgs.msg: Pose, PoseStamped
import .carp_ros.msg: Ellipsoid, EllipsoidArray
import .carp_ros.srv: CarpService, CarpServiceRequest, CarpServiceResponse

function carpServiceCB(req::CarpServiceRequest)
    println("running CARP service")
    println(req.goal)
    rsp = CarpServiceResponse()
    if length(req.obstacles.ellipsoids) == 0
        println("no obstacles returning goal")
        rsp.point = req.goal
    else
        # real code goes here
        for ob in req.obstacles.ellipsoids
            println(ob.center)
            println(ob.shape)
        end
    end
    println(rsp.point)
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