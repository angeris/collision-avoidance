#!/home/kunal/julia/bin/julia


using RobotOS
using CARP

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
        # build model
        println("building problem")
        agent = AgentModel()
        # set position
        println("position")
        position  = [req.position.x, req.position.y, req.position.z]
        set_current_point!(agent, position)
        # set goal
        println("goal")
        goalPt = [req.goal.x, req.goal.y, req.goal.z]
        println(req.goal)
        println(goalPt)
        set_goal_point!(agent, goalPt)
        println("obs in")
        for (i, ob) in enumerate(req.obstacles.ellipsoids)
            ob_center = [ob.center.x, ob.center.y, ob.center.z] 
            ob_shape = reshape(ob.shape, (3, 3))
            set_ellipsoid!(agent, "test", ob_center, ob_shape)
        end
        println("proj")
        find_projection!(agent)
        proj = agent.projected_point
        rsp.point.x = proj[1]
        rsp.point.y = proj[2]
        rsp.point.z = proj[3]
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