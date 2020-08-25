#!/home/kunal/julia/bin/julia


using RobotOS

@rosimport geometry_msgs.msg: Pose, PoseStamped
@rosimport carp_ros.msg: Ellipsoid, EllipsoidArray
rostypegen()
import .geometry_msgs.msg: Pose, PoseStamped
import .carp_ros.msg: Ellipsoid, EllipsoidArray

function callback(msg::PoseStamped, pub_obj::Publisher{Ellipsoid})
    ellipsoid = Ellipsoid()
    ellipsoid.center.x = 1
    publish(pub_obj, ellipsoid)
end

function main()
    init_node("carp_server")
    print("starting CARP service")
    pub = Publisher{Ellipsoid}("ellipsoid", queue_size=10)
    sub = Subscriber{PoseStamped}("/quad0/mavros/local_position/pose",
                                  callback, (pub,), queue_size=10)
end

if ! isinteractive()
    while ! is_shutdown()
        main()
        RobotOS.spin()
    end
end