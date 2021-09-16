rospy.init_node('turtle_tf_listener')

listener = tf.TransformListener()

rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
spawner(4, 2, 0, 'turtle2')

turtle_vel = rospy.Publisher(
     'turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

 rate = rospy.Rate(10.0)
  while not rospy.is_shutdown():
       try:
            (trans, rot) = listener.lookupTransform(
                '/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()


# debug
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

# pose goal stuff
# angleEuler = [-43.215957814544076, 81.69670258985799, -37.461921822267826]

