import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QDateTime
import Common

# Import messages
from std_msgs.msg import String, Int32MultiArray, Float32, Empty
from geometry_msgs.msg import PointStamped, Polygon
from sensor_msgs.msg import NavSatFix

class AutoSIFRosNode(Node, QObject):
    ## define signals
    update_data = pyqtSignal(int)
    
    def __init__(self):
        Node.__init__(self, 'AutoSIF_gui_node')
        QObject.__init__(self)
        self.data_struct = Common.CommonData()  # shared data structure for storing ROS data

        self.qos_profile = QoSProfile(
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        
        # Define subscribers
        self.create_subscription(NavSatFix, "/ga_planner/current_fix", self.current_fix_callback, 10)
        self.create_subscription(NavSatFix, "/ga_planner/next_waypoint_navinfo", self.target_fix_callback, self.qos_profile)
        self.create_subscription(PointStamped, "ga_planner/next_waypoint", self.target_local_pos_callback, self.qos_profile)
        self.create_subscription(NavSatFix, "/ga_planner/home_fix", self.home_fix_callback, self.qos_profile)

        self.create_subscription(String, "/ga_planner/drone_state", self.drone_state_callback, self.qos_profile)
        self.create_subscription(String, '/ga_planner/bridge_state', self.bridge_state_callback, self.qos_profile)
        self.create_subscription(String, '/ga_planner/planner_state', self.planner_state_callback, self.qos_profile)

        self.create_subscription(Polygon, '/ga_planner/POI_list', self.POI_list_callback, self.qos_profile)
        self.create_subscription(Int32MultiArray, '/ga_planner/visited_sequence', self.visited_sequence_callback, self.qos_profile)
        self.create_subscription(Int32MultiArray, '/ga_planner/best_sequence', self.best_sequence_callback, self.qos_profile)

        self.create_subscription(Float32, '/ga_planner/sampling_time', self.sampling_time_callback, 10)

        # Define publishers
        self.POI_clicked_pub = self.create_publisher(PointStamped, "/ga_planner/POI", 10)
        self.takeoff_request_pub = self.create_publisher(Empty, "/ga_planner/request_takeoff", 10)
        self.landing_request_pub = self.create_publisher(Empty, "/ga_planner/request_land", 10)
        self.RTL_request_pub = self.create_publisher(Empty, "/ga_planner/request_RTL", 10)

        # Timer for main loop (will be started when thread runs)
        self.timer = None

    ### define signal connections to / from gui ###
    def connect_update_gui(self, callback):
        self.update_data.connect(callback)        
    
    ### define callback functions from ros topics ###
    # def imu_callback(self, msg): 
    #     # get orientation and convert to euler angles
    #     # note that uses PX4 [w, x, y, z] 
    #     self.data_struct.update_imu(msg.q[1], msg.q[2], msg.q[3], msg.q[0])

    def current_fix_callback(self, msg):
        self.data_struct.update_pos(msg.latitude, msg.longitude, msg.altitude)

    def target_fix_callback(self, msg):
        self.data_struct.update_target_pos(msg.latitude, msg.longitude, msg.altitude)

    def target_local_pos_callback(self, msg):
        self.data_struct.update_target_local_pos(msg.point.x, msg.point.y, msg.point.z)
    
    def home_fix_callback(self, msg):
        self.data_struct.update_home_global_pos(msg.latitude, msg.longitude, msg.altitude)

    def drone_state_callback(self, msg):
        self.data_struct.update_drone_state(drone_state=msg.data)
    
    def bridge_state_callback(self, msg):
        self.data_struct.update_bridge_state(bridge_state=msg.data)

    def planner_state_callback(self, msg):
        self.data_struct.update_planner_state(planner_state=msg.data)

    def POI_list_callback(self, msg):
        POI_list = [(point.x, point.y, point.z) for point in msg.points]
        self.data_struct.update_POI_list(POI_list=POI_list)

    def visited_sequence_callback(self, msg):
        visited_sequence = msg.data
        self.data_struct.update_visited_sequence(visited_sequence=visited_sequence)

    def best_sequence_callback(self, msg):
        best_sequence = msg.data
        self.data_struct.update_best_sequence(best_sequence=best_sequence)

    def sampling_time_callback(self, msg):
        self.data_struct.update_sampling_time(msg.data)
    
    def publish_POI(self, new_POI):
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = new_POI[0]
        point_msg.point.y = new_POI[1]
        point_msg.point.z = new_POI[2]
        self.POI_clicked_pub.publish(point_msg)

    def publish_takeoff_request(self):
        empty_msg = Empty()
        self.takeoff_request_pub.publish(empty_msg)

    def publish_landing_request(self):
        empty_msg = Empty()
        self.landing_request_pub.publish(empty_msg)
    
    def publish_RTL_request(self):
        empty_msg = Empty()
        self.RTL_request_pub.publish(empty_msg)

    # Timer callback for main loop
    def timer_callback(self):
        self.update_data.emit(0)
    
    # main loop of ros node (for compatibility with thread)
    def run(self):
        # Start the timer when the thread begins
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz
        
        # Use executor to spin in this thread
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        
        try:
            executor.spin()
        except Exception as e:
            print(f"ROS executor error: {e}")
        finally:
            executor.shutdown()
            self.destroy_node()

class AutoSIFRosThread:
    def __init__(self, ui):
        super().__init__()
        self.ros_object = AutoSIFRosNode()
        self.thread = QThread()

        # setup signals
        self.ui = ui
        self.set_ros_callbacks()

        # Move ROS node to thread and start
        self.ros_object.moveToThread(self.thread)
        self.lock = self.ros_object.data_struct.lock
        self.thread.started.connect(self.ros_object.run)

        # POI init
        self._last_poi_count = 0

    def start(self):
        self.thread.start()
    
    def log_message(self, message):
        """Add timestamped message to GUI logging widget"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        formatted_message = f"[{timestamp}] {message}"
        self.ui.Logging.append(formatted_message)

    # define the signal-slot combination of ros and pyqt GUI
    def set_ros_callbacks(self):
        # feedbacks from ros
        self.ros_object.connect_update_gui(self.update_gui_data)

        # callbacks from GUI
        self.ui.map.poi_clicked.connect(lambda x, y: self.handle_poi_click(x, y))

        self.ui.FollowerTakeoffButton.clicked.connect(self.send_takeoff_request)
        self.ui.FollowerLandButton.clicked.connect(self.send_land_request)
        self.ui.FollowerHomeButton.clicked.connect(self.send_RTL_request)
        self.ui.FollowerEmergencyButton.clicked.connect(self.hold) 
        self.ui.ResetViewButton.clicked.connect(self.ui.map.reset_view)

    # update GUI data
    def update_gui_data(self):
        if not self.lock.tryLock():
            print("AutoSIFRosThread: lock failed")
            return
        
        # store to local variables for fast lock release
        self.imu_msg = self.ros_object.data_struct.current_imu
        self.global_pos_msg = self.ros_object.data_struct.current_global_pos
        self.local_pos_msg = self.ros_object.data_struct.current_local_pos
        self.target_global_pos_msg = self.ros_object.data_struct.target_global_pos
        self.target_local_pos_msg = self.ros_object.data_struct.target_local_pos
        self.home_global_pos_msg = self.ros_object.data_struct.home_global_pos
        self.home_local_pos_msg = self.ros_object.data_struct.home_local_pos
        self.state_msg = self.ros_object.data_struct.current_state
        self.POI_info_msg = self.ros_object.data_struct.current_POI_info
        self.sampling_time_msg = self.ros_object.data_struct.sampling_time
        self.lock.unlock()

        # IMU data

        # Global position data
        self.ui.follower_GPS_lat.display("{:.1f}".format(self.global_pos_msg.latitude, 2))
        self.ui.follower_GPS_lon.display("{:.1f}".format(self.global_pos_msg.longitude, 2))
        self.ui.follower_GPS_alt.display("{:.1f}".format(self.global_pos_msg.altitude, 2))

        # Local position data
        self.ui.follower_rel_x.display("{:.1f}".format(self.local_pos_msg.x, 2))
        self.ui.follower_rel_y.display("{:.1f}".format(self.local_pos_msg.y, 2))
        self.ui.follower_rel_z.display("{:.1f}".format(self.local_pos_msg.z, 2))

        # Target position data
        self.ui.follower_target_lat.display("{:.1f}".format(self.target_global_pos_msg.latitude, 2))
        self.ui.follower_target_lon.display("{:.1f}".format(self.target_global_pos_msg.longitude, 2))
        self.ui.follower_target_alt.display("{:.1f}".format(self.target_global_pos_msg.altitude, 2))

        # Target local position data
        self.ui.follower_target_rel_x.display("{:.1f}".format(self.target_local_pos_msg.x, 2))
        self.ui.follower_target_rel_y.display("{:.1f}".format(self.target_local_pos_msg.y, 2))
        self.ui.follower_target_rel_z.display("{:.1f}".format(self.target_local_pos_msg.z, 2))

        # Home position data
        self.ui.home_GPS_lat.display("{:.1f}".format(self.home_global_pos_msg.latitude, 2))
        self.ui.home_GPS_lon.display("{:.1f}".format(self.home_global_pos_msg.longitude, 2))
        self.ui.home_GPS_alt.display("{:.1f}".format(self.home_global_pos_msg.altitude, 2))

        # State data
        self.ui.FollowerDroneState.setText(self.state_msg.drone_state)
        self.ui.FollowerBridgeState.setText(self.state_msg.bridge_state)
        if self.state_msg.bridge_state == "READY":
            self.ui.FollowerBridgeState.setStyleSheet("color: green")
        self.ui.FollowerPlannerState.setText(self.state_msg.planner_state)
        if self.state_msg.planner_state == "READY":
            self.ui.FollowerPlannerState.setStyleSheet("color: green")

        # POI info data
        self.update_mission_map()

        # POI data
        if len(self.POI_info_msg.visited_sequence) > 0:
            self.ui.POI_ID.display(self.POI_info_msg.visited_sequence[-1] + 1)
        else:
            self.ui.POI_ID.display(0)

        self.ui.sample_time.display("{:.2f}".format(self.sampling_time_msg, 2))
    
    def send_takeoff_request(self):
        self.ros_object.publish_takeoff_request()
        self.log_message("Takeoff request sent.")

    def send_land_request(self):
        self.ros_object.publish_landing_request()
        self.log_message("Landing request sent.")

    def send_RTL_request(self):
        self.ros_object.publish_RTL_request()
        self.log_message("RTL request sent.")

    def hold(self):
        pass

    def handle_poi_click(self, x, y):
        # Convert the clicked position to a new POI in global coordinates
        # For simplicity, we assume the clicked position is in local coordinates relative to the current position
        local_x, local_y = self.ui.map.map_to_local(x, y)

        new_POI_local = (local_x, local_y, 0.0)  # Assuming z=0 for now
        # current_global = self.global_pos_msg
        # new_POI_global = self.local_to_global(new_POI_local, current_global)

        # Publish the new POI to ROS
        self.ros_object.publish_POI(new_POI_local)
        self.log_message(f"Published new POI: {new_POI_local}")

    def update_mission_map(self):
        # Draw home
        self.ui.map.draw_home(self.home_local_pos_msg.x, self.home_local_pos_msg.y)

        # Draw current follower drone position
        self.ui.map.draw_follower_drone(self.local_pos_msg.x, self.local_pos_msg.y)

        POI_list = self.POI_info_msg.POI_list

        # Clear temp POI
        if len(POI_list) != self._last_poi_count:
            self.ui.map.clear_temp_POIs()
            self._last_poi_count = len(POI_list)

        # Draw POIs and sequence
        self.ui.map.draw_pois(
        POI_list,
        list(self.POI_info_msg.visited_sequence),
        list(self.POI_info_msg.best_sequence)
        )

