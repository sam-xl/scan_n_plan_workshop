#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from snp_msgs.srv import GenerateMotionPlan  # Adjust to your package name
from snp_msgs.msg import ToolPath
from geometry_msgs.msg import PoseArray, Pose
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory


def create_dummy_tool_path():
    pose1 = Pose()

    pose1.position.x = 0.1
    pose1.position.y = 0.5
    pose1.position.z = 0.3
    pose1.orientation.w = 1.0  # Neutral orientation

    pose2 = Pose()
    pose2.position.x = 1.0
    pose2.position.y = 0.5
    pose2.position.z = 0.3
    pose2.orientation.w = 1.0

    segment = PoseArray()
    segment.header.frame_id = "ur10e_base_link"
    segment.poses.append(pose1)
    segment.poses.append(pose2)

    tool_path = ToolPath()
    tool_path.segments.append(segment)

    return tool_path

def create_inspection_toolpath():
    segment_15 = [
        {
            "position": {"x": 0.0004842154448851943, "y": -1.4100733995437622, "z": -0.002},
            "orientation": {"x": -0.07793066912874154, "y": 0.0007846599502398713, "z": 0.9969460004488957, "w": 0.004986713031622316}
        },
        {
            "position": {"x": 0.019951604306697845, "y": -1.4102667570114136, "z": 0.0007849433459341526},
            "orientation": {"x": -0.07793066912874154, "y": 0.0007846599502398713, "z": 0.9969460004488957, "w": 0.004986713031622316}
        },
        {
            "position": {"x": 0.05743969976902008, "y": -1.4106395244598389, "z": 0.00613534078001976},
            "orientation": {"x": -0.06747192389390057, "y": 0.0009063329664144652, "z": 0.9977082194254596, "w": 0.005002693023618797}
        },
        {
            "position": {"x": 0.09562046825885773, "y": -1.4110198020935059, "z": 0.010254361666738987},
            "orientation": {"x": -0.046655029273608664, "y": 0.0011689708477119899, "z": 0.9988977073574185, "w": 0.005031101939848325}
        },
        {
            "position": {"x": 0.13392628729343414, "y": -1.4114017486572266, "z": 0.013034529983997345},
            "orientation": {"x": -0.026901476394831963, "y": 0.001420398373742254, "z": 0.9996243667567101, "w": 0.005041668621051365}
        },
        {
            "position": {"x": 0.17231087386608124, "y": -1.4117851257324219, "z": 0.014344742521643639},
            "orientation": {"x": -0.005401685271853137, "y": 0.0016185226553705888, "z": 0.9999714067017337, "w": 0.0050386466033920715}
        },
        {
            "position": {"x": 0.21071772277355194, "y": -1.4121692180633545, "z": 0.01419512927532196},
            "orientation": {"x": 0.013907475132334167, "y": 0.0019085132649452905, "z": 0.9998888700069652, "w": 0.005018699906871694}
        },
        {
            "position": {"x": 0.2490965723991394, "y": -1.4125536680221558, "z": 0.012732498347759247},
            "orientation": {"x": 0.03282193846086334, "y": 0.0021798397719904445, "z": 0.9994463689970337, "w": 0.004992409526971616}
        },
        {
            "position": {"x": 0.28739097714424133, "y": -1.4129377603530884, "z": 0.009816630743443966},
            "orientation": {"x": 0.04343565764682736, "y": 0.0022648272520990665, "z": 0.9990414999165297, "w": 0.004929061455938332}
        },
        {
            "position": {"x": 0.32556450366973877, "y": -1.4133211374282837, "z": 0.005484301131218672},
            "orientation": {"x": 0.062388584362678266, "y": 0.002550439294381521, "z": 0.9980368121021533, "w": 0.0048663630770686235}
        },
    ]

    segment_16 = [
        {
            "position": {"x": 0.3254961371421814, "y": -1.5033247470855713, "z": 0.0059569827280938625},
            "orientation": {"x": 0.062051867305432, "y": 0.0027590571422973016, "z": 0.998057284070361, "w": 0.00485912355154414}
        },
        {
            "position": {"x": 0.2873583137989044, "y": -1.5029417276382446, "z": 0.010232768952846527},
            "orientation": {"x": 0.04316831078088589, "y": 0.0025463911577979286, "z": 0.9990524416519926, "w": 0.004922566971446661}
        },
        {
            "position": {"x": 0.24905790388584137, "y": -1.502557635307312, "z": 0.013091550208628178},
            "orientation": {"x": 0.03259571683678087, "y": 0.002419578121812722, "z": 0.9994532179953561, "w": 0.004992987516118157}
        },
        {
            "position": {"x": 0.21067661046981812, "y": -1.5021731853485107, "z": 0.01451333798468113},
            "orientation": {"x": 0.01353662900543489, "y": 0.002208393786305882, "z": 0.9998933226924881, "w": 0.005022539898247144}
        },
        {
            "position": {"x": 0.17226913571357727, "y": -1.5017890930175781, "z": 0.0146331200376153},
            "orientation": {"x": -0.005811664428499613, "y": 0.001958947489584137, "z": 0.9999684605352324, "w": 0.005046287358145638}
        },
        {
            "position": {"x": 0.1338822841644287, "y": -1.5014057159423828, "z": 0.013343703001737595},
            "orientation": {"x": -0.0271781066605266, "y": 0.0016886680135270245, "z": 0.9996164021680768, "w": 0.005054447075495158}
        },

        {
            "position": {"x": 0.09557326883077621, "y": -1.5010236501693726, "z": 0.01056517194956541},
            "orientation": {"x": -0.04710006014565197, "y": 0.0014174403777564384, "z": 0.9988764308625192, "w": 0.005044904796552793}
        },
        {
            "position": {"x": 0.05738726630806923, "y": -1.5006434917449951, "z": 0.006390026304870844},
            "orientation": {"x": -0.06791624335460865, "y": 0.0011145471704411024, "z": 0.997677780835554, "w": 0.005018695074096544}
        },
        {
            "position": {"x": 0.019318100064992905, "y": -1.5002650022506714, "z": 0.0008601610898040235},
            "orientation": {"x": -0.07833504430368805, "y": 0.000961864525434585, "z": 0.9969140723802074, "w": 0.005002793308190302}
        }
    ]

    segment_17 = [
        {
            "position": {"x": 0.01978740096092224, "y": -1.5902740955352783, "z": 0.0012296584900468588},
            "orientation": {"x": -0.0790502290025761, "y": 0.0011599248670142723, "z": 0.9968573094245635, "w": 0.005022003162392893}
        },
        {
            "position": {"x": 0.057350609451532364, "y": -1.5906474590301514, "z": 0.006654616445302963},
            "orientation": {"x": -0.06862984919460972, "y": 0.0013578451834967999, "z": 0.9976285677151815, "w": 0.005033978003359444}
        },
        {
            "position": {"x": 0.09552772343158722, "y": -1.5910277366638184, "z": 0.010854569263756275},
            "orientation": {"x": -0.04771635327145732, "y": 0.0017099215942290038, "z": 0.998846618320992, "w": 0.00506545825503371}
        },
        {
            "position": {"x": 0.13383476436138153, "y": -1.591409683227539, "z": 0.013692314736545086},
            "orientation": {"x": -0.02768448584489622, "y": 0.0020004340606250004, "z": 0.9996018572642265, "w": 0.0050689704064942305}
        },
        {
            "position": {"x": 0.172223299741745, "y": -1.5917929410934448, "z": 0.015044967643916607},
            "orientation": {"x": -0.006271139349755167, "y": 0.002361768677489835, "z": 0.9999647534465692, "w": 0.005058332186819972}
        },
        {
            "position": {"x": 0.21063542366027832, "y": -1.592177152633667, "z": 0.014948397874832153},
            "orientation": {"x": 0.013226924682870532, "y": 0.0025598855035921394, "z": 0.9998966024139466, "w": 0.00502791514328684}
        },
        {
            "position": {"x": 0.24902109801769257, "y": -1.5925614833831787, "z": 0.013533267192542553},
            "orientation": {"x": 0.03225544386420846, "y": 0.002661057353937697, "z": 0.9994636615430627, "w": 0.004989425800230573}
        },
        {
            "position": {"x": 0.287324458360672, "y": -1.5929456949234009, "z": 0.010666937567293644},
            "orientation": {"x": 0.04277147680212499, "y": 0.0028410978633959904, "z": 0.999068762726939, "w": 0.0049128686486628265}
        }
    ]
    segment_18 = [
        {
            "position": {"x": 0.28729671239852905, "y": -1.6829496622085571, "z": 0.011193695478141308},
            "orientation": {"x": 0.042503097601205776, "y": 0.0032226915666874107, "z": 0.9990791014363538, "w": 0.004904082635533626}
        },
        {
            "position": {"x": 0.24898356199264526, "y": -1.682565450668335, "z": 0.013998677022755146},
            "orientation": {"x": 0.031881818796215454, "y": 0.0032711133675066132, "z": 0.9994738566299654, "w": 0.004985916245333362}
        },
        {
            "position": {"x": 0.21059291064739227, "y": -1.6821811199188232, "z": 0.015401114709675312},
            "orientation": {"x": 0.013164688085252188, "y": 0.003149728080618573, "z": 0.9998956909850156, "w": 0.005037593694841437}
        },
        {
            "position": {"x": 0.17217697203159332, "y": -1.6817967891693115, "z": 0.01546697411686182},
            "orientation": {"x": -0.006563160101696104, "y": 0.0027207357557727844, "z": 0.9999619191673874, "w": 0.0050678142726465316}
        },
        {
            "position": {"x": 0.13378436863422394, "y": -1.6814135313034058, "z": 0.014107824303209782},
            "orientation": {"x": -0.028289372449795312, "y": 0.002402628138681321, "z": 0.9995839464796545, "w": 0.005086523821132416}
        },
        {
            "position": {"x": 0.09547081589698792, "y": -1.6810314655303955, "z": 0.011273316107690334},
            "orientation": {"x": -0.04828025419388572, "y": 0.002087891369029567, "z": 0.9988186831638968, "w": 0.005088804117300729}
        },
        {
            "position": {"x": 0.05728747323155403, "y": -1.6806511878967285, "z": 0.0070284404791891575},
            "orientation": {"x": -0.06934977394608938, "y": 0.001614676500526345, "z": 0.997578274914385, "w": 0.005058368537832509}
        },
        {
            "position": {"x": 0.018789276480674744, "y": -1.680268406867981, "z": 0.0013130001025274396},
            "orientation": {"x": -0.07979429269959183, "y": 0.001321175447109165, "z": 0.9967977720245061, "w": 0.0050325972410452365}
        }
    ]

    segment_1 = [
        {
            "position": {"x": 0.36352810263633728, "y": -0.83394348621368408, "z": -0.0018499144352972507},
            "orientation": {"x": 0.068234719521537174, "y": -8.2513282856245819e-05, "z": 0.99766674623291285, "w": 3.4083952761487759e-05}
        },
        {
            "position": {"x": 0.27311939001083374, "y": -0.83393824100494385, "z": 0.010160163044929504},
            "orientation": {"x": 0.042519145421228928, "y": -0.0005994888807324914, "z": 0.99886682535792137, "w": 8.6921428533035111e-06}
        },
        {
            "position": {"x": 0.18200258910655975, "y": -0.83393663167953491, "z": 0.014019723981618881},
            "orientation": {"x": -0.0010285475911698812, "y": -0.0010540100385643869, "z": 0.99972833827350238, "w": -6.0594365681364031e-05}
        },
        {
            "position": {"x": 0.090908445417881012, "y": -0.83393853902816772, "z": 0.0095905205234885216},
            "orientation": {"x": -0.045957471982855803, "y": -0.0010025612431318104, "z": 0.99866419794166417, "w": -0.00012380669785957079}
        },
        {
            "position": {"x": 0.00058835581876337528, "y": -0.83394402265548706, "z": -0.0030608219094574451},
            "orientation": {"x": -0.045957471982855803, "y": -0.0010025612431318104, "z": 0.99866419794166417, "w": -0.00012380669785957079}
        }
        ]

    segment_2 = [
        {
            "position": {"x": 0.00059514486929401755, "y": -0.93394404649734497, "z": -0.0030966650228947401},
            "orientation": {"x": -0.045553855098148688, "y": -0.00053504854804964754, "z": 0.99869039378895963, "w": -7.9258285757943206e-05}
        },
        {
            "position": {"x": 0.090928338468074799, "y": -0.93393862247467041, "z": 0.0094244014471769333},
            "orientation": {"x": -0.045553855098148688, "y": -0.00053504854804964754, "z": 0.99869039378895963, "w": -7.9258285757943206e-05}
        },
        {
            "position": {"x": 0.18201765418052673, "y": -0.9339367151260376, "z": 0.013819054700434208},
            "orientation": {"x": -0.0016370235462053963, "y": -0.00056249343520469288, "z": 0.99974613656844735, "w": -3.6662359343914388e-05}
        },
        {
            "position": {"x": 0.27313196659088135, "y": -0.93393832445144653, "z": 0.010062367655336857},
            "orientation": {"x": 0.041919312781426396, "y": -0.00010436598539019877, "z": 0.9988931559110924, "w": 8.7412618275676308e-06}
        },
        {
            "position": {"x": 0.36355909705162048, "y": -0.93394345045089722, "z": -0.0017418696079403162},
            "orientation": {"x": 0.067961086313658992, "y": 0.00036809929046757934, "z": 0.99768305281091396, "w": 5.3842027725355624e-06}
        }
        ]

    segment_3 = [
        {
            "position": {"x": 0.36357876658439636, "y": -1.0339434146881104, "z": -0.0015513536054641008},
            "orientation": {"x": 0.066280365060599158, "y": 0.0007504394696187859, "z": 0.99779785716550129, "w": -1.8719696416216543e-05}
        },
        {
            "position": {"x": 0.27313262224197388, "y": -1.0339384078979492, "z": 0.010075381025671959},
            "orientation": {"x": 0.041228660041582259, "y": 0.00035510155254447613, "z": 0.99893334651161614, "w": 9.2635335979522422e-06}
        },
        {
            "position": {"x": 0.18201784789562225, "y": -1.0339367389678955, "z": 0.01380540058016777},
            "orientation": {"x": -0.0011670444738023642, "y": -3.5668217082865123e-05, "z": 0.99972586651977513, "w": -1.217162103129096e-05}
        },
        {
            "position": {"x": 0.090935677289962769, "y": -1.0339386463165283, "z": 0.0093289641663432121},
            "orientation": {"x": -0.045313456562580796, "y": -8.0245698511132617e-05, "z": 0.99871128424385291, "w": -3.6835273560263393e-05}
        },
        {
            "position": {"x": 0.00059013190912082791, "y": -1.0339440107345581, "z": -0.003071177750825882},
            "orientation": {"x": -0.045313456562580796, "y": -8.0245698511132617e-05, "z": 0.99871128424385291, "w": -3.6835273560263393e-05}
        }
        ]



    poses_data = []
    # segment_15.reverse()
    poses_data.extend(segment_1)
    poses_data.extend(segment_2)
    poses_data.extend(segment_3)
    # poses_data.extend(segment_15)
    # segment_16.reverse()
    # poses_data.extend(segment_16)
    # segment_17.reverse()
    # poses_data.extend(segment_17)
    # segment_18.reverse()
    # poses_data.extend(segment_18)
    poses_data.reverse()
    
    segment = PoseArray()
    segment.header.frame_id = "composite_panel"

    for pose_data in poses_data:
        pose = Pose()
        pose.position.x = pose_data["position"]["x"]
        pose.position.y = pose_data["position"]["y"]
        pose.position.z = pose_data["position"]["z"]
        pose.orientation.x = pose_data["orientation"]["x"]
        pose.orientation.y = pose_data["orientation"]["y"]
        pose.orientation.z = pose_data["orientation"]["z"]
        pose.orientation.w = pose_data["orientation"]["w"]
        segment.poses.append(pose)

    tool_path = ToolPath()
    tool_path.segments.append(segment)

    return tool_path

class MotionPlanClient(Node):
    def __init__(self):
        super().__init__('motion_plan_client')
        self.cli = self.create_client(GenerateMotionPlan, '/generate_motion_plan')

        self.controller = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /generate_motion_plan...')
        
        self.req = GenerateMotionPlan.Request()

    def send_request(self):
        # Example: fill in tool paths (empty array here; fill as needed)
        self.req.tool_paths = [create_inspection_toolpath()]
        
        # Example: set motion group and TCP frame
        self.req.motion_group = 'manipulator'
        self.req.tcp_frame = 'tcp'

        self.future = self.cli.call_async(self.req)
        return self.future

    def send_goal(self, trajectory):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.controller.wait_for_server()
        self._send_goal_future = self.controller.send_goal_async(goal_msg)
        # self._send_goal_future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        self.ctrl_result_future = self.goal_response_callback(self._send_goal_future)
        rclpy.spin_until_future_complete(self, self.ctrl_result_future)
        # return self.ctrl_result_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"Goal rejected :(")
            return

        self.get_logger().info('Goal accepted :)')

        self.ctrl_result_future = goal_handle.get_result_async()
        self.ctrl_result_future.add_done_callback(self.get_result_callback)
        return self.ctrl_result_future

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.error_code))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)

    client = MotionPlanClient()
    try:
        plan_future  = client.send_request()
        rclpy.spin_until_future_complete(client, plan_future)

        response = plan_future.result()
        if response.success:
            client.get_logger().info('Received motion plan successfully!')
            # Example: print number of points in each trajectory
            client.get_logger().info(f'Approach: {len(response.approach.points)} points')
            client.get_logger().info(f'Process: {len(response.process.points)} points')
            # print(response.approach.points)
            client.get_logger().info(f'Departure: {len(response.departure.points)} points')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')

    # remove effort values because scaled_joint_trajectory_controller does not support it.
    for pt in response.approach.points:
     pt.effort=[]
    for pt in response.process.points:
     pt.effort=[]
    for pt in response.departure.points:
     pt.effort=[]

    client.send_goal(response.approach)
    client.send_goal(response.process)
    client.send_goal(response.departure)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
