#include <assembly_dual_controllers/assembly_triple_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace assembly_dual_controllers {

bool AssemblyTripleController::initArm(
    hardware_interface::RobotHW* robot_hw,
    const std::string& arm_id,
    const std::vector<std::string>& joint_names) {
  std::shared_ptr<FrankaModelUpdater> arm_data = std::make_shared<FrankaModelUpdater>();
  auto & t_7e = arm_data->t_7e_;
  
  t_7e.setIdentity();
  t_7e.translation() << 0.0, 0.0, 0.103;

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data->model_handle_ = std::make_shared<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data->state_handle_ = std::make_shared<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data->joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "AssemblyTripleController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }
  arm_data->arm_name_ = arm_id;
  arm_data->q_out_file_.open(arm_id + "_q_out.txt");
  arm_data->x_out_file_.open(arm_id + "_x_out.txt");

  // arm_data->q_offset_.setZero();

  arms_data_.emplace(std::make_pair(arm_id, arm_data));
  return true;
}

bool AssemblyTripleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {

  init_load_.mass = 0.0;
  ros::service::call("/panda_dual/panda_left/set_load",init_load_,load_response_);
  ros::service::call("/panda_dual/panda_right/set_load",init_load_,load_response_);
  ros::service::call("/panda_dual/panda_top/set_load",init_load_,load_response_);

  if (!node_handle.getParam("left/arm_id", left_arm_id_)) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Could not read parameter left_arm_id_");
    return false;
  }
  std::vector<std::string> left_joint_names;
  if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7) {
    ROS_ERROR(
        "AssemblyTripleController: Invalid or no left_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("right/arm_id", right_arm_id_)) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Could not read parameter right_arm_id_");
    return false;
  }

  std::vector<std::string> right_joint_names;
  if (!node_handle.getParam("right/joint_names", right_joint_names) ||
      right_joint_names.size() != 7) {
    ROS_ERROR(
        "AssemblyTripleController: Invalid or no right_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("top/arm_id", top_arm_id_)) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Could not read parameter top_arm_id_");
    return false;
  }
  
  std::vector<std::string> top_joint_names;
  if (!node_handle.getParam("top/joint_names", top_joint_names) ||
      top_joint_names.size() != 7) {
    ROS_ERROR(
        "AssemblyTripleController: Invalid or no top_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  // ROS_INFO("1: %s \n 2: %s",left_arm_id_.c_str(), right_arm_id_.c_str());
  
  bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names);
  bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names);
  bool top_success = initArm(robot_hw, top_arm_id_, top_joint_names);
  Eigen::Matrix<double, 7, 4> calib_dh_left, calib_dh_right, calib_dh_top;
  // calib_dh_left << -0.00143634, 0.0392062, -0.00115552, -0.00304185,
  //                  -0.000326658, -0.000137608, -0.00857803, 0.00248295, 
  //                  9.32098e-05, -0.0045131, -0.000780235, -0.00249025, 
  //                  0.00146636, 0.000473081, 0.000193528, 0.000903572, 
  //                  -0.000881238, -0.00353094, 0.00778013, 0.000699071, 
  //                  0.000450429, -0.000987945, -0.00378704, -0.00345825, 
  //                  -0.0013828, 0.0414851, 0, 0.000106256;

  // calib_dh_right << -0.00132824, 0.0480551, -0.00387675, 0.00168824, 
  //                   -0.000470529, 0.000101516, -0.00425589, 0.00200649, 
  //                   0.000517718, -0.000735514, -0.00178362, -0.00336632, 
  //                   -0.000195107, 0.000333496, -8.29645e-05, 0.00206581, 
  //                   -0.00143737, 0.000159796, 0.013517, -0.00368535, 
  //                   -0.000291216, 7.60754e-05, -0.00444792, -0.00478127, 
  //                    -0.00131139, 0.049067, 0, -1.03542e-05;


  // // Calib dasta with 30k
  // calib_dh_left << 0.000540746250601151,	-0.000222551552355999,	 -0.00277793951052244,	 0.000844588438659563,
  //                 -4.97040526222079E-05,	-0.000509692151550602,	 -0.00789475906342737,	   0.0042426608859909,
  //                 -0.000102299237124574,	-0.000928166811547355,	 0.000515513982174587,	  -0.0038264870280442,
  //                 -0.000352436759011317,	 0.000721483809756229,	 -0.00631103844569646,	 0.000963423536246296,
  //                 -0.000190326952386907,	-0.000444144452174211,	  0.00998496188770845,	 0.000959572582040312,
  //                 -0.000789947947519676,	-0.000267151345711696,	  -0.0151345960265135,	 -0.00517021174194495,
  //                 -0.000523379357159486,	 1.03321081536609E-05,	   0.0231633152784359,	  0.00104263137358886;

  // calib_dh_right << -0.00140633066541627,	0.000110753546471257,	-0.00347512488622924,	-0.00382753804075078,
  //     0.00013844549683204,	-0.000212866764674761,	 -0.00447205653387941,	   0.0020609965711085,
  //     -0.000112296758878324,	-0.00107223972275933,	 0.00268994416750297,	-0.00298687830249312,
  //     -2.81088273319779E-05,	  0.00144327970495369,	 0.000260683084451204,	  0.00288767899612705,
  //     -0.000941617578740498,	 -0.00011000224447067,	   0.0137565696997384,	-0.000977122744254999,
  //     -0.0010075512018736,	 0.00047178225542559,	 -0.0133612927923467,	-0.00348189317285152,
  //     -0.00120499781917066,	 2.17966690332205E-05,	-0.000223974534326366,	-0.000113886804504759;
  
  // calib_dh_top << -7.95986087035948E-05,	 7.43695131967947E-05,	  0.00106520123664394,	  -0.0041949770968857,
  //     -0.000318719210519893,	-0.000112054216472271,	 0.000572935638052752,	  0.00139592643451262,
  //     -0.000327305308487304,	 -0.00073696399814617,	  0.00236332200338907,	 -0.00217657846953025,
  //     -0.000202035105200611,	  0.00073877634121057,	 -0.00520396129993958,	 0.000289536084428593,
  //     0.000120175829418395,	-0.000709615556852372,	  0.00425101581002266,	 -0.00154771190193392,
  //      -0.00031795272737588,	-0.000335279426667158,	 3.00797569860408E-05,	 -0.00367310742445092,
  //     -0.000499216401243057,	 6.80647060261321E-06,	  -0.0197035335601325,	 0.000464371902981031;

  // Calib dasta with closed chain calibration with 2.09e-5 eval
  calib_dh_left << 0.000474560429981023,	 0.000483166682165302,	 -0.00304883355188383,	  0.00148667086907321,
                  -3.69828865924539e-05,	-0.000288069909956647,	 -0.00812428761844092,	  0.00421567136144437,
                  -0.000154357131719552,	  -0.0010921364777817,	  0.00031894496234845,	  -0.0030474925191138,
                  -0.000117600404870226,	 0.000712982958085577,	 -0.00571261767823764,	  0.00176867969486185,
                  -0.00058993701921134,	-0.000326649645213642,	  0.00939394386245098,	  0.00123723772258799,
                  -0.000433705606644922,	-0.000293762477507038,	  -0.0156742348127345,	 -0.00529320945025931,
                  -0.000589815315429364,	  6.2389274666678e-05,	   0.0291501803388187,	  0.00113202442328629;

  calib_dh_right << -0.00153055068483914,	-0.000772485919009139,	 -0.00374187596482555,	 -0.00183369895482027,
                    0.000163834922530543,	-0.000212890734425727,	  -0.0041768339596184,	  0.00292223805919776,
                    -2.60271767179549e-05,	 -0.00100930869860036,	  0.00208915153420725,	 -0.00362801030623702,
                      0.0002126348171224,	  0.00108679894872676,	 9.81965015663654e-05,	  0.00292912798333623,
                    -0.00136529072609946,	-5.62799006356164e-05,	   0.0139799357258803,	 -0.00122374898174726,
                    -0.000502587880406569,	 0.000336192838117574,	  -0.0139808833528828,	 -0.00268675457630875,
                    -0.00104647166032287,	 0.000135297170834114,	  0.00498364620994882,	 0.000349775306996936;

  calib_dh_top << -0.000171539356569936,	 0.000591551783574421,	  0.00119525002639617,	 -0.00650097874689066,
                  -0.000330274824097498,	-0.000124214443868683,	 7.10962210595555e-05,	  0.00166835357174836,
                  -0.00027921463294522,	-0.000683991542242173,	  0.00203760570771006,	  -0.0018819778569208,
                  -0.000355247972007034,	 0.000891508331427601,	 -0.00513318787489872,	 0.000168196477584221,
                  0.000190817101859644,	-0.000635118518697479,	  0.00445732420517835,	 -0.00167223312645046,
                  -0.000244350284995939,	-0.000209543585115151,	 0.000118913154576129,	 -0.00356076901549127,
                  -0.000484192538997231,	 4.41640702632187e-05,	  -0.0155483388661319,	 0.000602582057747216;

  //KETI theta7 =0
  // calib_dh_left << 
  // -0.0013412, -0.0003918, -0.001096066770249, -0.001989675347267,
  // -0.0002564, -0.0001278, -0.008594001236793, 0.002752384230387,
  // -1.55E-05, 0.0006677, 0.00064926248174, -0.002103121748647,
  // 0.0009512, 0.0001758, -0.00371231531898, 0.000694641042292,
  // -0.000307, 0.0003903, 0.008051203839425, 0.002471386220816,
  // -0.000198, -0.0004224, -0.016758651477597, -0.005904448859478,
  // 0.0002232, -0.0011991, -0.01, 0.001268854366196;

  // calib_dh_right <<           
  // -0.0012854, -0.0014064, -0.00086917396749, -0.00093026049131,
  // 5.57E-05, -0.0001631, -0.003710569989728, 0.003317870908031,
  // -9.06E-05, 0.0006887, 0.00409803308367, -0.002958333082121,
  // 0.0010734, 0.0011147, -5.23598775596667E-06, 0.002099631090143,
  // -0.0003737, 0.0003613, 0.012063715789747, 0.001898918226164,
  // -0.0004362, 0.0003637, -0.011492993124347, -0.005555383009081,
  // 2.74000000000001E-05, -7.46E-05, 0.0, 0.002607521902471;

  // calib_dh_top <<
  // -0.0001923, -0.0029379, 0.000123918376891, -0.000776671517135,
  // 8.49E-05, -0.0002109, -0.004572762640211, 0.00317649923862,
  // -3.26E-05, 0.0006541, -0.001012290966154, -0.002075196480615,
  // 0.0004254, -0.0001784, -0.000457276264021, -0.001019272283162,
  // -0.0004222, 0.0009502, 0.009377654070936, 0.004321435227925,
  // -3E-06, -0.0003776, -0.006679375047361, -0.005880014249951,
  // 0.0004182, 0.0001634, 0.0, 0.000890117918514;

  //original KETI
  // calib_dh_left << 
  // -0.0013412, -0.0003918, -0.001096066770249, -0.001989675347267,
  // -0.0002564, -0.0001278, -0.008594001236793, 0.002752384230387,
  // -1.55E-05, 0.0006677, 0.00064926248174, -0.002103121748647,
  // 0.0009512, 0.0001758, -0.00371231531898, 0.000694641042292,
  // -0.000307, 0.0003903, 0.008051203839425, 0.002471386220816,
  // -0.000198, -0.0004224, -0.016758651477597, -0.005904448859478,
  // 0.0002232, -0.0011991, -0.032421236184946, 0.001268854366196;

  // calib_dh_right <<           
  // -0.0012854, -0.0014064, -0.00086917396749, -0.00093026049131,
  // 5.57E-05, -0.0001631, -0.003710569989728, 0.003317870908031,
  // -9.06E-05, 0.0006887, 0.00409803308367, -0.002958333082121,
  // 0.0010734, 0.0011147, -5.23598775596667E-06, 0.002099631090143,
  // -0.0003737, 0.0003613, 0.012063715789747, 0.001898918226164,
  // -0.0004362, 0.0003637, -0.011492993124347, -0.005555383009081,
  // 2.74000000000001E-05, -7.46E-05, -0.02926568089735, 0.002607521902471;

  // calib_dh_top <<
  // -0.0001923, -0.0029379, 0.000123918376891, -0.000776671517135,
  // 8.49E-05, -0.0002109, -0.004572762640211, 0.00317649923862,
  // -3.26E-05, 0.0006541, -0.001012290966154, -0.002075196480615,
  // 0.0004254, -0.0001784, -0.000457276264021, -0.001019272283162,
  // -0.0004222, 0.0009502, 0.009377654070936, 0.004321435227925,
  // -3E-06, -0.0003776, -0.006679375047361, -0.005880014249951,
  // 0.0004182, 0.0001634, -0.035850808165104, 0.000890117918514;

  arms_data_["panda_left"]->rbdl_model_.initModel(calib_dh_left);
  arms_data_["panda_right"]->rbdl_model_.initModel(calib_dh_right);
  arms_data_["panda_top"]->rbdl_model_.initModel(calib_dh_top);
  
  // arms_data_["panda_left"]->q_offset_ << -0.00209657, -0.0103961,-0.00302744,-0.00134568, 0.00393938, -0.0253182 -7.66409e-12;
  // arms_data_["panda_right"]->q_offset_ << -0.00113419,  -0.00316993,  0.000600501,  -0.00200384,  0.000897991,  -0.00657346, -2.02488e-12;
  // arms_data_["panda_right"]->q_offset_ << -0.00130828, -0.00322306, 0.000850395, -0.00201338,  0.00523785,  -0.0057014, -0.00770664;
  // arms_data_["panda_top"]->q_offset_.setZero();
    // Get the transformation from right_O_frame to left_O_frame
  // tf::StampedTransform transform;
  // tf::TransformListener listener;
  // try {
  //   if (listener.waitForTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", 
  //                                 ros::Time(0), ros::Duration(4.0))) {
  //     listener.lookupTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0",
  //                                 ros::Time(0), transform);
  //   } else {
  //     ROS_ERROR(
  //         "AssemblyTripleController: Failed to read transform from %s to %s. "
  //         "Aborting init!",
  //         (right_arm_id_ + "_link0").c_str(), (left_arm_id_ + "_link0").c_str());
  //     return false;
  //   }
  // } catch (tf::TransformException& ex) {
  //   ROS_ERROR("AssemblyTripleController: %s", ex.what());
  //   return false;
  // }
  // tf::transformTFToEigen(transform, Ol_T_Or_);  // NOLINT (readability-identifier-naming)


  idle_control_server_ = std::make_shared<IdleControlServer>
  ("/assembly_dual_controller/idle_control", node_handle, arms_data_);

  assemble_approach_action_server_ = std::make_shared<AssembleApproachActionServer>
  ("/assembly_dual_controller/assemble_approach_control", node_handle, arms_data_);
  assemble_spiral_action_server_ = std::make_shared<AssembleSpiralActionServer>
  ("/assembly_dual_controller/assemble_spiral_control", node_handle, arms_data_);
  assemble_insert_action_server_ = std::make_shared<AssembleInsertActionServer>
  ("/assembly_dual_controller/assemble_exert_force_control", node_handle, arms_data_);
  assemble_verify_action_server_ = std::make_shared<AssembleVerifyActionServer>
  ("/assembly_dual_controller/assemble_verify_completion_control", node_handle, arms_data_);
  joint_trajectory_action_server_ = std::make_shared<JointTrajectoryActionServer>
  ("/assembly_dual_controller/joint_trajectory_control", node_handle, arms_data_);
  assemble_move_action_server_ = std::make_shared<AssembleMoveActionServer>
  ("/assembly_dual_controller/assemble_move_control", node_handle, arms_data_);
  // assemble_press_action_server_ = std::make_shared<AssemblePressActionServer>
  // ("/assembly_dual_controller/assemble_press_control", node_handle, arms_data_);
  // assemble_side_chair_action_server_ = std::make_shared<AssembleSideChairActionServer>
  // ("/assembly_dual_controller/assemble_side_chair_control", node_handle, arms_data_);
  assemble_dual_spiral_action_server_ = std::make_shared<AssembleDualSpiralActionServer>
  ("/assembly_dual_controller/assemble_dual_spiral_control", node_handle, arms_data_);
  assemble_dual_approach_action_server_ = std::make_shared<AssembleDualApproachActionServer>
  ("/assembly_dual_controller/assemble_dual_approach_control", node_handle, arms_data_);
  assemble_rotation_action_server_ = std::make_shared<AssembleRotationActionServer>
  ("/assembly_dual_controller/assemble_rotation_control", node_handle, arms_data_);
  assemble_triple_recovery_action_server_ = std::make_shared<AssembleTripleRecoveryActionServer>
  ("/assembly_dual_controller/assemble_triple_recovery_control", node_handle, arms_data_);
  assemble_approach_bolt_action_server_ = std::make_shared<AssembleApproachBoltActionServer>
  ("/assembly_dual_controller/assemble_approach_bolt_control", node_handle, arms_data_);
  assemble_retreat_bolt_action_server_ = std::make_shared<AssembleRetreatBoltActionServer>
  ("/assembly_dual_controller/assemble_retreat_bolt_control", node_handle, arms_data_);  
  task_space_move_action_server_ = std::make_shared<TaskSpaceMoveActionServer>
  ("/assembly_dual_controller/task_space_move", node_handle, arms_data_);
  task_space_multiple_move_action_server_ = std::make_shared<TaskSpaceMultipleMoveActionServer>
  ("/assembly_dual_controller/task_space_multiple_move", node_handle, arms_data_);
  assemble_probe_edge_action_server_ = std::make_shared<AssembleProbeEdgeActionServer>
  ("/assembly_dual_controller/assemble_probe_edge_control", node_handle, arms_data_);
  assemble_triple_move_action_server_ = std::make_shared<AssembleTripleMoveActionServer>
  ("/assembly_dual_controller/assemble_triple_move_control", node_handle, arms_data_);
  assemble_approach_hip_action_server_ = std::make_shared<AssembleApproachHipActionServer>
  ("/assembly_dual_controller/assemble_approach_hip_control", node_handle, arms_data_);
  assemble_kitting_action_server_ = std::make_shared<AssembleKittingActionServer>
  ("/assembly_dual_controller/assemble_kitting_control", node_handle, arms_data_);
  assemble_back_forth_action_server_ = std::make_shared<AssembleBackForthActionServer>
  ("/assembly_dual_controller/assemble_back_forth_control", node_handle, arms_data_);
  assemble_bolting_ready_action_server_ = std::make_shared<AssembleBoltingReadyActionServer>
  ("/assembly_dual_controller/assemble_bolting_ready_control", node_handle, arms_data_);
  assemble_GMM_spiral_action_server_ = std::make_shared<AssembleGMMSpiralActionServer>
  ("/assembly_dual_controller/assemble_GMM_spiral_control", node_handle, arms_data_);
  assemble_triple_hold_action_server_ = std::make_shared<AssembleTripleHoldActionServer>
  ("/assembly_dual_controller/assemble_triple_hold", node_handle, arms_data_);
  joint_limit_avoidance_action_server_ = std::make_shared<JointLimitAvoidanceActionServer>
  ("/assembly_dual_controller/joint_limit_avoidance_control", node_handle, arms_data_);

  // single_peginhole_action_server_ = std::make_shared<SinglePegInHoleActionServer>
  // ("/assembly_dual_controller/single_peg_in_hole_control", node_handle, dual_arm_info_);



  action_servers_.push_back(assemble_approach_action_server_);
  action_servers_.push_back(assemble_spiral_action_server_);
  action_servers_.push_back(assemble_insert_action_server_);
  action_servers_.push_back(assemble_verify_action_server_);
  action_servers_.push_back(joint_trajectory_action_server_);
  action_servers_.push_back(assemble_move_action_server_);
  action_servers_.push_back(assemble_dual_spiral_action_server_);
  action_servers_.push_back(assemble_dual_approach_action_server_);
  action_servers_.push_back(assemble_rotation_action_server_);
  action_servers_.push_back(assemble_triple_recovery_action_server_);
  action_servers_.push_back(assemble_approach_bolt_action_server_);
  action_servers_.push_back(assemble_retreat_bolt_action_server_);
  action_servers_.push_back(task_space_move_action_server_);
  action_servers_.push_back(task_space_multiple_move_action_server_);
  action_servers_.push_back(assemble_probe_edge_action_server_);
  action_servers_.push_back(assemble_triple_move_action_server_);
  action_servers_.push_back(assemble_approach_hip_action_server_);
  action_servers_.push_back(assemble_kitting_action_server_);
  action_servers_.push_back(assemble_back_forth_action_server_);
  action_servers_.push_back(assemble_bolting_ready_action_server_);
  action_servers_.push_back(assemble_GMM_spiral_action_server_);
  action_servers_.push_back(assemble_triple_hold_action_server_);
  action_servers_.push_back(joint_limit_avoidance_action_server_);
  
  std::string homedir = getenv("HOME");
  
  for (auto & as : action_servers_)
  {
    as->openDebugFile(homedir + "/suhan_motion_control/logs");
  }

  return left_success && right_success && top_success;
}


void AssemblyTripleController::startingArm(FrankaModelUpdater& arm_data) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Isometry3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  sb_.reset();
}

void AssemblyTripleController::starting(const ros::Time& time) {
  // unsigned long mask = 1; /* processor 0 */  
  // /* bind process to processor 0 */  
  // if (pthread_setaffinity_np(pthread_self(), sizeof(mask), (cpu_set_t *)&mask) <0) 
  // {  
  //   perror("pthread_setaffinity_np");  
  // }

  for (auto& arm_data : arms_data_) {
    startingArm(*arm_data.second);
  }
  start_time_ = ros::Time::now();
}

void AssemblyTripleController::update(const ros::Time& time, const ros::Duration& period) {

  // std::cout << period.toSec() << std::endl;
  double t[30];
  t[0] = sb_.elapsedAndReset();

  std::vector<std::thread> model_update_threads;
  for (auto& arm : arms_data_) {
    model_update_threads.push_back(std::thread([&]() {
      arm.second->updateModel();
      arm.second->target_updated_ = false;
    }));
  }
  for (auto & thread : model_update_threads)
  {
    thread.join();
  }

  
  Eigen::IOFormat tab_format(Eigen::FullPrecision, 0, "\t", "\n");
  int q_num = 0;

  // // ONE ARM
  // Eigen::Matrix<double, 7, 1> q_total;
  // q_total.segment<7>(q_num * 7) = arms_data_["panda_right"] -> q_;
  // q_num++;
  
  // ALL ARM
  Eigen::Matrix<double, 21, 1> q_total;
  for (auto& arm : arms_data_)
  {
    q_total.segment<7>(q_num * 7) = arm.second->q_;
    q_num++;
  }

  // debug_file_q << q_total.transpose().format(tab_format) << std::endl;

  t[1] = sb_.elapsedAndReset();

  int ctr_index = 2;
  for (auto & as : action_servers_)
  {
    as->compute(time);
    t[ctr_index++] = sb_.elapsedAndReset();
  }
  // add normal action server above ----------
  idle_control_server_->compute(time);
  t[ctr_index++] = sb_.elapsedAndReset();
  // assemble_dual_side_chair_recovery_action_server_->compute(time);
  // t[ctr_index++] = sb_.elapsedAndReset();


  for(int i=0; i<ctr_index; ++i)
  {
    debug_file_td_ << t[i] << '\t';
  }
  debug_file_td_ << std::endl;
    

  // Eigen::Matrix<double, 7, 1> tau_cmd;  
  // tau_cmd.setZero();
  // for (size_t i = 0; i < 7; ++i) {
  //     arms_data_[right_arm_id_].joint_handles_[i].setCommand(tau_cmd(i));
  //   }
  // for (size_t i = 0; i < 7; ++i) {
  //   arms_data_[left_arm_id_].joint_handles_[i].setCommand(tau_cmd(i));
  // }
///////////////////////////////////////////////
}

Eigen::Matrix<double, 7, 1> AssemblyTripleController::saturateTorqueRate(
    const FrankaModelUpdater& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}

void AssemblyTripleController::writeDebugInfos(const std::string &title, const std::string &context)
{
  const auto now = std::chrono::system_clock::now();
  const auto now_time_t = std::chrono::system_clock::to_time_t(now);
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  debug_file_ << "["
      << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %a %T")
      << '.' << std::setfill('0') << std::setw(3) << now_ms.count() << "]-";
  
  debug_file_ << "[" << title << "]: " << context << std::endl;
  debug_file_.precision(4);
  debug_file_ << std::setfill(' ');
}

/*
void AssemblyTripleController::updateArm(FrankaModelUpdater& arm_data) {
  // get state variables
  arm_data.updateModel();

  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  tau_d.setZero();
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(arm_data, tau_d, arm_data.tau_desired_read_);
  for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles_[i].setCommand(tau_d(i));
  }
      //arm_data.orientation_d_.slerp(arm_data.filter_params_, arm_data.orientation_d_target_);
}
*/


}  // namespace assembly_dual_controllers

PLUGINLIB_EXPORT_CLASS(assembly_dual_controllers::AssemblyTripleController,
                       controller_interface::ControllerBase)
