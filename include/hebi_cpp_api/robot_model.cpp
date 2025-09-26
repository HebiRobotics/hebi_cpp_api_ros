#include "hebi_cpp_api/robot_model.hpp"

#include <iostream>

namespace hebi {
namespace robot_model {

////////////////////////// Objectives

EndEffectorPositionObjective::EndEffectorPositionObjective(const Eigen::Vector3d& obj)
  : _weight(1.0f), _x(obj[0]), _y(obj[1]), _z(obj[2]) {}

EndEffectorPositionObjective::EndEffectorPositionObjective(double weight, const Eigen::Vector3d& obj)
  : _weight(weight), _x(obj[0]), _y(obj[1]), _z(obj[2]) {}

HebiStatusCode EndEffectorPositionObjective::addObjective(HebiIKPtr ik) const {
  return hebiIKAddObjectiveFramePosition(ik, static_cast<float>(_weight), HebiFrameTypeEndEffector, 0, _x, _y, _z);
}

// clang-format off
EndEffectorSO3Objective::EndEffectorSO3Objective(const Eigen::Matrix3d& matrix)
  : _weight(1.0f), _matrix{
    matrix(0,0), matrix(1,0), matrix(2,0),
    matrix(0,1), matrix(1,1), matrix(2,1),
    matrix(0,2), matrix(1,2), matrix(2,2)}
{ }

EndEffectorSO3Objective::EndEffectorSO3Objective(double weight, const Eigen::Matrix3d& matrix)
  : _weight(weight), _matrix{
    matrix(0,0), matrix(1,0), matrix(2,0),
    matrix(0,1), matrix(1,1), matrix(2,1),
    matrix(0,2), matrix(1,2), matrix(2,2)}
{ }
// clang-format on

HebiStatusCode EndEffectorSO3Objective::addObjective(HebiIKPtr ik) const {
  return hebiIKAddObjectiveFrameSO3(ik, static_cast<float>(_weight), HebiFrameTypeEndEffector, 0, _matrix, HebiMatrixOrderingColumnMajor);
}

EndEffectorTipAxisObjective::EndEffectorTipAxisObjective(const Eigen::Vector3d& obj)
  : _weight(1.0f), _x(obj[0]), _y(obj[1]), _z(obj[2]) {}

EndEffectorTipAxisObjective::EndEffectorTipAxisObjective(double weight, const Eigen::Vector3d& obj)
  : _weight(weight), _x(obj[0]), _y(obj[1]), _z(obj[2]) {}

HebiStatusCode EndEffectorTipAxisObjective::addObjective(HebiIKPtr ik) const {
  return hebiIKAddObjectiveFrameTipAxis(ik, static_cast<float>(_weight), HebiFrameTypeEndEffector, 0, _x, _y, _z);
}

JointLimitConstraint::JointLimitConstraint(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions,
                                           double effect_range)
  : _weight(1.0f), _min_positions(min_positions), _max_positions(max_positions), _effect_range(effect_range) {}

JointLimitConstraint::JointLimitConstraint(double weight, const Eigen::VectorXd& min_positions,
                                           const Eigen::VectorXd& max_positions, double effect_range)
  : _weight(weight), _min_positions(min_positions), _max_positions(max_positions), _effect_range(effect_range) {}

HebiStatusCode JointLimitConstraint::addObjective(HebiIKPtr ik) const {
  if (_min_positions.size() != _max_positions.size())
    return HebiStatusInvalidArgument;

  auto num_joints = _min_positions.size();
  auto res = hebiIKAddConstraintRampedJointAngles(ik, _weight, num_joints, _min_positions.data(), _max_positions.data(), _effect_range);

  return res;
}

////////////////////////// RobotModel

bool RobotModel::tryAdd(HebiRobotModelElementPtr element) {
  HebiStatusCode res = hebiRobotModelAdd(internal_, nullptr, 0, element);
  if (res == HebiStatusFailure) {
    hebiRobotModelElementRelease(element);
    return false;
  }
  return true;
}

RobotModel::RobotModel(HebiRobotModelPtr internal) : internal_(internal) {}

RobotModel::RobotModel() : internal_(hebiRobotModelCreate()) {}

// Helper function to validate the HRDF file/string that was loaded in
static bool validateHRDF(HebiRobotModelPtr model) {
  // nullptr return means an import error; print error and return empty
  // unique_ptr.
  if (model == nullptr) {
    fprintf(stderr, "HRDF Error: %s\n", hebiRobotModelGetImportError());
    return false;
  }

  // Display any relevant warnings.
  size_t num_warnings = hebiRobotModelGetImportWarningCount();
  for (size_t i = 0; i < num_warnings; ++i) {
    fprintf(stderr, "HRDF Warning: %s\n", hebiRobotModelGetImportWarning(i));
  }

  return true;
}

std::unique_ptr<RobotModel> RobotModel::loadHRDF(const std::string& file) {
  HebiRobotModelPtr internal = hebiRobotModelImport(file.c_str());
  if (!validateHRDF(internal)) {
    return nullptr;
  }

  // Create/return the robot model
  return std::unique_ptr<RobotModel>(new RobotModel(internal));
}

std::unique_ptr<RobotModel> RobotModel::loadHRDFString(const std::string& string) {
  HebiRobotModelPtr internal = hebiRobotModelImportBuffer(string.data(), string.size());
  if (!validateHRDF(internal)) {
    return nullptr;
  }

  // Create/return the robot model
  return std::unique_ptr<RobotModel>(new RobotModel(internal));
}

std::unique_ptr<RobotModel> RobotModel::createSubtree(size_t element_index) {
  HebiRobotModelPtr internal = hebiRobotModelCreateSubtreeFromElement(internal_, static_cast<int>(element_index));
  if (internal == nullptr)
    return {};
  return std::unique_ptr<RobotModel>(new RobotModel(internal));
}

RobotModel::~RobotModel() noexcept { hebiRobotModelRelease(internal_); }

void RobotModel::setBaseFrame(const Eigen::Matrix4d& base_frame) {
  // Put data into an array
  double transform[16];
  Map<Matrix<double, 4, 4>> tmp(transform);
  tmp = base_frame;
  hebiRobotModelSetBaseFrame(internal_, transform, HebiMatrixOrderingColumnMajor);
}

Eigen::Matrix4d RobotModel::getBaseFrame() const {
  // Get the data into an array
  double transform[16];
  hebiRobotModelGetBaseFrame(internal_, transform, HebiMatrixOrderingColumnMajor);

  // Copy out data
  Map<const Matrix<double, 4, 4>> tmp(transform);
  Eigen::Matrix4d res;
  res = tmp;
  return res;
}

size_t RobotModel::getFrameCount(FrameType frame_type) const {
  return hebiRobotModelGetNumberOfFrames(internal_, static_cast<HebiFrameType>(frame_type));
}

size_t RobotModel::getDoFCount() const { return hebiRobotModelGetNumberOfDoFs(internal_); }

std::string RobotModel::getMeshPath(size_t mesh_frame_index) const {
  size_t required_size{};
  auto res = hebiRobotModelGetMeshPath(internal_, mesh_frame_index, nullptr, &required_size);
  if (res == HebiStatusInvalidArgument)
    return "";
  std::string mesh_path(required_size, '\0');
  res = hebiRobotModelGetMeshPath(internal_, mesh_frame_index, &mesh_path[0], &required_size);
  mesh_path.pop_back(); // C API uses null character, so we drop it here.
  if (res != HebiStatusSuccess)
    return "";
  return mesh_path;
}

// TODO: handle trees/etc by passing in parent object here, and output index
bool RobotModel::addRigidBody(const Eigen::Matrix4d& com, const Eigen::VectorXd& inertia, double mass,
                              const Eigen::Matrix4d& output) {
  if (inertia.size() != 6)
    return false;

  // Allocate double arrays for C interop:
  double com_array[16];
  double inertia_array[6];
  double output_array[16];

  // Convert the data:
  {
    Map<Matrix<double, 4, 4>> tmp(com_array);
    tmp = com;
  }
  {
    Map<Eigen::VectorXd> tmp(inertia_array, 6);
    tmp = inertia;
  }
  {
    Map<Matrix<double, 4, 4>> tmp(output_array);
    tmp = output;
  }

  HebiRobotModelElementPtr body = hebiRobotModelElementCreateRigidBody(com_array, inertia_array, mass, 1, output_array,
                                                                       HebiMatrixOrderingColumnMajor);

  return tryAdd(body);
}

// TODO: handle trees/etc by passing in parent object here, and output index
bool RobotModel::addJoint(JointType joint_type) {
  return tryAdd(hebiRobotModelElementCreateJoint(static_cast<HebiJointType>(joint_type)));
}

bool RobotModel::addActuator(robot_model::ActuatorType actuator_type) {
  HebiRobotModelElementPtr element = hebiRobotModelElementCreateActuator(static_cast<HebiActuatorType>(actuator_type));
  if (element == nullptr)
    return false;
  return tryAdd(element);
}

bool RobotModel::addLink(robot_model::LinkType link_type, double extension, double twist, LinkInputType input_type,
                         LinkOutputType output_type) {
  HebiRobotModelElementPtr element =
      hebiRobotModelElementCreateLink(static_cast<HebiLinkType>(link_type), static_cast<HebiLinkInputType>(input_type),
                                      static_cast<HebiLinkOutputType>(output_type), extension, twist);
  if (element == nullptr)
    return false;
  return tryAdd(element);
}

bool RobotModel::addBracket(robot_model::BracketType bracket_type) {
  HebiRobotModelElementPtr element = hebiRobotModelElementCreateBracket(static_cast<HebiBracketType>(bracket_type));
  if (element == nullptr)
    return false;
  return tryAdd(element);
}

bool RobotModel::addEndEffector(EndEffectorType end_effector_type) {
  auto element = hebiRobotModelElementCreateEndEffector(static_cast<HebiEndEffectorType>(end_effector_type), nullptr,
                                                        nullptr, 0, nullptr, HebiMatrixOrderingColumnMajor);
  if (element == nullptr)
    return false;
  return tryAdd(element);
}

// A particular probe location:
bool RobotModel::addEndEffector(const Eigen::Matrix4d& com, const Eigen::VectorXd& inertia, double mass,
                                const Eigen::Matrix4d& output) {
  if (inertia.size() != 6)
    return false;

  // Allocate double arrays for C interop:
  double com_array[16];
  double inertia_array[6];
  double output_array[16];

  // Convert the data:
  {
    Map<Matrix<double, 4, 4>> tmp(com_array);
    tmp = com;
  }
  {
    Map<Eigen::VectorXd> tmp(inertia_array, 6);
    tmp = inertia;
  }
  {
    Map<Matrix<double, 4, 4>> tmp(output_array);
    tmp = output;
  }

  auto ee = hebiRobotModelElementCreateEndEffector(HebiEndEffectorTypeCustom, com_array, inertia_array, mass,
                                                   output_array, HebiMatrixOrderingColumnMajor);
  if (ee == nullptr)
    return false;
  return tryAdd(ee);
}

void RobotModel::getForwardKinematics(FrameType frame_type, const Eigen::VectorXd& positions,
                                      Matrix4dVector& frames) const {
  getFK(frame_type, positions, frames);
}

void RobotModel::getFK(FrameType frame_type, const Eigen::VectorXd& positions, Matrix4dVector& frames) const {
  // Put data into an array
  size_t num_frames = getFrameCount(frame_type);
  std::vector<double> frame_array(16 * num_frames);
  // Get data from C API
  hebiRobotModelGetForwardKinematics(internal_, static_cast<HebiFrameType>(frame_type), positions.data(), frame_array.data(),
                                     HebiMatrixOrderingColumnMajor);
  // Copy into vector of matrices passed in
  frames.resize(num_frames);
  for (size_t i = 0; i < num_frames; ++i) {
    Map<Matrix<double, 4, 4>> tmp(frame_array.data() + i * 16);
    frames[i] = tmp;
  }
}

void RobotModel::getEndEffector(const Eigen::VectorXd& positions, Eigen::Matrix4d& transform) const {
  double transform_array[16];
  hebiRobotModelGetForwardKinematics(internal_, HebiFrameTypeEndEffector, positions.data(), transform_array,
                                     HebiMatrixOrderingColumnMajor);
  {
    Map<Matrix<double, 4, 4>> tmp(transform_array);
    transform = tmp;
  }
}

void RobotModel::getJacobians(FrameType frame_type, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const {
  getJ(frame_type, positions, jacobians);
}
void RobotModel::getJ(FrameType frame_type, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const {
  size_t num_frames = getFrameCount(frame_type);
  size_t num_dofs = positions.size();
  size_t rows = 6 * num_frames;
  size_t cols = num_dofs;
  std::vector<double> jacobians_array(rows * cols);
  hebiRobotModelGetJacobians(internal_, static_cast<HebiFrameType>(frame_type), positions.data(), jacobians_array.data(),
                             HebiMatrixOrderingColumnMajor);
  jacobians.resize(num_frames);
  for (size_t i = 0; i < num_frames; ++i) {
    Map<Matrix<double, Dynamic, Dynamic>> tmp(jacobians_array.data() + i * cols * 6, 6, cols);
    jacobians[i] = tmp;
  }
}
void RobotModel::getJacobianEndEffector(const Eigen::VectorXd& positions, Eigen::MatrixXd& jacobian) const {
  getJEndEffector(positions, jacobian);
}
void RobotModel::getJEndEffector(const Eigen::VectorXd& positions, Eigen::MatrixXd& jacobian) const {
  MatrixXdVector tmp_jacobians;
  getJacobians(FrameType::EndEffector, positions, tmp_jacobians);

  // NOTE: could make this more efficient by writing additional lib function
  // for this, instead of tossing away almost everything from the full one!

  size_t num_dofs = positions.size();
  jacobian.resize(6, num_dofs);
  jacobian = *tmp_jacobians.rbegin();
}

void RobotModel::getMasses(Eigen::VectorXd& masses) const {
  size_t num_masses = getFrameCount(FrameType::CenterOfMass);
  masses.resize(num_masses);
  hebiRobotModelGetMasses(internal_, masses.data());
}

void RobotModel::getPayloads(Eigen::VectorXd& payloads) const noexcept {
  size_t num_payloads = getFrameCount(FrameType::EndEffector);
  payloads.resize(num_payloads);
  // Note -- no errors possible unless RobotModel object has become corrupt and has null internal_ value.
  hebiRobotModelGetEndEffectorPayloads(internal_, payloads.data());
}

double RobotModel::getPayload(size_t end_effector_index) const {
  double payload;
  if (hebiRobotModelGetEndEffectorPayload(internal_, end_effector_index, &payload) == HebiStatusArgumentOutOfRange) {
    throw std::out_of_range("getPayload called with invalid end effector index: " + std::to_string(end_effector_index));
  }
  // Note -- no other errors possible unless RobotModel object has become corrupt and has null internal_ value.
  return payload;
}

bool RobotModel::setPayloads(const Eigen::VectorXd& payloads) {
  size_t num_payloads = getFrameCount(FrameType::EndEffector);
  if (static_cast<size_t>(payloads.size()) != num_payloads) {
    throw std::invalid_argument("setPayloads called with incorrect number of payloads. Expected "
      + std::to_string(num_payloads) + ", got " + std::to_string(payloads.size()));
  }
  return hebiRobotModelSetEndEffectorPayloads(internal_, payloads.data()) == HebiStatusSuccess;
}

bool RobotModel::setPayload(size_t end_effector_index, double payload) {
  HebiStatusCode res = hebiRobotModelSetEndEffectorPayload(internal_, end_effector_index, payload);
  if (res == HebiStatusArgumentOutOfRange) {
    throw std::out_of_range("setPayload called with invalid end effector index: " + std::to_string(end_effector_index));
  }
  return res == HebiStatusSuccess;
}

Eigen::Vector3d RobotModel::getPayloadCenterOfMass(size_t end_effector_index) {
  Eigen::Vector3d res;
  if (hebiRobotModelGetEndEffectorPayloadCenterOfMass(internal_, end_effector_index, res.data()) == HebiStatusArgumentOutOfRange) {
    throw std::out_of_range("getPayloadCenterOfMass called with invalid end effector index: " + std::to_string(end_effector_index));
  }
  return res;
}

void RobotModel::setPayloadCenterOfMass(size_t end_effector_index, const Eigen::Vector3d& com) {
  HebiStatusCode res = hebiRobotModelSetEndEffectorPayloadCenterOfMass(internal_, end_effector_index, com.data());
  if (res == HebiStatusArgumentOutOfRange) {
    throw std::out_of_range("getPayloadCenterOfMass called with invalid end effector index: " + std::to_string(end_effector_index));
  } else if (res == HebiStatusInvalidArgument) {
    throw std::invalid_argument("getPayloadCenterOfMass called with non-finite CoM");
  }
}

void RobotModel::getMetadata(std::vector<MetadataBase>& metadata) const {
  const auto num_elems = hebiRobotModelGetNumberOfElements(internal_);
  metadata.resize(num_elems);
  for (size_t i = 0; i < num_elems; ++i) {
    hebiRobotModelGetElementMetadata(internal_, i, &metadata[i].metadata_);
  }
}

void RobotModel::getMaxSpeeds(Eigen::VectorXd& max_speeds) const {
  size_t num_actuators = getDoFCount();
  max_speeds.resize(num_actuators);
  hebiRobotModelGetMaxSpeeds(internal_, max_speeds.data());
}
void RobotModel::getMaxEfforts(Eigen::VectorXd& max_efforts) const {
  size_t num_actuators = getDoFCount();
  max_efforts.resize(num_actuators);
  hebiRobotModelGetMaxEfforts(internal_, max_efforts.data());
}

void RobotModel::getGravCompEfforts(const Eigen::VectorXd& position, const Eigen::Vector3d& gravity,
                                    Eigen::VectorXd& comp_torque) const {
  comp_torque.resize(getDoFCount());
  comp_torque.setZero();
  // Use an API function to return the grav comp torques.  The calculation is the sum of:
  //   J^T * -gravity * mass
  // for each mass-containing element in the robot (e.g., anything that has a "center of mass" frame, and also
  // any end effector payloads)
  hebiRobotModelGetGravityCompensationTorques(internal_, position.data(), gravity.data(), comp_torque.data());
}

void RobotModel::getDynamicCompEfforts(const Eigen::VectorXd& position, const Eigen::VectorXd& cmd_pos, const Eigen::VectorXd& cmd_vel,
                                       const Eigen::VectorXd& cmd_accel, Eigen::VectorXd& comp_torque, double /*dt*/) const {
  comp_torque.resize(getDoFCount());
  comp_torque.setZero();
  // Use an API function to return the grav comp torques.  The calculation is the sum of:
  //   J^T * acceleration * mass
  // for each mass-containing element in the robot (e.g., anything that has a "center of mass" frame, and also
  // any end effector payloads).
  // The "acceleration" here is a wrench of linear accelerations for each frame, calculated by differentiating
  // the FK generated by the commanded position, velocity, and accelerations. We extrapolate to the previous and
  // next commands along the quadratic by using:
  //   cmd_prev = cmd_pos - cmd_vel * dt + 0.5 * cmd_accel * dt^2
  //   cmd_next = cmd_pos + cmd_vel * dt + 0.5 * cmd_accel * dt^2
  // and then use the differentiated translational component of the FK at these positions as the acceleration:
  //   acceleration = fk_next.pos + fk_prev.pos - 2 * fk_curr.pos / dt^2
  //
  // Note that this ignores rotational inertia effects at the current time, so the acceleration wrench used has
  // zeros for the rx/ry/rz components.
  hebiRobotModelGetDynamicsCompensationTorques(internal_, position.data(), cmd_pos.data(), cmd_vel.data(), cmd_accel.data(), comp_torque.data());
}

} // namespace robot_model
} // namespace hebi
