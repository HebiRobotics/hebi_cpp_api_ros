#include "group_info.hpp"

namespace hebi {

GroupInfo::GroupInfo(size_t number_of_modules)
  : internal_(std::make_shared<GroupInfoWrapper>(number_of_modules)), number_of_modules_(number_of_modules) {
  for (size_t i = 0; i < number_of_modules_; i++)
    infos_.emplace_back(hebiGroupInfoGetModuleInfo(internal_->internal_, i));
}

GroupInfo::GroupInfo(std::shared_ptr<GroupInfoWrapper> internal, std::vector<int> indices)
  : internal_(std::move(internal)), number_of_modules_(indices.size()), is_subview_(true) {
  for (auto i : indices)
    infos_.emplace_back(hebiGroupInfoGetModuleInfo(internal_->internal_, i));
}

GroupInfo GroupInfo::subview(std::vector<int> indices) const {
  for (auto i : indices) {
    if (i < 0 || i >= number_of_modules_)
      throw std::out_of_range("Invalid index when creating subview.");
  }
  return GroupInfo(internal_, indices);
}

size_t GroupInfo::size() const { return number_of_modules_; }

const Info& GroupInfo::operator[](size_t index) const { return infos_[index]; }

bool GroupInfo::writeGains(const std::string& file) const {
  if (is_subview_)
    return false;
  return hebiGroupInfoWriteGains(internal_->internal_, file.c_str()) == HebiStatusSuccess;
}

Eigen::VectorXd GroupInfo::getSpringConstant() const {
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    auto& info = infos_[i].settings().actuator().springConstant();
    res[i] = (info) ? info.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

void GroupInfo::getSpringConstant(Eigen::VectorXd& out) const {
  if (out.size() != number_of_modules_) {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i) {
    auto& info = infos_[i].settings().actuator().springConstant();
    out[i] = (info) ? info.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

} // namespace hebi
