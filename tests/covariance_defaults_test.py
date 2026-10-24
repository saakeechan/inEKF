"""Exercise actual robot covariance setup without the Pinocchio dependency."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'src/utility/RobotModel.cpp').read_text()
header = (root / 'src/utility/RobotModel.hpp').read_text()
function = source.split('void RobotModel::SetCovariances()', 1)[1].split(
    '///////////////////////////////////////////////////////////////////////////////////////////', 1)[0]
constants = re.findall(r'double (cov_\w+);', header)
matrices = re.findall(r'Eigen::Matrix<double,3,3>\s+(\w+);', header)
code = '''#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <cassert>
#include <cmath>
struct RobotModel {
    YAML::Node config;
    void SetCovariances();
'''
code += ''.join(f'double {name};\n' for name in constants)
code += ''.join(f'Eigen::Matrix3d {name};\n' for name in matrices)
code += '};\nvoid RobotModel::SetCovariances()' + function
code += '''
int main() {
    RobotModel r;
    r.config = YAML::Load("{}");
    r.SetCovariances();
    assert(r.cov_gyro_const == 1e-4 && r.cov_acc_const == 1e-2);
    assert(r.cov_contact_const == 1e-3 && r.cov_enc_const == 1e-4);
    assert(r.cov_bias_gyro_const == 1e-6);
    assert(r.Covariance_Contact.isApprox(1e-3 * Eigen::Matrix3d::Identity()));
    r.config = YAML::Load("covariance: {gyro: 0.04, contact: 0.09}");
    r.SetCovariances();
    assert(r.cov_gyro_const == 0.04 && r.cov_contact_const == 0.09);
    assert(r.cov_acc_const == 1e-2);
    assert(r.Covariance_Gyro.isApprox(0.04 * Eigen::Matrix3d::Identity()));
    r.config = YAML::Load("covariance: {}");
    r.SetCovariances();
    assert(r.cov_gyro_const == 1e-4 && r.cov_contact_const == 1e-3);
}
'''
with tempfile.TemporaryDirectory() as temporary:
    cpp = Path(temporary) / 'test.cpp'
    binary = Path(temporary) / 'test'
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++17', str(cpp), '-lyaml-cpp', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('Robot YAML defaults, partial overrides, matrices, and reload defaults passed.')
