"""Compile and exercise the actual contact struct and window without robot dependencies."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'src/estimatorManager/InEKF_manager.cpp').read_text()
function = source.split('void INEKFilter_manager::sliding_window()', 1)[1].split(
    'void INEKFilter_manager::Onestep(', 1)[0]
header = (root / 'src/estimatorManager/InEKF_manager.hpp').read_text()
contact_struct = 'struct ContactState {' + header.split('struct ContactState {', 1)[1].split(
    'struct KinematicsMeasurement', 1)[0]
fields = [
    'Position_s', 'Rotation_s', 'Velocity_s', 'Bias_Gyro_s', 'Bias_Acc_s',
    'IMU_Gyro', 'IMU_Acc', 'ENCODER', 'ENCODERDOT', 'MOTOR_TORQUES',
]
flags = ['contact', 'hard_contact', 'slip']
code = '''#include <eigen3/Eigen/Dense>
#include <vector>
#include <limits>
#include <cassert>
#include <cmath>
''' + contact_struct + '''struct INEKFilter_manager {
    int frame_count = 0;
    int sliding_window_size = 2;
    struct { struct { int num_legs = 2; } model; } robot;
    std::vector<ContactState> CONTACT_s;
    void sliding_window();
'''
for field in fields:
    kind = 'Eigen::Matrix3d' if field == 'Rotation_s' else 'Eigen::VectorXd'
    code += f'    std::vector<{kind}> {field};\n'
code += '};\nvoid INEKFilter_manager::sliding_window()' + function
code += '\nint main() { for (int n : {2, 3, 5}) { INEKFilter_manager f; f.sliding_window_size = n;\n'
for field in fields:
    code += f'f.{field}.resize(n);\n'
    if field == 'Rotation_s':
        code += f'for (int i=0; i<n; ++i) f.{field}[i].setConstant(i+1);\n'
    else:
        code += f'for (int i=0; i<n; ++i) f.{field}[i] = Eigen::VectorXd::Constant(6, i+1);\n'
code += 'f.CONTACT_s.resize(n);\nfor (int i=0; i<n; ++i) f.CONTACT_s[i].reset(2, i+1);\n'
for offset, flag in enumerate(flags):
    code += f'for (int i=0; i<n; ++i) for (int leg=0; leg<2; ++leg) f.CONTACT_s[i].{flag}[leg] = (i+leg+{offset})%2;\n'
code += 'f.sliding_window(); assert(f.frame_count == n-1);\n'
for field in fields:
    if field == 'Rotation_s':
        code += f'for (int i=0; i<n-1; ++i) assert(f.{field}[i](0,0) == i+2);\n'
        code += f'assert(f.{field}.back().isIdentity());\n'
    else:
        code += f'for (int i=0; i<n-1; ++i) assert(f.{field}[i](0) == i+2);\n'
        code += f'assert(f.{field}.back().size() == 6);\n'
        code += f'assert(f.{field}.back().isZero());\n'
for offset, flag in enumerate(flags):
    code += f'for (int i=0; i<n-1; ++i) for (int leg=0; leg<2; ++leg) assert(f.CONTACT_s[i].{flag}[leg] == bool((i+1+leg+{offset})%2));\n'
    code += f'assert(f.CONTACT_s.back().{flag} == std::vector<bool>(2, false));\n'
for field, size in [('d_v', 6), ('dv_norm', 2)]:
    code += f'for (int i=0; i<n-1; ++i) assert(f.CONTACT_s[i].{field}.isApprox(Eigen::VectorXd::Constant({size}, i+2)));\n'
    code += f'assert(f.CONTACT_s.back().{field}.size() == {size});\n'
    code += f'assert(f.CONTACT_s.back().{field}.array().isInf().all());\n'
code += '''f.CONTACT_s.back().d_v.setConstant(99);
f.sliding_window();
assert(f.CONTACT_s[n-2].d_v(0) == 99);
assert(std::isinf(f.CONTACT_s.back().d_v(0)));
f.CONTACT_s.back().contact.assign(2, true);
f.CONTACT_s.back().reset(2, std::numeric_limits<double>::quiet_NaN());
assert(f.CONTACT_s.back().contact == std::vector<bool>(2, false));
assert(f.CONTACT_s.back().d_v.array().isNaN().all());
assert(f.CONTACT_s.back().dv_norm.array().isNaN().all());
} }
'''
with tempfile.TemporaryDirectory() as temp:
    cpp = Path(temp) / 'test.cpp'
    binary = Path(temp) / 'test'
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++17', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('Contact-state and window shifts/resets passed for N=2, 3, 5.')
