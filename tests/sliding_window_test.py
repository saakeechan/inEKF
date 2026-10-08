"""Compile and exercise the actual window function without ROS/Pinocchio/ONNX."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'src/estimatorManager/InEKF_manager.cpp').read_text()
function = source.split('void INEKFilter_manager::sliding_window()', 1)[1].split(
    'void INEKFilter_manager::Onestep(', 1)[0]
fields = [
    'Position_s', 'Rotation_s', 'Velocity_s', 'Bias_Gyro_s', 'Bias_Acc_s',
    'HARD_CONTACT_t', 'CONTACT_t', 'SLIP_t', 'IMU_Gyro', 'IMU_Acc',
    'ENCODER', 'ENCODERDOT', 'MOTOR_TORQUES',
    'd_v', 'dv_norm',
]
contacts = {'HARD_CONTACT_t', 'CONTACT_t', 'SLIP_t'}
code = '''#include <eigen3/Eigen/Dense>
#include <vector>
#include <limits>
#include <cassert>
#include <cmath>
struct INEKFilter_manager {
    int frame_count = 0;
    int sliding_window_size = 2;
    struct { struct { int num_legs = 2; } model; } robot;
    void sliding_window();
'''
for field in fields:
    kind = ('std::vector<bool>' if field in contacts else
            'Eigen::Matrix3d' if field == 'Rotation_s' else 'Eigen::VectorXd')
    code += f'    std::vector<{kind}> {field};\n'
code += '};\nvoid INEKFilter_manager::sliding_window()' + function
code += '\nint main() { for (int n : {2, 3, 5}) { INEKFilter_manager f; f.sliding_window_size = n;\n'
for field in fields:
    code += f'f.{field}.resize(n);\n'
    if field in contacts:
        code += f'for (int i=0; i<n; ++i) f.{field}[i].assign(2, i%2);\n'
    elif field == 'Rotation_s':
        code += f'for (int i=0; i<n; ++i) f.{field}[i].setConstant(i+1);\n'
    else:
        code += f'for (int i=0; i<n; ++i) f.{field}[i] = Eigen::VectorXd::Constant(6, i+1);\n'
code += 'f.sliding_window(); assert(f.frame_count == n-1);\n'
for field in fields:
    if field in contacts:
        code += f'for (int i=0; i<n-1; ++i) assert(f.{field}[i][0] == bool((i+1)%2));\n'
        code += f'assert(f.{field}.back() == std::vector<bool>(2, false));\n'
    elif field == 'Rotation_s':
        code += f'for (int i=0; i<n-1; ++i) assert(f.{field}[i](0,0) == i+2);\n'
        code += f'assert(f.{field}.back().isIdentity());\n'
    else:
        code += f'for (int i=0; i<n-1; ++i) assert(f.{field}[i](0) == i+2);\n'
        code += f'assert(f.{field}.back().size() == 6);\n'
        code += (f'assert(std::isinf(f.{field}.back()(0)));\n' if field in {'d_v', 'dv_norm'}
                 else f'assert(f.{field}.back().isZero());\n')
code += '''f.d_v.back().setConstant(99);
f.sliding_window();
assert(f.d_v[n-2](0) == 99);
assert(std::isinf(f.d_v.back()(0)));
} }
'''
with tempfile.TemporaryDirectory() as temp:
    cpp = Path(temp) / 'test.cpp'
    binary = Path(temp) / 'test'
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++17', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('Window shifts and resets passed for N=2, 3, 5.')
