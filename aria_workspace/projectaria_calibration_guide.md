# ProjectARIA 标定系统使用指南

## 概述

ProjectARIA 标定系统提供了一套完整的传感器标定功能，支持多种传感器类型包括相机、IMU、磁力计、气压计和麦克风。该系统通过内参和外参标定，实现高精度的多传感器数据融合。

## 核心类说明

### 1. 设备标定类 (DeviceCalibration)

设备标定类是整个标定系统的核心，用于存储和访问设备的所有传感器标定信息。

```python
# 构造函数参数
DeviceCalibration(
    camera_calibs: dict[str, CameraCalibration],      # 相机标定字典
    imu_calibs: dict[str, ImuCalibration],            # IMU标定字典
    magnetometer_calibs: dict[str, MagnetometerCalibration],  # 磁力计标定字典
    barometer_calibs: dict[str, BarometerCalibration], # 气压计标定字典
    microphone_calibs: dict[str, MicrophoneCalibration], # 麦克风标定字典
    device_cad_extrinsics: DeviceCadExtrinsics,       # 设备CAD外参
    device_subtype: str,                              # 设备子类型 (如 "DVT-S", "DVT-L")
    origin_label: str                                 # 原点标签，定义设备坐标系
)
```

**主要方法：**
- `get_camera_calib(label)`: 获取指定相机标定
- `get_imu_calib(label)`: 获取指定IMU标定
- `get_transform_device_sensor(label)`: 获取设备到传感器的变换矩阵
- `get_aria_et_camera_calib()`: 获取Aria眼动追踪相机标定（返回左右眼数组）
- `get_aria_microphone_calib()`: 获取Aria麦克风标定（返回7个麦克风数组）

### 2. 相机标定类 (CameraCalibration)

相机标定类提供完整的相机内参、外参和投影功能。

```python
# 构造函数参数
CameraCalibration(
    label: str,                                       # 相机标签
    projection_model_type: CameraModelType,          # 投影模型类型
    projection_params: numpy.ndarray,                # 投影参数
    T_Device_Camera: SE3,                            # 设备到相机的变换
    image_width: int,                                # 图像宽度
    image_height: int,                               # 图像高度
    maybe_valid_radius: float | None,               # 有效半径（可选）
    max_solid_angle: float,                          # 最大立体角
    serial_number: str                               # 序列号
)
```

**投影模型类型 (CameraModelType)：**
- `LINEAR`: 线性针孔投影模型
- `SPHERICAL`: 球面投影模型  
- `KANNALA_BRANDT_K3`: Kannala-Brandt K3模型（球面+多项式径向畸变）
- `FISHEYE624`: 鱼眼模型（球面+径向+切向畸变）

**主要方法：**
- `project(point_in_camera)`: 3D点投影到2D像素（含有效性检查）
- `unproject(camera_pixel)`: 2D像素反投影到3D射线（含有效性检查）
- `project_no_checks()` / `unproject_no_checks()`: 无检查版本
- `is_visible(camera_pixel)`: 检查像素是否在有效区域
- `get_focal_lengths()`: 获取焦距
- `get_principal_point()`: 获取主点
- `rescale()`: 重新缩放标定参数

### 3. IMU标定类 (ImuCalibration)

IMU标定包含加速度计和陀螺仪的内参模型。

```python
# 构造函数参数
ImuCalibration(
    label: str,                                      # IMU标签
    accel_rectification: numpy.ndarray[3,3],         # 加速度计校正矩阵
    accel_bias: numpy.ndarray[3,1],                  # 加速度计偏置
    gyro_rectification: numpy.ndarray[3,3],          # 陀螺仪校正矩阵  
    gyro_bias: numpy.ndarray[3,1],                   # 陀螺仪偏置
    T_Device_Imu: SE3                                # 设备到IMU的变换
)
```

**主要方法：**
- `raw_to_rectified_accel(raw)`: 原始加速度数据转校正数据
- `raw_to_rectified_gyro(raw)`: 原始陀螺仪数据转校正数据
- `rectified_to_raw_accel(rectified)`: 校正数据转原始数据（仿真用）
- `rectified_to_raw_gyro(rectified)`: 校正数据转原始数据（仿真用）

### 4. 其他传感器标定类

**磁力计标定 (MagnetometerCalibration)：**
```python
MagnetometerCalibration(label, rectification_matrix, bias)
```

**气压计标定 (BarometerCalibration)：**
```python
BarometerCalibration(label, scale, offset)
```

**麦克风标定 (MicrophoneCalibration)：**
```python
MicrophoneCalibration(label, sensitivity)
```

## 工具函数

### 标定加载函数

```python
# 从JSON文件加载设备标定
device_calib = device_calibration_from_json("calibration.json")

# 从JSON字符串加载设备标定  
device_calib = device_calibration_from_json_string(json_string)
```

### 图像畸变校正函数

```python
# 图像畸变校正
corrected_image = distort_by_calibration(
    source_image,           # 源图像
    dst_calibration,        # 目标标定
    src_calibration,        # 源标定
    interpolation_method    # 插值方法（可选）
)

# 深度图像畸变校正
corrected_depth = distort_depth_by_calibration(depth_image, dst_calib, src_calib)

# 标签图像畸变校正（使用最近邻插值）
corrected_labels = distort_label_by_calibration(label_image, dst_calib, src_calib)
```

### 简单标定创建函数

```python
# 创建线性相机标定
linear_calib = get_linear_camera_calibration(
    image_width=1280,
    image_height=720, 
    focal_length=500.0,
    label="camera-rgb",
    T_Device_Camera=SE3()  # 可选
)

# 创建球面相机标定
spherical_calib = get_spherical_camera_calibration(
    image_width=1280,
    image_height=720,
    focal_length=500.0
)

# 顺时针旋转90度（仅支持线性模型）
rotated_calib = rotate_camera_calib_cw90deg(camera_calibration)
```

## 使用示例

### 1. 加载和使用设备标定

```python
import numpy as np
from projectaria_tools.core.calibration import *

# 加载设备标定
device_calib = device_calibration_from_json("aria_calibration.json")

# 获取相机标定
left_cam = device_calib.get_camera_calib("camera-slam-left")
right_cam = device_calib.get_camera_calib("camera-slam-right")

# 获取相机内参
focal_lengths = left_cam.get_focal_lengths()  # [fx, fy]
principal_point = left_cam.get_principal_point()  # [cx, cy]
image_size = left_cam.get_image_size()  # [width, height]

print(f"焦距: {focal_lengths}")
print(f"主点: {principal_point}")
print(f"图像尺寸: {image_size}")
```

### 2. 3D点投影和反投影

```python
# 3D点投影到像素
point_3d = np.array([0.0, 0.0, 1.0])  # 相机坐标系下的点
pixel = left_cam.project(point_3d)

if pixel is not None:
    print(f"投影像素: {pixel}")
    
    # 反投影回3D射线
    ray_3d = left_cam.unproject(pixel)
    if ray_3d is not None:
        print(f"反投影射线: {ray_3d}")
```

### 3. IMU数据校正

```python
# 获取IMU标定
imu_calib = device_calib.get_imu_calib("imu-right")

# 校正IMU原始数据
raw_accel = np.array([9.8, 0.0, 0.0])  # 原始加速度
raw_gyro = np.array([0.0, 0.0, 0.1])   # 原始角速度

corrected_accel = imu_calib.raw_to_rectified_accel(raw_accel)
corrected_gyro = imu_calib.raw_to_rectified_gyro(raw_gyro)

print(f"校正后加速度: {corrected_accel}")
print(f"校正后角速度: {corrected_gyro}")
```

### 4. 坐标系变换

```python
# 获取传感器间的变换关系
T_device_camera = device_calib.get_transform_device_sensor("camera-slam-left")
T_device_imu = device_calib.get_transform_device_sensor("imu-right")

# 计算相机到IMU的变换
T_camera_imu = T_device_camera.inverse() * T_device_imu

print(f"相机到IMU变换: {T_camera_imu}")
```

### 5. Aria专用功能

```python
# 获取Aria眼动追踪相机（左右眼）
et_calibs = device_calib.get_aria_et_camera_calib()
if et_calibs:
    left_et = et_calibs[0]
    right_et = et_calibs[1]
    print(f"左眼ET相机: {left_et.get_label()}")
    print(f"右眼ET相机: {right_et.get_label()}")

# 获取Aria麦克风阵列（7个麦克风）
mic_calibs = device_calib.get_aria_microphone_calib()
if mic_calibs:
    for i, mic in enumerate(mic_calibs):
        print(f"麦克风{i}: {mic.get_label()}")
```

## 注意事项

1. **坐标系定义**: 所有外参都是相对于设备坐标系定义的，设备坐标系由`origin_label`指定的传感器确定。

2. **有效性检查**: 使用`project()`和`unproject()`时会进行有效性检查，包括视野范围和有效半径检查。如需性能优化可使用`*_no_checks()`版本。

3. **畸变校正**: 畸变校正函数支持不同数据类型（uint8, float32, uint16, uint64）和多种插值方法。

4. **Aria设备**: 对于Aria设备，设备子类型为"DVT-S"或"DVT-L"，分别表示小型和大型Aria单元。

5. **线性模型限制**: 某些功能（如旋转）仅支持线性投影模型。

## 传感器类型枚举

```python
# 传感器标定类型
SensorCalibrationType.CAMERA_CALIBRATION      # 相机标定
SensorCalibrationType.IMU_CALIBRATION         # IMU标定  
SensorCalibrationType.MAGNETOMETER_CALIBRATION # 磁力计标定
SensorCalibrationType.BAROMETER_CALIBRATION   # 气压计标定
SensorCalibrationType.MICROPHONE_CALIBRATION  # 麦克风标定
SensorCalibrationType.ARIA_ET_CALIBRATION     # Aria眼动追踪标定
SensorCalibrationType.ARIA_MIC_CALIBRATION    # Aria麦克风阵列标定
```

这套标定系统为ProjectARIA多传感器平台提供了完整的标定解决方案，支持高精度的传感器数据处理和多传感器融合应用。
