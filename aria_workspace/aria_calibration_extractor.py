#!/usr/bin/env python3
"""
Aria Stereo Extrinsics Extractor
Ultra-precise extractor for stereo camera extrinsics from Aria device_calibration.json.
Generates dual coordinate system versions: SLAM-LEFT origin and cam0 origin.
"""

import json
import numpy as np
from typing import Dict, List, Tuple, Optional
from pathlib import Path
from dataclasses import dataclass
from enum import Enum


class CoordinateSystem(Enum):
    """Coordinate system reference frames"""
    SLAM_LEFT_ORIGIN = "slam_left_origin"  # SLAM-LEFT camera as world origin
    CAM0_ORIGIN = "cam0_origin"           # First camera in pair as origin


@dataclass
class CameraPose:
    """6DOF camera pose representation"""
    label: str
    translation: np.ndarray      # [x, y, z] in meters (float64)
    rotation_matrix: np.ndarray  # 3x3 rotation matrix (float64)
    
    def to_homogeneous_matrix(self) -> np.ndarray:
        """Convert to 4x4 homogeneous transformation matrix"""
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.rotation_matrix
        T[:3, 3] = self.translation
        return T
    
    def inverse(self) -> 'CameraPose':
        """Compute inverse pose"""
        R_inv = self.rotation_matrix.T
        t_inv = -R_inv @ self.translation
        return CameraPose(f"inv_{self.label}", t_inv, R_inv)


@dataclass
class StereoPair:
    """Stereo camera pair configuration"""
    cam0_label: str
    cam1_label: str
    description: str
    
    def get_pair_name(self) -> str:
        """Generate standardized pair name"""
        # Normalize camera labels for consistent naming
        cam0_short = self.cam0_label.replace('camera-', '').replace('-', '_')
        cam1_short = self.cam1_label.replace('camera-', '').replace('-', '_')
        return f"{cam0_short}_{cam1_short}"


class AriaExtrinsicsProcessor:
    """
    Professional-grade processor for Aria stereo extrinsics extraction.
    Implements dual coordinate system generation with mathematical precision.
    """
    
    # Standard stereo pairs for SLAM and RGB cameras
    STANDARD_PAIRS = [
        StereoPair("camera-slam-left", "camera-rgb", "High-precision near-field stereo"),
        StereoPair("camera-slam-left", "camera-slam-right", "Primary SLAM stereo system"),
        StereoPair("camera-rgb", "camera-slam-right", "RGB-enhanced stereo vision"),
    ]
    
    def __init__(self, calibration_path: str):
        """
        Initialize with Aria factory calibration.
        
        Args:
            calibration_path: Path to device_calibration.json
        """
        self.calibration_path = Path(calibration_path)
        self._load_and_validate_calibration()
        self._extract_camera_poses()
    
    def _load_and_validate_calibration(self) -> None:
        """Load and validate Aria calibration JSON with comprehensive checks"""
        try:
            with open(self.calibration_path, 'r', encoding='utf-8') as f:
                self.calib_data = json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Calibration file not found: {self.calibration_path}")
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON format in calibration file: {e}")
        
        # Validate essential structure
        required_fields = ['CameraCalibrations', 'DeviceClassInfo', 'Serial']
        for field in required_fields:
            if field not in self.calib_data:
                raise ValueError(f"Missing required field '{field}' in calibration data")
        
        # Extract device metadata
        self.device_serial = self.calib_data['Serial']
        self.device_type = self.calib_data['DeviceClassInfo']['BuildVersion']
        self.calibration_source = self.calib_data.get('CalibrationSource', 'Unknown')
        
        print(f"✓ Loaded calibration for {self.device_type} device: {self.device_serial}")
    
    def _quaternion_to_rotation_matrix(self, quaternion: List) -> np.ndarray:
        """
        Convert Aria quaternion [w, [x, y, z]] to 3x3 rotation matrix.
        Uses Shepperd's method for numerical stability.
        
        Args:
            quaternion: [scalar, [vector]] format from Aria calibration
            
        Returns:
            3x3 orthonormal rotation matrix (float64)
        """
        w = float(quaternion[0])
        x, y, z = [float(v) for v in quaternion[1]]
        
        # Normalize quaternion to unit length for stability
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm < 1e-12:
            raise ValueError(f"Degenerate quaternion with norm {norm}")
        
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        # Convert to rotation matrix using optimized formula
        # Avoids redundant multiplications compared to naive approach
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        
        R = np.array([
            [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
            [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
        ], dtype=np.float64)
        
        # Verify orthonormality (optional but recommended for production)
        if not np.allclose(R @ R.T, np.eye(3), atol=1e-10):
            raise ValueError("Generated rotation matrix is not orthonormal")
        
        return R
    
    def _extract_camera_poses(self) -> None:
        """Extract 6DOF poses for all cameras from calibration data"""
        self.camera_poses = {}
        
        for cam_calib in self.calib_data['CameraCalibrations']:
            label = cam_calib['Label']
            T_device_camera = cam_calib['T_Device_Camera']
            
            # Extract translation and rotation
            translation = np.array(T_device_camera['Translation'], dtype=np.float64)
            rotation_matrix = self._quaternion_to_rotation_matrix(T_device_camera['UnitQuaternion'])
            
            # Store pose
            pose = CameraPose(label, translation, rotation_matrix)
            self.camera_poses[label] = pose
            
        print(f"✓ Extracted poses for {len(self.camera_poses)} cameras: {list(self.camera_poses.keys())}")
    
    def _compute_relative_extrinsics(self, cam0_label: str, cam1_label: str, 
                                   coordinate_system: CoordinateSystem) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute stereo extrinsics in specified coordinate system.
        
        Args:
            cam0_label: Reference camera label
            cam1_label: Target camera label  
            coordinate_system: Coordinate system reference frame
            
        Returns:
            Tuple of (T_cam0, T_cam1) - 4x4 transformation matrices
        """
        if cam0_label not in self.camera_poses:
            raise ValueError(f"Camera '{cam0_label}' not found in calibration")
        if cam1_label not in self.camera_poses:
            raise ValueError(f"Camera '{cam1_label}' not found in calibration")
        
        pose0 = self.camera_poses[cam0_label]
        pose1 = self.camera_poses[cam1_label]
        
        if coordinate_system == CoordinateSystem.SLAM_LEFT_ORIGIN:
            # Use SLAM-LEFT as world origin
            if 'camera-slam-left' not in self.camera_poses:
                raise ValueError("SLAM-LEFT camera not found, cannot use as origin")
            
            slam_left_pose = self.camera_poses['camera-slam-left']
            T_world_slam_left = slam_left_pose.to_homogeneous_matrix()
            T_slam_left_world = np.linalg.inv(T_world_slam_left)
            
            # Transform both cameras to SLAM-LEFT coordinate system
            T_device_cam0 = pose0.to_homogeneous_matrix()
            T_device_cam1 = pose1.to_homogeneous_matrix()
            
            T_slam_left_cam0 = T_slam_left_world @ T_device_cam0
            T_slam_left_cam1 = T_slam_left_world @ T_device_cam1
            
            return T_slam_left_cam0, T_slam_left_cam1
            
        elif coordinate_system == CoordinateSystem.CAM0_ORIGIN:
            # Use cam0 as origin (cam0 = identity, cam1 = relative)
            T_device_cam0 = pose0.to_homogeneous_matrix()
            T_device_cam1 = pose1.to_homogeneous_matrix()
            
            # Compute relative transformation
            T_cam0_device = np.linalg.inv(T_device_cam0)
            T_cam0_cam1 = T_device_cam1 @ T_cam0_device
            
            T_identity = np.eye(4, dtype=np.float64)
            
            return T_identity, T_cam0_cam1
        
        else:
            raise ValueError(f"Unsupported coordinate system: {coordinate_system}")
    
    def _format_transformation_matrix(self, T: np.ndarray, precision: int = 9) -> List[List[float]]:
        """
        Format transformation matrix for YAML output with controlled precision.
        
        Args:
            T: 4x4 transformation matrix
            precision: Decimal precision for output
            
        Returns:
            Nested list suitable for YAML serialization
        """
        formatted_matrix = []
        for row in T:
            formatted_row = []
            for val in row:
                # Use high precision for non-negligible values
                if abs(val) > 1e-12:
                    formatted_row.append(round(float(val), precision))
                else:
                    formatted_row.append(0.0)
            formatted_matrix.append(formatted_row)
        
        return formatted_matrix
    
    def generate_stereo_extrinsics(self, cam0_label: str, cam1_label: str, 
                                  coordinate_system: CoordinateSystem) -> Dict:
        """
        Generate stereo extrinsics configuration.
        
        Args:
            cam0_label: Primary camera label
            cam1_label: Secondary camera label
            coordinate_system: Reference coordinate system
            
        Returns:
            Stereo extrinsics configuration dictionary
        """
        # Compute transformations
        T_cam0, T_cam1 = self._compute_relative_extrinsics(cam0_label, cam1_label, coordinate_system)
        
        # Calculate baseline distance
        baseline_vector = T_cam1[:3, 3] - T_cam0[:3, 3]
        baseline_distance = float(np.linalg.norm(baseline_vector))
        
        # Determine coordinate system description
        if coordinate_system == CoordinateSystem.SLAM_LEFT_ORIGIN:
            coord_description = "SLAM-LEFT camera as world origin"
            origin_camera = "camera-slam-left"
        else:
            coord_description = f"{cam0_label} as world origin"
            origin_camera = cam0_label
        
        # Build configuration
        config = {
            'metadata': {
                'coordinate_system': coordinate_system.value,
                'description': coord_description,
                'origin_camera': origin_camera,
                'camera_pair': [cam0_label, cam1_label],
                'baseline_distance_m': round(baseline_distance, 6),
                'baseline_distance_cm': round(baseline_distance * 100, 2),
                'device_info': {
                    'serial': self.device_serial,
                    'type': self.device_type,
                    'calibration_source': self.calibration_source
                }
            },
            
            'extrinsics': {
                'cam0': {
                    'label': cam0_label,
                    'T_world_camera': self._format_transformation_matrix(T_cam0),
                    'T_type': 0  # Euroc format
                },
                'cam1': {
                    'label': cam1_label,
                    'T_world_camera': self._format_transformation_matrix(T_cam1),
                    'T_type': 0  # Euroc format
                }
            }
        }
        
        return config
    
    def export_stereo_pair(self, stereo_pair: StereoPair, output_dir: str) -> None:
        """
        Export stereo pair in both coordinate systems.
        
        Args:
            stereo_pair: Stereo camera pair specification
            output_dir: Output directory path
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        pair_name = stereo_pair.get_pair_name()
        
        # Generate both coordinate system versions
        for coord_sys in CoordinateSystem:
            try:
                config = self.generate_stereo_extrinsics(
                    stereo_pair.cam0_label, 
                    stereo_pair.cam1_label, 
                    coord_sys
                )
                
                # Create filename
                suffix = "slam_origin" if coord_sys == CoordinateSystem.SLAM_LEFT_ORIGIN else "cam0_origin"
                filename = f"{pair_name}_{suffix}_extrinsics.yaml"
                output_file = output_path / filename
                
                # Write YAML with custom formatting
                self._write_formatted_yaml(config, output_file, stereo_pair)
                
                baseline_cm = config['metadata']['baseline_distance_cm']
                print(f"✓ {coord_sys.value:15} | {pair_name:20} | {baseline_cm:6.2f} cm | {filename}")
                
            except ValueError as e:
                print(f"✗ {coord_sys.value:15} | {pair_name:20} | ERROR: {e}")
    
    def _write_formatted_yaml(self, config: Dict, output_file: Path, stereo_pair: StereoPair) -> None:
        """Write YAML with custom formatting and comprehensive comments"""
        
        coord_sys = config['metadata']['coordinate_system']
        baseline_cm = config['metadata']['baseline_distance_cm']
        origin_camera = config['metadata']['origin_camera']
        
        with open(output_file, 'w', encoding='utf-8') as f:
            # Header comments
            f.write("# Aria Stereo Camera Extrinsics\n")
            f.write(f"# Camera Pair: {stereo_pair.cam0_label} <-> {stereo_pair.cam1_label}\n")
            f.write(f"# Description: {stereo_pair.description}\n")
            f.write(f"# Coordinate System: {config['metadata']['description']}\n")
            f.write(f"# Baseline Distance: {baseline_cm:.2f} cm\n")
            f.write(f"# Device: {config['metadata']['device_info']['type']} ({config['metadata']['device_info']['serial']})\n")
            f.write("#\n")
            f.write("# T_world_camera: Transformation from world coordinate to camera coordinate\n")
            f.write("# For stereo rectification: use relative transformation between cameras\n")
            f.write("#\n\n")
            
            # Metadata section
            f.write("metadata:\n")
            for key, value in config['metadata'].items():
                if key == 'device_info':
                    f.write(f"  {key}:\n")
                    for sub_key, sub_value in value.items():
                        f.write(f"    {sub_key}: {repr(sub_value)}\n")
                else:
                    f.write(f"  {key}: {repr(value)}\n")
            f.write("\n")
            
            # Extrinsics section
            f.write("extrinsics:\n")
            
            # cam0
            cam0_data = config['extrinsics']['cam0']
            f.write(f"  cam0:  # {cam0_data['label']}\n")
            if coord_sys == 'slam_left_origin':
                f.write(f"    # Pose relative to SLAM-LEFT camera coordinate system\n")
            else:
                f.write(f"    # Origin camera - identity transformation\n")
            
            f.write(f"    label: {repr(cam0_data['label'])}\n")
            f.write(f"    T_type: {cam0_data['T_type']}  # 0 for Euroc format\n")
            f.write("    T_world_camera:\n")
            
            T0 = cam0_data['T_world_camera']
            for i, row in enumerate(T0):
                if i < 3:
                    translation_comments = [
                        "  # Translation X",
                        "  # Translation Y", 
                        "  # Translation Z"
                    ]
                    f.write(f"    - {row}{translation_comments[i]}\n")
                else:
                    f.write(f"    - {row}  # Homogeneous coordinates\n")
            f.write("\n")
            
            # cam1
            cam1_data = config['extrinsics']['cam1']
            f.write(f"  cam1:  # {cam1_data['label']}\n")
            if coord_sys == 'slam_left_origin':
                f.write(f"    # Pose relative to SLAM-LEFT camera coordinate system\n")
            else:
                f.write(f"    # Relative pose: cam1 relative to cam0\n")
            
            f.write(f"    label: {repr(cam1_data['label'])}\n")
            f.write(f"    T_type: {cam1_data['T_type']}  # 0 for Euroc format\n")
            f.write("    T_world_camera:\n")
            
            T1 = cam1_data['T_world_camera']
            for i, row in enumerate(T1):
                if i < 3:
                    translation_comments = [
                        f"  # Translation X: {row[3]:.6f}m",
                        f"  # Translation Y: {row[3]:.6f}m",
                        f"  # Translation Z: {row[3]:.6f}m"
                    ]
                    f.write(f"    - {row}{translation_comments[i]}\n")
                else:
                    f.write(f"    - {row}  # Homogeneous coordinates\n")
    
    def process_all_pairs(self, output_dir: str = "aria_stereo_extrinsics") -> None:
        """
        Process all standard stereo pairs and generate extrinsics files.
        
        Args:
            output_dir: Output directory for generated files
        """
        print(f"\n=== Aria Stereo Extrinsics Processor ===")
        print(f"Output Directory: {output_dir}")
        print(f"Processing {len(self.STANDARD_PAIRS)} stereo pairs...\n")
        
        print(f"{'Coordinate System':<15} | {'Camera Pair':<20} | {'Baseline':<8} | {'Output File'}")
        print("-" * 80)
        
        for stereo_pair in self.STANDARD_PAIRS:
            # Check if both cameras exist
            if (stereo_pair.cam0_label in self.camera_poses and 
                stereo_pair.cam1_label in self.camera_poses):
                self.export_stereo_pair(stereo_pair, output_dir)
            else:
                missing_cameras = []
                if stereo_pair.cam0_label not in self.camera_poses:
                    missing_cameras.append(stereo_pair.cam0_label)
                if stereo_pair.cam1_label not in self.camera_poses:
                    missing_cameras.append(stereo_pair.cam1_label)
                
                pair_name = stereo_pair.get_pair_name()
                print(f"✗ {'N/A':<15} | {pair_name:<20} | {'N/A':<8} | Missing: {', '.join(missing_cameras)}")
        
        print(f"\n✓ Processing complete! Files saved to: {output_dir}/")


def main():
    """Process Aria stereo extrinsics extraction with default settings"""
    
    # Default configuration
    calibration_file = "aria_workspace/device_calibration.json"
    output_directory = "aria_stereo_extrinsics"
    
    try:
        # Initialize processor
        processor = AriaExtrinsicsProcessor(calibration_file)
        
        # Process all standard pairs
        processor.process_all_pairs(output_directory)
        
        return 0
        
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    exit(main())